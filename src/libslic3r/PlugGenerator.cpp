#include "PlugGenerator.hpp"
#include "ExPolygon.hpp"
#include "Triangulation.hpp"
#include "libslic3r.h" // for SCALING_FACTOR

#include <boost/log/trivial.hpp>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <algorithm>

namespace Slic3r {

// ─── helpers ──────────────────────────────────────────────────────────────────

// Build a coordinate frame on the plane defined by `normal`.
// Returns two orthonormal tangent vectors (u, v) such that (u, v, normal) is
// a right-handed frame.
void build_plane_frame(const Vec3f &normal, Vec3f &u_out, Vec3f &v_out)
{
    // Pick a vector not parallel to normal.
    Vec3f arbitrary = (std::abs(normal.x()) < 0.9f) ? Vec3f(1, 0, 0) : Vec3f(0, 1, 0);
    u_out = normal.cross(arbitrary).normalized();
    v_out = normal.cross(u_out).normalized();
}

// Project a 3D point onto a 2D plane coordinate system.
Vec2d project_to_2d(const Vec3f &point, const Vec3f &origin,
                           const Vec3f &u, const Vec3f &v)
{
    Vec3f rel = point - origin;
    return Vec2d((double)rel.dot(u), (double)rel.dot(v));
}

// Unproject a 2D plane coordinate back to 3D.
Vec3f unproject_to_3d(const Vec2d &pt2d, const Vec3f &origin,
                             const Vec3f &u, const Vec3f &v)
{
    return origin + (float)pt2d.x() * u + (float)pt2d.y() * v;
}

// Maximum Z-span (mm) for a single side-wall quad.  Quads whose boundary
// edge spans more than this in Z are subdivided into strips so the slicer's
// horizontal Z-cuts don't hit diagonal triangle edges that would create
// visible sawtooth artifacts.
static constexpr float SIDE_WALL_Z_STEP = 0.05f;  // 50 µm

// ─── main implementation ─────────────────────────────────────────────────────

TriangleMesh generate_plug(const HoleBoundary &boundary, float depth)
{
    const auto &loop = boundary.loop;
    const int n = (int)loop.size();
    if (n < 3 || depth <= 0.f)
        return TriangleMesh();

    Vec3f normal = boundary.plane_normal.normalized();
    Vec3f origin = boundary.plane_origin;

    // Build a 2D coordinate frame on the hole's plane.
    Vec3f u, v;
    build_plane_frame(normal, u, v);

    Vec3f offset = -normal * depth;  // Inward direction.

    // ── Step 1: Build ring vertex arrays with Z-subdivision ─────────────
    // For each boundary edge, if the Z-span exceeds SIDE_WALL_Z_STEP we
    // insert intermediate vertices along the edge.  These intermediates
    // become part of both the cap polygon (as CDT constraint edges) and
    // the side walls, ensuring bit-exact vertex sharing and a watertight
    // manifold mesh after its_merge_vertices.
    //
    // We build parallel arrays:
    //   pts_2d       — scaled integer 2D coordinates for CDT input
    //   pts_3d_front — exact 3D positions on the front face
    //   pts_3d_back  — exact 3D positions on the back face (= front + offset)
    //
    // The outer ring occupies indices [0, outer_ring_size).
    // Each inner ring occupies a contiguous range after that.

    Points              pts_2d;        // CDT input (Slic3r scaled integer coords)
    std::vector<Vec3f>  pts_3d_front;  // exact 3D positions (front face)

    // Track ring structure: each ring is a contiguous range of indices.
    struct Ring {
        uint32_t start;
        uint32_t count;
    };
    std::vector<Ring> rings;  // rings[0] = outer, rings[1..] = inner holes

    // Helper: add a 3D point to the arrays and return its index.
    auto add_point = [&](const Vec3f &pt3d) -> uint32_t {
        uint32_t idx = (uint32_t)pts_2d.size();
        Vec2d p2 = project_to_2d(pt3d, origin, u, v);
        pts_2d.emplace_back(Point(scale_(p2.x()), scale_(p2.y())));
        pts_3d_front.push_back(pt3d);
        return idx;
    };

    // Helper: Z-subdivide edge from pt_a to pt_b and add all points
    // (including pt_a, excluding pt_b since pt_b is the next edge's start).
    auto add_ring_edge = [&](const Vec3f &pt_a, const Vec3f &pt_b) {
        add_point(pt_a);
        float dz = std::abs(pt_b.z() - pt_a.z());
        int n_sub = 1;
        if (dz > SIDE_WALL_Z_STEP)
            n_sub = std::min((int)std::ceil(dz / SIDE_WALL_Z_STEP), 500);
        if (n_sub > 1) {
            for (int s = 1; s < n_sub; ++s) {
                float t = (float)s / (float)n_sub;
                Vec3f mp = pt_a + t * (pt_b - pt_a);
                add_point(mp);
            }
        }
    };

    // ── Outer ring ──
    uint32_t outer_start = (uint32_t)pts_2d.size();
    for (int i = 0; i < n; ++i) {
        int i_next = (i + 1) % n;
        add_ring_edge(loop[i], loop[i_next]);
    }
    uint32_t outer_count = (uint32_t)pts_2d.size() - outer_start;
    rings.push_back({outer_start, outer_count});

    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] boundary.inner_loops.size() = "
        << boundary.inner_loops.size()
        << " (normal=" << normal.x() << "," << normal.y() << "," << normal.z() << ")";
    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Outer ring: " << n << " corners -> "
        << outer_count << " vertices (with Z-subdivision)";

    // ── Inner rings (island holes) ──
    for (size_t il = 0; il < boundary.inner_loops.size(); ++il) {
        const auto &inner = boundary.inner_loops[il];
        int in_n = (int)inner.size();
        if (in_n < 3) continue;
        uint32_t inner_start = (uint32_t)pts_2d.size();
        for (int i = 0; i < in_n; ++i) {
            int i_next = (i + 1) % in_n;
            add_ring_edge(inner[i], inner[i_next]);
        }
        uint32_t inner_count = (uint32_t)pts_2d.size() - inner_start;
        rings.push_back({inner_start, inner_count});
        BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Inner ring " << il << ": "
            << in_n << " corners -> " << inner_count << " vertices";
    }

    uint32_t n_all = (uint32_t)pts_2d.size();

    // Build pts_3d_back from pts_3d_front + offset.
    std::vector<Vec3f> pts_3d_back(n_all);
    for (uint32_t i = 0; i < n_all; ++i)
        pts_3d_back[i] = pts_3d_front[i] + offset;

    // ── Step 2: Canonicalize ring winding for CDT ───────────────────────
    // CDT needs: outer ring CCW, inner rings CW (as 2D integer polygons).
    // Check each ring's winding and reverse if needed.  We also track
    // the original winding for side-wall emission.

    // Compute signed area of a ring in the pts_2d array.
    auto ring_signed_area = [&](const Ring &r) -> double {
        double area = 0;
        for (uint32_t i = 0; i < r.count; ++i) {
            uint32_t j = (i + 1) % r.count;
            const Point &pi = pts_2d[r.start + i];
            const Point &pj = pts_2d[r.start + j];
            area += (double)pi.x() * pj.y() - (double)pj.x() * pi.y();
        }
        return area;  // > 0 = CCW
    };

    // Reverse a ring's entries in all parallel arrays.
    auto reverse_ring = [&](const Ring &r) {
        std::reverse(pts_2d.begin() + r.start, pts_2d.begin() + r.start + r.count);
        std::reverse(pts_3d_front.begin() + r.start, pts_3d_front.begin() + r.start + r.count);
        std::reverse(pts_3d_back.begin() + r.start, pts_3d_back.begin() + r.start + r.count);
    };

    // Outer ring: must be CCW.
    bool outer_was_ccw = (ring_signed_area(rings[0]) > 0);
    if (!outer_was_ccw) {
        reverse_ring(rings[0]);
    }

    // Inner rings: must be CW.
    std::vector<bool> inner_was_ccw;
    for (size_t ri = 1; ri < rings.size(); ++ri) {
        bool is_ccw = (ring_signed_area(rings[ri]) > 0);
        inner_was_ccw.push_back(is_ccw);
        if (is_ccw) {
            reverse_ring(rings[ri]);
        }
    }

    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Winding: outer_was_ccw=" << outer_was_ccw
        << ", inner rings=" << inner_was_ccw.size();

    // ── Step 3: Check for duplicate 2D points ───────────────────────────
    // CDT requires unique points.  Z-subdivision intermediates on edges
    // that differ only in Z could project to the same 2D point.
    // Use the Triangulation::Changes API to handle duplicates.
    Points dup_pts = collect_duplicates(pts_2d);
    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Duplicate 2D points: " << dup_pts.size();

    // ── Step 4: Build constraint half-edges ─────────────────────────────
    Triangulation::HalfEdges half_edges;
    half_edges.reserve(n_all);

    if (dup_pts.empty()) {
        // No duplicates — straightforward edge insertion.
        for (const auto &ring : rings) {
            for (uint32_t i = 0; i < ring.count; ++i) {
                uint32_t a = ring.start + i;
                uint32_t b = ring.start + ((i + 1) % ring.count);
                half_edges.push_back({a, b});
            }
        }
    } else {
        // Remap through changes to handle duplicate points.
        Triangulation::Changes changes = Triangulation::create_changes(pts_2d, dup_pts);
        for (const auto &ring : rings) {
            for (uint32_t i = 0; i < ring.count; ++i) {
                uint32_t a = changes[ring.start + i];
                uint32_t b = changes[ring.start + ((i + 1) % ring.count)];
                if (a != b)  // skip degenerate edges from merged duplicates
                    half_edges.push_back({a, b});
            }
        }
    }

    // Sort lexicographically as required by Triangulation::triangulate.
    std::sort(half_edges.begin(), half_edges.end());
    // Remove exact duplicates (shouldn't happen, but defensive).
    half_edges.erase(std::unique(half_edges.begin(), half_edges.end()), half_edges.end());

    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] CDT input: " << n_all << " points, "
        << half_edges.size() << " constraint edges";

    // ── Step 5: Triangulate with CDT ────────────────────────────────────
    Triangulation::Indices cap_tris;
    if (dup_pts.empty()) {
        cap_tris = Triangulation::triangulate(pts_2d, half_edges);
    } else {
        // Build deduplicated point set for CDT.
        Triangulation::Changes changes = Triangulation::create_changes(pts_2d, dup_pts);
        uint32_t n_unique = *std::max_element(changes.begin(), changes.end()) + 1;
        Points pts_unique(n_unique);
        for (uint32_t i = 0; i < n_all; ++i)
            pts_unique[changes[i]] = pts_2d[i];
        cap_tris = Triangulation::triangulate(pts_unique, half_edges);
        // Remap triangle indices back to original (pick any representative).
        // Build reverse map: unique_idx -> first original_idx
        std::vector<uint32_t> rev_changes(n_unique, 0);
        for (uint32_t i = 0; i < n_all; ++i) {
            // First occurrence wins (the one we stored the point from).
            // Since create_changes maps later dupes to earlier indices,
            // we want the original index that maps to each unique index.
            rev_changes[changes[i]] = i;
        }
        for (auto &tri : cap_tris) {
            tri[0] = rev_changes[tri[0]];
            tri[1] = rev_changes[tri[1]];
            tri[2] = rev_changes[tri[2]];
        }
    }

    int num_cap_tris = (int)cap_tris.size();
    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] CDT produced " << num_cap_tris << " triangles";
    if (num_cap_tris == 0)
        return TriangleMesh();

    // ── Step 6: Build the full plug mesh ────────────────────────────────
    // Vertex layout:
    //   [0, n_all)       = front face vertices (pts_3d_front)
    //   [n_all, 2*n_all) = back face vertices  (pts_3d_back)
    // Cap and side-wall triangles share these vertex indices directly.

    std::vector<Vec3f>   vertices(2 * n_all);
    std::vector<Vec3i32> faces;
    faces.reserve(2 * num_cap_tris + 4 * n_all);  // caps + sidewalls estimate

    for (uint32_t i = 0; i < n_all; ++i) {
        vertices[i]          = pts_3d_front[i];
        vertices[n_all + i]  = pts_3d_back[i];
    }

    // ── 6a: Front cap ──
    // CDT triangles index into pts_2d[0..n_all-1] which maps 1:1 to
    // pts_3d_front.  Front cap normal points along +normal (outward).
    // CDT with CCW outer ring produces CCW triangles viewed from +normal.
    for (const auto &tri : cap_tris) {
        faces.push_back(Vec3i32(tri[0], tri[1], tri[2]));
    }

    // ── 6b: Back cap ──
    // Same triangles but shifted to back-face indices and reversed winding.
    for (const auto &tri : cap_tris) {
        faces.push_back(Vec3i32(
            (int)n_all + tri[0],
            (int)n_all + tri[2],   // reversed winding
            (int)n_all + tri[1]));
    }

    // ── 6c: Side walls ──
    // For each consecutive pair of ring vertices, emit a quad (two triangles).
    // The quad connects front[i]→front[j]→back[j]→back[i].
    //
    // For a CCW outer ring viewed from +normal:
    //   Cap front edge direction: i → j (CCW)
    //   Side wall must share edge j → i (reverse) to be manifold
    //   So side-wall triangles (front face outward): (j, i, n_all+i), (j, n_all+i, n_all+j)
    //   This makes the side-wall outward normal point away from the plug center.
    //
    // For CW inner rings:
    //   Cap front edge direction: i → j (CW)
    //   Side wall must share edge j → i (reverse) — same pattern works!
    //   The outward normal of inner side walls points inward toward the hole,
    //   which is correct (the hole faces inward).

    for (const auto &ring : rings) {
        for (uint32_t i = 0; i < ring.count; ++i) {
            uint32_t fi = ring.start + i;
            uint32_t fj = ring.start + ((i + 1) % ring.count);
            uint32_t bi = n_all + fi;
            uint32_t bj = n_all + fj;

            // Two triangles forming a quad.
            // Front-face of side wall: normal pointing outward from plug.
            faces.push_back(Vec3i32((int)fj, (int)fi, (int)bi));
            faces.push_back(Vec3i32((int)fj, (int)bi, (int)bj));
        }
    }

    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Mesh built: "
        << vertices.size() << " verts, " << faces.size() << " faces"
        << " (normal=" << normal.x() << "," << normal.y() << "," << normal.z() << ")";

    // ── Step 7: Build TriangleMesh ──────────────────────────────────────
    indexed_triangle_set its;
    its.vertices = std::move(vertices);
    its.indices  = std::move(faces);

    // Merge duplicate vertices — should only collapse ring start/end
    // junctions since all other vertices are shared by index.
    int pre_merge_verts = (int)its.vertices.size();
    int pre_merge_faces = (int)its.indices.size();
    its_merge_vertices(its);
    int post_merge_verts = (int)its.vertices.size();
    int post_merge_faces = (int)its.indices.size();
    int degenerate_faces = 0;
    for (const auto &f : its.indices) {
        if (f[0] == f[1] || f[1] == f[2] || f[0] == f[2])
            ++degenerate_faces;
    }

    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Mesh: "
        << pre_merge_verts << "->" << post_merge_verts << " verts, "
        << pre_merge_faces << "->" << post_merge_faces << " faces, "
        << degenerate_faces << " degenerate"
        << " (normal=" << normal.x() << "," << normal.y() << "," << normal.z() << ")";

    // Dump plug mesh to STL in user's temp directory for inspection.
    {
        static int plug_id = 0;
        // Use USERPROFILE\Downloads on Windows, /tmp on Linux.
        std::string tmp_dir;
        const char *userprofile = std::getenv("USERPROFILE");
        if (userprofile)
            tmp_dir = std::string(userprofile) + "\\Downloads";
        else
            tmp_dir = "/tmp";
        const char *tmp = tmp_dir.c_str();
        std::string dump_path = std::string(tmp) + "/holefill_plug_" + std::to_string(plug_id++) + ".stl";
        if (its_write_stl_ascii(dump_path.c_str(), "holefill_plug", its))
            BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Dumped mesh to " << dump_path;
        else
            BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Failed to dump mesh to " << dump_path;
    }

    TriangleMesh mesh(std::move(its));
    return mesh;
}

// ─── island negative volume ──────────────────────────────────────────────────

TriangleMesh generate_island_negative(const std::vector<Vec3f> &inner_loop,
                                      const Vec3f &plane_normal,
                                      const Vec3f &plane_origin,
                                      float depth)
{
    const int n = (int)inner_loop.size();
    if (n < 3 || depth <= 0.f)
        return TriangleMesh();

    Vec3f normal = plane_normal.normalized();
    Vec3f offset = -normal * depth;  // Inward direction (same as plug).

    // Build a simple closed prism: front cap, back cap, side walls.
    // This is a solid volume covering only the island area.
    // When added as NEGATIVE_VOLUME, the slicer subtracts it from the
    // plug+body union, leaving the island area with only the original body.

    // We need to slightly expand the negative volume so it fully covers
    // the island even with floating-point imprecision. Expand by a tiny
    // epsilon along the plane and slightly beyond the surface.
    constexpr float SURFACE_EPS = 0.01f;  // 10 microns past surface

    std::vector<Vec3f> vertices;
    std::vector<Vec3i32> faces;

    // Front ring: slightly outside the surface (past the plug's front cap).
    Vec3f front_offset = normal * SURFACE_EPS;
    for (int i = 0; i < n; ++i)
        vertices.push_back(inner_loop[i] + front_offset);

    // Back ring: slightly past the plug's back cap.
    Vec3f back_offset_total = offset - normal * SURFACE_EPS;
    for (int i = 0; i < n; ++i)
        vertices.push_back(inner_loop[i] + back_offset_total);

    // Determine winding of the inner loop by projecting to 2D and checking area.
    Vec3f u, v;
    build_plane_frame(normal, u, v);

    double signed_area = 0;
    for (int i = 0; i < n; ++i) {
        Vec3f rel_a = inner_loop[i] - plane_origin;
        Vec3f rel_b = inner_loop[(i + 1) % n] - plane_origin;
        double ax = rel_a.dot(u), ay = rel_a.dot(v);
        double bx = rel_b.dot(u), by = rel_b.dot(v);
        signed_area += (ax * by - bx * ay);
    }
    // signed_area > 0 means CCW when viewed from +normal direction.
    bool is_ccw = (signed_area > 0);

    // Front cap: outward normal along +normal requires CCW winding from +normal view.
    // If loop is already CCW, use (0, i, i+1). If CW, use (0, i+1, i).
    for (int i = 1; i < n - 1; ++i) {
        if (is_ccw)
            faces.push_back(Vec3i32(0, i, i + 1));
        else
            faces.push_back(Vec3i32(0, i + 1, i));
    }

    // Back cap: outward normal along -normal (reversed winding from front cap).
    for (int i = 1; i < n - 1; ++i) {
        if (is_ccw)
            faces.push_back(Vec3i32(n, n + i + 1, n + i));
        else
            faces.push_back(Vec3i32(n, n + i, n + i + 1));
    }

    // Side walls: winding must be consistent with outward-facing normals.
    // For CCW front loop, side quads go (f0, f1, b0) and (f1, b1, b0).
    // For CW front loop, reverse: (f0, b0, f1) and (f1, b0, b1).
    for (int i = 0; i < n; ++i) {
        int i_next = (i + 1) % n;
        int f0 = i;
        int f1 = i_next;
        int b0 = n + i;
        int b1 = n + i_next;

        if (is_ccw) {
            faces.push_back(Vec3i32(f0, b0, f1));
            faces.push_back(Vec3i32(f1, b0, b1));
        } else {
            faces.push_back(Vec3i32(f0, f1, b0));
            faces.push_back(Vec3i32(f1, b1, b0));
        }
    }

    indexed_triangle_set its;
    its.vertices = std::move(vertices);
    its.indices  = std::move(faces);

    its_merge_vertices(its);

    BOOST_LOG_TRIVIAL(debug) << "[PlugGen] Island negative: " << n
        << " verts, depth=" << depth << ", eps=" << SURFACE_EPS;

    TriangleMesh mesh(std::move(its));
    return mesh;
}

} // namespace Slic3r
