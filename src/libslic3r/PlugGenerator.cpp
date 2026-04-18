#include "PlugGenerator.hpp"
#include "ExPolygon.hpp"
#include "Tesselate.hpp"
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

// Emit side-wall quads between two boundary-ring edge endpoints,
// subdividing along Z when the edge's Z-span exceeds SIDE_WALL_Z_STEP.
// `front_a / front_b` are the 3D positions of the two consecutive ring
// vertices on the front cap; `back_offset` is the vector from front to back.
// `reverse_winding` flips the triangle winding (used for inner-loop walls).
static void emit_side_wall_quads(
    const Vec3f &front_a, const Vec3f &front_b,
    const Vec3f &back_offset,
    bool         reverse_winding,
    std::vector<Vec3f>    &vertices,
    std::vector<Vec3i32>  &faces)
{
    float dz = std::abs(front_b.z() - front_a.z());

    // Number of subdivisions along this edge.
    // Skip subdivision for near-horizontal edges — their diagonal's Z-shift
    // is already negligible regardless of layer height.
    int n_sub = 1;
    if (dz > SIDE_WALL_Z_STEP)
        n_sub = std::min((int)std::ceil(dz / SIDE_WALL_Z_STEP), 500);

    // We always emit new vertices for the intermediate strip endpoints.
    // The first point coincides with front_a, the last with front_b.
    int base = (int)vertices.size();

    // Push front and back ring for each subdivision point.
    for (int s = 0; s <= n_sub; ++s) {
        float t = (float)s / (float)n_sub;
        Vec3f pt_front = front_a + t * (front_b - front_a);
        vertices.push_back(pt_front);
        vertices.push_back(pt_front + back_offset);
    }
    // Vertex layout at `base`:
    //   base + 2*s     = front point for strip s
    //   base + 2*s + 1 = back  point for strip s

    for (int s = 0; s < n_sub; ++s) {
        int f0 = base + 2 * s;       // front current
        int b0 = base + 2 * s + 1;   // back  current
        int f1 = base + 2 * (s + 1); // front next
        int b1 = base + 2 * (s + 1) + 1; // back next

        if (!reverse_winding) {
            faces.push_back(Vec3i32(f0, b0, f1));
            faces.push_back(Vec3i32(f1, b0, b1));
        } else {
            faces.push_back(Vec3i32(f0, f1, b0));
            faces.push_back(Vec3i32(f1, b1, b0));
        }
    }
}

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

    // ── Step 1: Create a Slic3r Polygon from the boundary loop ──────────
    // Project 3D boundary to 2D, then convert to scaled Slic3r Points.
    // The cap polygon uses the original loop corners (not Z-subdivided)
    // so the tessellator produces clean triangles.  Z-subdivision points
    // are added later as stitching fan triangles to bridge cap edges to
    // the finer side-wall segmentation (see Step 3f).

    Vec3f offset = -normal * depth;  // Inward direction.

    Polygon poly_2d;
    poly_2d.points.reserve(n);
    for (int i = 0; i < n; ++i) {
        Vec2d p = project_to_2d(loop[i], origin, u, v);
        poly_2d.points.emplace_back(Point(scale_(p.x()), scale_(p.y())));
    }

    // Pre-compute Z-subdivision for each boundary edge.  This table is
    // shared by the side-wall emitter and the cap-stitching fan generator.
    // subdiv_pts[i] = list of INTERMEDIATE 3D points between loop[i] and
    // loop[(i+1)%n], NOT including the endpoints.  Empty if n_sub == 1.
    std::vector<std::vector<Vec3f>> subdiv_pts(n);
    for (int i = 0; i < n; ++i) {
        int i_next = (i + 1) % n;
        float dz = std::abs(loop[i_next].z() - loop[i].z());
        int n_sub = 1;
        if (dz > SIDE_WALL_Z_STEP)
            n_sub = std::min((int)std::ceil(dz / SIDE_WALL_Z_STEP), 500);
        if (n_sub > 1) {
            subdiv_pts[i].reserve(n_sub - 1);
            for (int s = 1; s < n_sub; ++s) {
                float t = (float)s / (float)n_sub;
                subdiv_pts[i].push_back(loop[i] + t * (loop[i_next] - loop[i]));
            }
        }
    }

    // Detect original 2D winding before canonicalisation.
    bool outer_is_ccw = poly_2d.is_counter_clockwise();

    // Ensure CCW orientation (for correct triangulation normals).
    if (poly_2d.is_clockwise())
        poly_2d.reverse();

    // Wrap in ExPolygon with inner loops (islands) as holes.
    ExPolygon expoly;
    expoly.contour = std::move(poly_2d);

    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] boundary.inner_loops.size() = "
        << boundary.inner_loops.size()
        << " (normal=" << normal.x() << "," << normal.y() << "," << normal.z() << ")";

    // Add inner loops (e.g., the counter inside letter "A") as holes in the ExPolygon.
    // These regions will NOT be filled — they stay as part of the original surface.
    // Track original winding of each inner loop so side walls match cap edges.
    std::vector<bool> inner_is_ccw;
    inner_is_ccw.reserve(boundary.inner_loops.size());
    for (const auto &inner : boundary.inner_loops) {
        Polygon hole_2d;
        hole_2d.points.reserve(inner.size());
        for (const Vec3f &pt : inner) {
            Vec2d p = project_to_2d(pt, origin, u, v);
            hole_2d.points.emplace_back(Point(scale_(p.x()), scale_(p.y())));
        }
        inner_is_ccw.push_back(hole_2d.is_counter_clockwise());
        // Holes in ExPolygon must be CW (opposite of contour).
        if (hole_2d.is_counter_clockwise())
            hole_2d.reverse();
        expoly.holes.push_back(std::move(hole_2d));
    }

    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Winding: outer_is_ccw=" << outer_is_ccw
        << ", inner_is_ccw count=" << inner_is_ccw.size();

    // ── Step 2: Triangulate the cap face ────────────────────────────────
    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] ExPolygon contour: " << expoly.contour.points.size()
        << " pts, " << expoly.holes.size() << " holes";
    for (size_t hi = 0; hi < expoly.holes.size(); ++hi) {
        BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Hole " << hi << ": "
            << expoly.holes[hi].points.size() << " pts, area="
            << std::abs(expoly.holes[hi].area());
    }

    // Use the existing Slic3r tessellation which handles concave polygons.
    std::vector<Vec2d> tri_pts_2d = triangulate_expolygon_2d(expoly, NORMALS_UP);
    // tri_pts_2d contains groups of 3 points (triangle vertices).
    int num_cap_tris = (int)tri_pts_2d.size() / 3;
    BOOST_LOG_TRIVIAL(warning) << "[PlugGen] Tessellation produced " << num_cap_tris << " triangles";
    if (num_cap_tris == 0)
        return TriangleMesh();

    // ── Step 3: Build the full plug mesh ────────────────────────────────
    // Side-wall quads are emitted by emit_side_wall_quads() which
    // Z-subdivides edges to avoid slicer zigzag artifacts.  Cap
    // triangles come from the tessellated 2D ExPolygon.

    std::vector<Vec3f> vertices;
    std::vector<Vec3i32> faces;

    // ── 3c: Side walls for outer boundary ──
    // Each boundary edge is Z-subdivided by emit_side_wall_quads.
    int total_side_tris = 0;
    for (int i = 0; i < n; ++i) {
        int i_next = (i + 1) % n;
        size_t before = faces.size();
        emit_side_wall_quads(loop[i], loop[i_next], offset,
                             /*reverse_winding=*/!outer_is_ccw, vertices, faces);
        total_side_tris += (int)(faces.size() - before);
    }

    // ── 3c-2: Side walls for inner loops (island holes) ──
    // Inner holes are canonicalised to CW by the ExPolygon builder.
    // If the original inner loop was already CW, side walls match as-is
    // (reverse_winding=true for "inward-facing" normals).
    // If it was CCW, the canonicalisation flipped it, so side walls
    // must NOT reverse (the flip already matches).
    for (size_t idx = 0; idx < boundary.inner_loops.size(); ++idx) {
        const auto &inner = boundary.inner_loops[idx];
        int in_n = (int)inner.size();
        if (in_n < 3) continue;

        // The cap tessellation canonicalises inner holes to CW.  Side walls must
        // produce the REVERSE edge direction at the cap junction for manifold edges.
        // Original CW  -> no flip during canonicalisation -> side wall raw order
        //   matches cap -> side walls need normal winding (reverse=false).
        // Original CCW -> flipped to CW -> cap boundary reversed relative to raw
        //   loop -> side walls must reverse to match (reverse=true).
        bool reverse = (idx < inner_is_ccw.size()) ? inner_is_ccw[idx] : true;
        for (int i = 0; i < in_n; ++i) {
            int i_next = (i + 1) % in_n;
            emit_side_wall_quads(inner[i], inner[i_next], offset,
                                 /*reverse_winding=*/reverse, vertices, faces);
        }
    }

    BOOST_LOG_TRIVIAL(debug) << "[PlugGen] Side walls: " << total_side_tris
        << " triangles (Z-step=" << SIDE_WALL_Z_STEP << "mm)";

    // ── Collect all boundary vertices for snapping ──
    // The 2D→3D round-trip through integer-scaled Slic3r coordinates
    // introduces floating-point drift. Cap vertices that sit on the
    // boundary won't exactly match side-wall vertices, leaving gaps
    // in the mesh. We snap cap vertices to the nearest boundary vertex
    // within a tight tolerance so its_merge_vertices (exact equality)
    // can weld them.
    // Use the original loop corners plus all Z-subdivision intermediates
    // so every cap boundary vertex AND fan vertex can snap.
    std::vector<Vec3f> boundary_pts_front;  // front face positions
    std::vector<Vec3f> boundary_pts_back;   // back face positions
    boundary_pts_front.reserve(n * 32 + 16);
    boundary_pts_back.reserve(n * 32 + 16);
    for (int i = 0; i < n; ++i) {
        boundary_pts_front.push_back(loop[i]);
        boundary_pts_back.push_back(loop[i] + offset);
        for (const Vec3f &sp : subdiv_pts[i]) {
            boundary_pts_front.push_back(sp);
            boundary_pts_back.push_back(sp + offset);
        }
    }
    for (const auto &inner : boundary.inner_loops) {
        for (const Vec3f &pt : inner) {
            boundary_pts_front.push_back(pt);
            boundary_pts_back.push_back(pt + offset);
        }
    }

    // Snap helper: if pt is within tolerance of any boundary vertex,
    // replace it with the exact boundary vertex.
    const float snap_tol_sq = 0.001f * 0.001f;  // 1 µm tolerance
    auto snap_to_boundary = [&](Vec3f &pt, const std::vector<Vec3f> &bpts) {
        for (const Vec3f &bp : bpts) {
            if ((pt - bp).squaredNorm() < snap_tol_sq) {
                pt = bp;
                return;
            }
        }
    };

    // ── 3d/3e: Cap triangles with fan-stitching ──
    // Each cap triangle from the tessellator is checked for boundary edges.
    // If a boundary edge has Z-subdivision intermediates, the cap triangle
    // is replaced by a fan of smaller triangles from the opposite vertex
    // through the subdivision points.  This ensures the cap's boundary
    // edges exactly match the finer side-wall segmentation.
    //
    // Boundary edge detection: an edge (P, Q) is a boundary edge if both
    // P and Q snap to consecutive boundary-ring vertices loop[i] and
    // loop[(i+1)%n].  We build a lookup from (snapped) boundary vertex
    // pairs to the corresponding subdivision-point list.

    // Build lookup: boundary vertex pair → subdivision intermediates.
    // Key: (front_a, front_b) as 3D positions of consecutive ring verts.
    // We'll match snapped cap vertices against this.
    struct BoundaryEdge {
        Vec3f a, b;                // front positions of the two corners
        std::vector<Vec3f> intermediates;  // Z-subdivision points between a and b
    };
    std::vector<BoundaryEdge> outer_boundary_edges;
    outer_boundary_edges.reserve(n);
    for (int i = 0; i < n; ++i) {
        BoundaryEdge be;
        be.a = loop[i];
        be.b = loop[(i + 1) % n];
        be.intermediates = subdiv_pts[i];  // may be empty
        outer_boundary_edges.push_back(std::move(be));
    }
    // Also add inner-loop boundary edges (no Z-subdivision for inner loops
    // since they typically have small dZ, but include for completeness).
    // Inner-loop edges currently have no Z-subdivision, so intermediates
    // will always be empty.

    // Helper: find if two 3D points match a boundary edge (within snap tol)
    // and return the intermediates if found.  Also checks the reversed
    // direction (since the tessellator may reverse polygon winding).
    // `reversed_out` is set to true when the match is in reverse order.
    auto find_boundary_intermediates = [&](const Vec3f &pa, const Vec3f &pb,
                                           bool &reversed_out)
        -> const std::vector<Vec3f>* {
        for (const auto &be : outer_boundary_edges) {
            if ((pa - be.a).squaredNorm() < snap_tol_sq &&
                (pb - be.b).squaredNorm() < snap_tol_sq) {
                reversed_out = false;
                return &be.intermediates;
            }
            if ((pa - be.b).squaredNorm() < snap_tol_sq &&
                (pb - be.a).squaredNorm() < snap_tol_sq) {
                reversed_out = true;
                return &be.intermediates;
            }
        }
        return nullptr;
    };

    // Emit cap triangles for one face (front or back).
    // For each tessellated triangle, check all 3 edges for boundary-edge
    // matches with non-empty intermediates.  If found, replace the triangle
    // with a fan through the subdivision points.
    auto emit_cap_tris = [&](
        const std::vector<Vec3f> &bpts,   // snap targets
        const Vec3f &face_offset,         // Vec3f(0,0,0) for front, offset for back
        bool reverse_winding)             // true for back cap
    {
        for (int t = 0; t < num_cap_tris; ++t) {
            Vec3f v0 = unproject_to_3d(tri_pts_2d[t*3+0], origin, u, v) + face_offset;
            Vec3f v1 = unproject_to_3d(tri_pts_2d[t*3+1], origin, u, v) + face_offset;
            Vec3f v2 = unproject_to_3d(tri_pts_2d[t*3+2], origin, u, v) + face_offset;
            snap_to_boundary(v0, bpts);
            snap_to_boundary(v1, bpts);
            snap_to_boundary(v2, bpts);

            // Check each of the 3 edges for boundary intermediates.
            // verts[e] → verts[(e+1)%3], opposite = verts[(e+2)%3].
            Vec3f tri[3] = {v0, v1, v2};
            int fan_edge = -1;  // which edge (0,1,2) has intermediates
            const std::vector<Vec3f> *intermediates = nullptr;
            bool edge_reversed = false;  // true if cap edge is reversed vs stored

            for (int e = 0; e < 3; ++e) {
                const Vec3f &ea = tri[e];
                const Vec3f &eb = tri[(e + 1) % 3];
                bool rev = false;
                auto *mid = find_boundary_intermediates(ea, eb, rev);
                if (mid && !mid->empty()) {
                    fan_edge = e;
                    intermediates = mid;
                    edge_reversed = rev;
                    break;  // handle one subdivided edge per triangle
                }
            }

            if (fan_edge < 0) {
                // No subdivided boundary edge — emit original triangle.
                int base = (int)vertices.size();
                vertices.push_back(v0);
                vertices.push_back(v1);
                vertices.push_back(v2);
                if (!reverse_winding)
                    faces.push_back(Vec3i32(base, base+1, base+2));
                else
                    faces.push_back(Vec3i32(base, base+2, base+1));
            } else {
                // Replace triangle with fan from opposite vertex through
                // subdivision points along the boundary edge.
                const Vec3f &ea = tri[fan_edge];
                const Vec3f &eb = tri[(fan_edge + 1) % 3];
                const Vec3f &opp = tri[(fan_edge + 2) % 3];

                // Build the full chain along the boundary edge in the
                // same direction as the cap traversal (ea → eb).
                // If edge_reversed, intermediates are stored A→B but
                // the cap traverses B→A, so we reverse the intermediates.
                std::vector<Vec3f> chain;
                chain.reserve(intermediates->size() + 2);
                chain.push_back(ea);
                if (!edge_reversed) {
                    for (const Vec3f &mp : *intermediates)
                        chain.push_back(mp + face_offset);
                } else {
                    for (int mi = (int)intermediates->size() - 1; mi >= 0; --mi)
                        chain.push_back((*intermediates)[mi] + face_offset);
                }
                chain.push_back(eb);

                for (size_t ci = 0; ci + 1 < chain.size(); ++ci) {
                    int base = (int)vertices.size();
                    vertices.push_back(opp);
                    vertices.push_back(chain[ci]);
                    vertices.push_back(chain[ci + 1]);
                    if (!reverse_winding)
                        faces.push_back(Vec3i32(base, base+1, base+2));
                    else
                        faces.push_back(Vec3i32(base, base+2, base+1));
                }
            }
        }
    };

    // Front cap: snap to front boundary, no offset, normal winding.
    emit_cap_tris(boundary_pts_front, Vec3f(0, 0, 0), /*reverse_winding=*/false);

    // Back cap: snap to back boundary, offset applied, reversed winding.
    emit_cap_tris(boundary_pts_back, offset, /*reverse_winding=*/true);

    // ── Step 4: Build TriangleMesh ──────────────────────────────────────
    indexed_triangle_set its;
    its.vertices = std::move(vertices);
    its.indices  = std::move(faces);

    // Merge duplicate vertices to clean up the mesh.
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
