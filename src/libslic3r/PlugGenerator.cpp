#include "PlugGenerator.hpp"
#include "ExPolygon.hpp"
#include "Tesselate.hpp"
#include "libslic3r.h" // for SCALING_FACTOR

#include <boost/log/trivial.hpp>

#include <cassert>
#include <cmath>
#include <algorithm>

namespace Slic3r {

// ─── helpers ──────────────────────────────────────────────────────────────────

// Build a coordinate frame on the plane defined by `normal`.
// Returns two orthonormal tangent vectors (u, v) such that (u, v, normal) is
// a right-handed frame.
static void build_plane_frame(const Vec3f &normal, Vec3f &u_out, Vec3f &v_out)
{
    // Pick a vector not parallel to normal.
    Vec3f arbitrary = (std::abs(normal.x()) < 0.9f) ? Vec3f(1, 0, 0) : Vec3f(0, 1, 0);
    u_out = normal.cross(arbitrary).normalized();
    v_out = normal.cross(u_out).normalized();
}

// Project a 3D point onto a 2D plane coordinate system.
static Vec2d project_to_2d(const Vec3f &point, const Vec3f &origin,
                           const Vec3f &u, const Vec3f &v)
{
    Vec3f rel = point - origin;
    return Vec2d((double)rel.dot(u), (double)rel.dot(v));
}

// Unproject a 2D plane coordinate back to 3D.
static Vec3f unproject_to_3d(const Vec2d &pt2d, const Vec3f &origin,
                             const Vec3f &u, const Vec3f &v)
{
    return origin + (float)pt2d.x() * u + (float)pt2d.y() * v;
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
    Polygon poly_2d;
    poly_2d.points.reserve(n);
    for (int i = 0; i < n; ++i) {
        Vec2d p = project_to_2d(loop[i], origin, u, v);
        // Slic3r's Polygon uses scaled integer coordinates.
        poly_2d.points.emplace_back(Point(scale_(p.x()), scale_(p.y())));
    }

    // Ensure CCW orientation (for correct triangulation normals).
    if (poly_2d.is_clockwise())
        poly_2d.reverse();

    // Wrap in ExPolygon with inner loops (islands) as holes.
    ExPolygon expoly;
    expoly.contour = std::move(poly_2d);

    // Add inner loops (e.g., the counter inside letter "A") as holes in the ExPolygon.
    // These regions will NOT be filled — they stay as part of the original surface.
    for (const auto &inner : boundary.inner_loops) {
        Polygon hole_2d;
        hole_2d.points.reserve(inner.size());
        for (const Vec3f &pt : inner) {
            Vec2d p = project_to_2d(pt, origin, u, v);
            hole_2d.points.emplace_back(Point(scale_(p.x()), scale_(p.y())));
        }
        // Holes in ExPolygon must be CW (opposite of contour).
        if (hole_2d.is_counter_clockwise())
            hole_2d.reverse();
        expoly.holes.push_back(std::move(hole_2d));
    }

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
    // Vertices layout:
    //   [0,      n-1]       : front cap ring (flush with surface)
    //   [n,      2n-1]      : back cap ring  (inset by depth)
    //   [2n,     2n+Nt*3-1] : front cap triangulation vertices (may not align with ring)
    //   [2n+Nt*3, ...]      : back cap triangulation vertices
    //
    // Actually, let's simplify: since the triangulated cap vertices are in 2D
    // and we need them in 3D, and the ring vertices are the boundary loop itself,
    // we'll build separate vertex arrays and merge.

    std::vector<Vec3f> vertices;
    std::vector<Vec3i32> faces;

    Vec3f offset = -normal * depth;  // Inward direction.

    // ── 3a: Front ring vertices [0 .. n-1] ──
    for (int i = 0; i < n; ++i)
        vertices.push_back(loop[i]);

    // ── 3b: Back ring vertices [n .. 2n-1] ──
    for (int i = 0; i < n; ++i)
        vertices.push_back(loop[i] + offset);

    // ── 3c: Side walls for outer boundary ──
    // Connect front ring to back ring with two triangles per edge.
    for (int i = 0; i < n; ++i) {
        int i_next = (i + 1) % n;
        int f0 = i;           // front current
        int f1 = i_next;      // front next
        int b0 = n + i;       // back current
        int b1 = n + i_next;  // back next

        // Two triangles forming a quad.
        // Winding: outward-facing sides.
        faces.push_back(Vec3i32(f0, b0, f1));
        faces.push_back(Vec3i32(f1, b0, b1));
    }

    // ── 3c-2: Side walls for inner loops (island holes) ──
    // Each inner loop needs its own side wall ring, with reversed winding
    // (the "outside" of an inner hole faces inward toward the hole center).
    for (const auto &inner : boundary.inner_loops) {
        int in_n = (int)inner.size();
        if (in_n < 3) continue;

        int inner_front_base = (int)vertices.size();
        for (int i = 0; i < in_n; ++i)
            vertices.push_back(inner[i]);

        int inner_back_base = (int)vertices.size();
        for (int i = 0; i < in_n; ++i)
            vertices.push_back(inner[i] + offset);

        for (int i = 0; i < in_n; ++i) {
            int i_next = (i + 1) % in_n;
            int f0 = inner_front_base + i;
            int f1 = inner_front_base + i_next;
            int b0 = inner_back_base + i;
            int b1 = inner_back_base + i_next;

            // Reversed winding compared to outer walls (faces inward).
            faces.push_back(Vec3i32(f0, f1, b0));
            faces.push_back(Vec3i32(f1, b1, b0));
        }
    }

    // ── 3d: Front cap triangles ──
    // Convert triangulated 2D points back to 3D and add as vertices.
    int front_cap_base = (int)vertices.size();
    for (const Vec2d &p : tri_pts_2d)
        vertices.push_back(unproject_to_3d(p, origin, u, v));

    for (int t = 0; t < num_cap_tris; ++t) {
        int base = front_cap_base + t * 3;
        // Normal should point outward (same direction as plane_normal).
        faces.push_back(Vec3i32(base, base + 1, base + 2));
    }

    // ── 3e: Back cap triangles ──
    // Same shape but offset inward, with reversed winding.
    int back_cap_base = (int)vertices.size();
    for (const Vec2d &p : tri_pts_2d)
        vertices.push_back(unproject_to_3d(p, origin, u, v) + offset);

    for (int t = 0; t < num_cap_tris; ++t) {
        int base = back_cap_base + t * 3;
        // Reversed winding so normal points inward (opposite of plane_normal).
        faces.push_back(Vec3i32(base, base + 2, base + 1));
    }

    // ── Step 4: Build TriangleMesh ──────────────────────────────────────
    indexed_triangle_set its;
    its.vertices = std::move(vertices);
    its.indices  = std::move(faces);

    // Merge duplicate vertices to clean up the mesh.
    its_merge_vertices(its);

    TriangleMesh mesh(std::move(its));
    return mesh;
}

} // namespace Slic3r
