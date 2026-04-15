#ifndef slic3r_PlugGenerator_hpp_
#define slic3r_PlugGenerator_hpp_

#include "TriangleMesh.hpp"
#include "HoleFinder.hpp"

namespace Slic3r {

// Default fill depth in mm.
constexpr float PLUG_DEFAULT_DEPTH_MM = 1.0f;

// Generate a watertight "plug" mesh that fills a hole boundary.
//
// The plug is a solid extrusion:
//   - Front cap: flush with the hole surface (coplanar with the boundary loop).
//   - Side walls: connecting front and back caps.
//   - Back cap: offset inward by `depth` along the negative plane normal.
//
// The resulting mesh can be added as a ModelVolume assigned to a second extruder,
// effectively filling the hole with a different filament color.
//
// Parameters:
//   boundary  - The HoleBoundary detected by HoleFinder.
//   depth     - How deep the plug extends into the model (mm). Default: 1.0mm.
//
// Returns:
//   A watertight TriangleMesh representing the plug, positioned in the same
//   coordinate space as the original mesh.
TriangleMesh generate_plug(const HoleBoundary &boundary, float depth = PLUG_DEFAULT_DEPTH_MM);

// Generate a small "negative" plug that covers an inner loop (island).
// This mesh is meant to be added as a NEGATIVE_VOLUME to subtract the island
// area from the main plug, preventing the slicer's boolean union from filling
// the island with the plug's extruder.
//
// Parameters:
//   inner_loop    - The 3D vertices of the inner loop (island boundary).
//   plane_normal  - The outward-facing normal of the hole's plane.
//   plane_origin  - A point on the hole's plane (used for 2D projection).
//   depth         - How deep the negative volume extends (should match plug depth).
//
// Returns:
//   A watertight TriangleMesh representing the negative volume.
TriangleMesh generate_island_negative(const std::vector<Vec3f> &inner_loop,
                                      const Vec3f &plane_normal,
                                      const Vec3f &plane_origin,
                                      float depth = PLUG_DEFAULT_DEPTH_MM);

// ─── shared geometry helpers (used by PlugGenerator and hover preview) ────────

// Build a right-handed coordinate frame (u, v) on the plane defined by `normal`.
// Guarantees: u × v == normal (approximately), |u| == |v| == 1.
void build_plane_frame(const Vec3f &normal, Vec3f &u_out, Vec3f &v_out);

// Project a 3D point onto a 2D plane coordinate system defined by (origin, u, v).
Vec2d project_to_2d(const Vec3f &point, const Vec3f &origin,
                    const Vec3f &u, const Vec3f &v);

// Unproject a 2D plane coordinate back to 3D.
Vec3f unproject_to_3d(const Vec2d &pt2d, const Vec3f &origin,
                      const Vec3f &u, const Vec3f &v);

} // namespace Slic3r

#endif // slic3r_PlugGenerator_hpp_
