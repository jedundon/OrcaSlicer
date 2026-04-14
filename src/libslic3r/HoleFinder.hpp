#ifndef slic3r_HoleFinder_hpp_
#define slic3r_HoleFinder_hpp_

#include "Point.hpp"
#include "TriangleMesh.hpp"
#include <vector>

namespace Slic3r {

// Represents a detected hole boundary on a planar face of a mesh.
struct HoleBoundary {
    // Ordered 3D vertices forming the closed boundary loop of the hole.
    std::vector<Vec3f> loop;
    // The plane normal of the surrounding face (pointing outward).
    Vec3f              plane_normal;
    // A point on the plane (centroid of the boundary loop).
    Vec3f              plane_origin;
};

// Find hole boundary loops on the planar face surrounding the given facet.
//
// Algorithm:
//   1. Starting from `seed_facet_idx`, flood-fill to collect all neighboring
//      facets whose normals are within `angle_tolerance_deg` of the seed facet's normal.
//   2. Identify boundary edges of this face region — edges that have no neighbor
//      or whose neighbor is not in the coplanar set.
//   3. Chain boundary edges into closed loops.
//   4. Classify loops: the outermost loop is the face perimeter; inner loops are holes.
//   5. Return the inner loops (holes) as HoleBoundary structs.
//
// Parameters:
//   its                 - The indexed triangle set of the mesh.
//   seed_facet_idx      - Index of the facet the user clicked on or near.
//   angle_tolerance_deg - Maximum angle (degrees) between face normals to consider
//                         them part of the same planar region. Default: 5°.
//
// Returns:
//   Vector of HoleBoundary. Empty if no holes are found on the face region.
std::vector<HoleBoundary> find_hole_boundaries(
    const indexed_triangle_set &its,
    int                         seed_facet_idx,
    float                       angle_tolerance_deg = 5.0f);

// Convenience overload: find the single hole nearest to `hit_point`.
// Returns true and fills `out` if a hole is found, false otherwise.
bool find_nearest_hole(
    const indexed_triangle_set &its,
    int                         seed_facet_idx,
    const Vec3f                &hit_point,
    HoleBoundary               &out,
    float                       angle_tolerance_deg = 5.0f);

} // namespace Slic3r

#endif // slic3r_HoleFinder_hpp_
