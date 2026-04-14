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

} // namespace Slic3r

#endif // slic3r_PlugGenerator_hpp_
