# Hole Fill Color Tool — Technical Design Document

## Overview
Extends OrcaSlicer's Color Painting (MMU Segmentation) gizmo with a "Hole Fill" tool
that lets users click on engraved holes in a mesh and fill them flush with a second
filament color, eliminating bridging artifacts on face-down surfaces.

## Problem
3D-printable models with engraved text (letters, logos) create bridging problems when
printed face-down. Users with multi-color capability (e.g., Bambu AMS) don't need the
engraving — they'd prefer flush multi-color text instead.

## Solution (Option A: Plug Volume)
When the user clicks on a detected hole:
1. **HoleFinder** identifies the hole boundary loop on the planar face
2. **PlugGenerator** creates a watertight plug mesh filling the hole to configurable depth
3. The plug is added as a new `ModelVolume` assigned to the selected extruder

## Architecture

### New Files
| File | Purpose |
|------|---------|
| `src/libslic3r/HoleFinder.hpp/cpp` | Boundary loop detection via flood-fill + edge chaining |
| `src/libslic3r/PlugGenerator.hpp/cpp` | Plug mesh generation via 2D tessellation + extrusion |

### Modified Files
| File | Changes |
|------|---------|
| `src/libslic3r/CMakeLists.txt` | Added new source files |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.hpp` | Added hole fill members, gizmo_event override |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | Added tool button, UI panel, click handler, perform_hole_fill() |

## User Flow
1. Open Color Painting gizmo (must have 2+ filaments configured)
2. Select the **Hole Fill** tool (7th button, or press `J`)
3. Set fill depth (default 1.0mm, range 0.2–5.0mm, Ctrl+scroll to adjust)
4. Click on a hole in the model → plug volume is created with selected filament
5. Slice and print — the hole is now flush with a second color, no bridging

## Parameters
- **Fill depth**: 1.0mm default (configurable 0.2–5.0mm)
- **Angle tolerance**: 5° (for coplanar face flood-fill)

## Algorithms

### HoleFinder
1. Flood-fill from clicked face to collect coplanar region (within angle tolerance)
2. Find boundary edges (edges where neighbor is not in region or doesn't exist)
3. Chain boundary edges into closed loops using a from-vertex map
4. Classify: largest loop = face perimeter; smaller loops = holes
5. Return hole loops with plane normal and centroid

### PlugGenerator
1. Build 2D coordinate frame on the hole's plane
2. Project boundary loop to 2D Slic3r Polygon
3. Tessellate using existing `triangulate_expolygon_2d()` (GLU tessellator)
4. Build front cap (flush), back cap (offset by depth), side walls
5. Merge duplicate vertices, return watertight TriangleMesh

## Known Limitations (MVP)
- Only detects holes on flat/planar faces (within angle tolerance)
- Uses a temporary icon (PUA codepoint) — needs dedicated SVG icon
- Does not handle nested holes (hole within a hole)
- Plug mesh shares the source volume's transformation matrix

## Future Enhancements
- Hover preview (highlight hole boundary before clicking)
- Auto-detect all holes on a face with one click
- Curved surface support
- Undo individual hole fills (currently uses standard undo/redo)
