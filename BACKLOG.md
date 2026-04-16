# Hole Fill Color Tool — Issue Backlog

Tracking deferred issues, improvements, and TODOs for the OrcaSlicer Hole Fill Color feature.
Branch: `feature/hole-fill-color` on `https://github.com/jedundon/OrcaSlicer`

## Legend
- 🔴 Bug / Correctness
- 🟡 Robustness / Edge Case
- 🟢 Polish / Nice-to-Have
- ⬜ Not Started | 🟨 In Progress | ✅ Done

---

## 🔴 Bugs / Correctness

### HF-01: No guard against duplicate hole fills ✅ `(pending commit)`
Clicking the same hole twice creates two overlapping HoleFill volumes.
- **Fix:** Bounding-box-center comparison (100µm threshold) before adding. Shows notification "This hole is already filled" and skips. User can remove existing volume from object list to refill.
- **File:** `GLGizmoMmuSegmentation.cpp` → `perform_hole_fill()`

### HF-02: Stale `hit_volume` pointer in render_hole_fill_hover ⬜
If `mo->volumes` changes between frames (e.g., user adds/deletes a HoleFill volume while hovering), the cached `m_hover_mesh_id` could point to the wrong volume. Low probability race.
- **File:** `GLGizmoMmuSegmentation.cpp` → `render_hole_fill_hover()`
- **Severity:** Low — would need volumes to change mid-hover

### HF-03: Inner loop side wall winding order in PlugGenerator ⬜
Outer walls use `(f0, b0, f1), (f1, b0, b1)` and inner walls use reversed order. If inner loop winds opposite to expected, wall normals could point inward, causing slicer artifacts (gaps, wrong infill).
- **File:** `PlugGenerator.cpp`
- **Severity:** Medium — could produce bad slicing output. Needs a test print to verify.

### HF-04: Coordinate space — plug placement with non-origin-centered models ⬜
Plug vertices are in volume-local space, transform copies source volume. Reviewed and appears correct, but needs real-world validation with translated/rotated models.
- **File:** `GLGizmoMmuSegmentation.cpp` → `perform_hole_fill()`
- **Severity:** Medium — untested scenario

### HF-05: Gizmo reinit after adding volume ⬜
`m_triangle_selectors` may be stale after adding a HoleFill volume. Could cause crashes or wrong painting if user continues without re-entering the gizmo.
- **Decision:** Document as known limitation for now ("re-enter gizmo after filling"). Long-term fix is HF-30 (separate gizmo).
- **File:** `GLGizmoMmuSegmentation.cpp`
- **Severity:** Medium

---

## 🟡 Robustness / Edge Cases

### HF-10: `point_in_loop_2d` centroid can fail for non-convex loops ⬜
Centroid of a non-convex polygon (C, S, crescent shapes) can lie outside the polygon, causing containment test to give wrong results. Affects both connected and disconnected island detection.
- **Fix:** Use bounding-box center, or sample multiple points with majority vote.
- **File:** `HoleFinder.cpp`

### HF-11: `its_face_neighbors` recomputed on every hover ⬜ 🔴 HIGH PRIORITY (when needed)
O(n) operation over all faces, called fresh on every facet change. For high-poly models (100k+ faces) this could cause hover lag.
- **Fix:** Cache neighbor table per mesh (invalidate on mesh change).
- **File:** `HoleFinder.cpp` + `GLGizmoMmuSegmentation.cpp`
- **Deferred:** 2026-04-16. Fine on current test models (~1200 faces). Revisit when testing on 50k+ face models.
- **See also:** HF-12 (same root cause — combined they make hover O(n) per facet change)

### HF-12: Step 7 scans ALL mesh faces on every hover ⬜ 🔴 HIGH PRIORITY (when needed)
Disconnected island detection iterates every face in the mesh. Combined with HF-11, this is O(n) per facet change. Fine for ~1200 faces, could lag on complex models.
- **Fix:** Cache `find_hole_boundaries()` results keyed by `(mesh_id, seed_facet_idx)`. Results are stable unless mesh changes. Invalidate on volume add/remove.
- **File:** `HoleFinder.cpp` + `GLGizmoMmuSegmentation.cpp`
- **Deferred:** 2026-04-16. Adds complexity that could mask bugs during correctness iteration.

### HF-13: E letter (open channels) cannot be detected ⬜
Letters whose strokes are open channels connected to the face perimeter have no enclosed boundary loops. The boundary-loop approach fundamentally can't detect these.
- **Fix:** Would need depth-based detection (find recessed bottom faces and work upward). Separate feature.
- **Severity:** UX limitation — affects E, F, L, T, and similar open-channel engravings

### HF-14: A counter bottom false positive (seed 646) ⬜
Hovering directly on the tiny flat face at the bottom of the A's counter (11 faces) shows a small triangle highlight. Technically correct (valid face with valid interior loop), but not what the user intends to fill.
- **Fix:** Minimum area filter (HF-20) would catch this.

### HF-15: Zero-area degenerate loops not handled ⬜
If a loop has zero area (degenerate triangle strip), it could cause division-by-zero or NaN in area calculations.
- **File:** `HoleFinder.cpp`

---

## 🟢 Polish / Nice-to-Have

### HF-20: Add minimum area filter ⬜
Very small holes (< 1 mm²) are detected and offered for fill. A configurable minimum area threshold would improve UX by filtering noise.
- **File:** `HoleFinder.cpp`

### HF-21: Debug logging at `warning` level — should be `debug`/`trace` ✅ `f80654bc`
All `[HoleFinder]` and `[PlugGen]` log lines switched from `warning` → `debug`.
- **File:** `HoleFinder.cpp`, `PlugGenerator.cpp`

### HF-22: `build_plane_frame` duplicated ✅ `f80654bc`
Extracted to public API in `PlugGenerator.hpp`. Hover renderer now calls shared helper.
- **Files:** `PlugGenerator.cpp`, `PlugGenerator.hpp`, `GLGizmoMmuSegmentation.cpp`

### HF-23: ExPolygon tessellation code duplicated — partially done ✅ `f80654bc`
`project_to_2d()` and `unproject_to_3d()` extracted to shared header. Full `boundary_to_expolygon()` helper still TODO.
- **Files:** `PlugGenerator.cpp`, `PlugGenerator.hpp`, `GLGizmoMmuSegmentation.cpp`

### HF-24: Tool button icon is placeholder ⬜
PUA codepoint `0xF0FF` renders as `?`. Need to create a proper SVG icon for the Hole Fill tool.
- **File:** Icon font + `GLGizmoMmuSegmentation.cpp`

### HF-25: Disable white triangle outline during hole fill hover ⬜
James noted the default white facet outline is distracting during hole fill hover. Could suppress it when a valid hole preview is showing.
- **File:** `GLGizmoMmuSegmentation.cpp`

### HF-26: Add ribbon region detection heuristic ⬜
Area/perimeter ratio could identify ribbon/strip regions (thin wall faces) for better UX feedback — e.g., "this is a wall, not a fillable hole."
- **File:** `HoleFinder.cpp`

### HF-27: Add Step 7 summary logging ⬜
Log how many coplanar regions were scanned and how many matched as disconnected islands.
- **File:** `HoleFinder.cpp`

---

## ✅ Completed

### HF-C1: Side face perimeter depth bug ✅ `3ebd41ba`
Complex non-convex perimeter caused point-in-polygon false positive. Fixed by forcing perimeter to depth 0.

### HF-C2: Disconnected island detection (A's counter) ✅ `307b7533`
A's counter triangle was separate coplanar region not found by main flood-fill. Added Step 7 to scan mesh for disconnected coplanar islands.

### HF-C3: MSVC Eigen overload ambiguity ✅ `eafe9b2e`
Vec3f cast fix for `add_vertex()` calls.

### HF-C4: Forward reference to lambda ✅ `3a8a73cf`
`loop_centroid_2d` used before defined in debug logging.

---

---

## 🏗️ Architecture / Future

### HF-30: Extract Hole Fill into a separate gizmo ⬜ 🔴 HIGH PRIORITY
Hole filling is fundamentally a **modeling operation** (creates real ModelVolumes), not a painting operation (assigns extruders to existing triangles). Housing it inside the MMU painting gizmo causes:
- Stale triangle selectors after volume add (HF-05)
- Tangled undo between paint strokes and volume additions
- Confusing UX (filled volumes appear in object list as real parts, but come from a "painting" tool)
- Shared state conflicts blocking further improvements

Extracting to a dedicated gizmo (own toolbar icon, own render loop, own lifecycle) resolves all of these.
- **Blocks:** HF-05 proper fix, HF-01 replace option, clean undo/redo
- **Depends on:** Core feature being proven and stable first
- **Decision date:** 2026-04-16, agreed with James

---

*Last updated: 2026-04-16 04:54 UTC*
