# HF-34 — Batch Paint Design for `GLGizmoHoleFill`

**Status:** Draft for review
**Author:** Hole-Fill workstream
**Target:** OrcaSlicer `feature/hole-fill-color`
**Depends on:** HF-30 (standalone gizmo), HF-31 (remove), HF-32 (shift-click scope)
**Parallel to:** HF-33 (cut-mode plugs)

---

## 0. Motivation and scope

The canonical stress-test is **Boggle**: 81 identical dice, each with 6 faces, each face with a recessed letter. A single game = **486 holes** that must all be re-colored with the letter filament. Today each hole requires: click → verify hover → click to commit → wait for `Plater::update()` → next. At ~2 s per hole with a fast operator, that is ~16 minutes of undifferentiated clicking, 486 undo entries, and 486 separate `HoleFill_*` volumes with no cross-object consistency guarantee.

HF-34 turns the single-click flow into a batch flow: *pick one hole as a reference, preview every matching hole across the selection, confirm once, commit as one undo snapshot*. The feature must degrade gracefully to single-click behavior when a single object is selected.

**Non-goals:** (a) cross-object automatic de-duplication of plug meshes (tracked separately), (b) persisting batch selections into the 3MF (the result is plain `ModelVolume`s — they persist already), (c) batch paint across *different* source geometries (different mesh topologies). We match within a logical object / its instances.

---

## 1. Recommended UX flow

### 1.1 Activation

Batch mode is **implicit, not modal**. The gizmo already activates on `selection.is_single_full_instance() || selection.is_any_volume()` (see `GLGizmoHoleFill::on_is_activable`, `GLGizmoHoleFill.cpp:57`). We extend activation to also accept `selection.is_multiple_full_instance()` and `selection.is_multiple_full_object()`. When the selection count is > 1, the gizmo is in *batch mode*:

- The top of the ImGui panel shows a banner: **"Batch paint: N objects · M instances"**.
- A new dropdown **"Apply to"** appears with three options (see §3).
- A new checkbox **"All instances of each object"** (default ON) appears below the dropdown.
- The "Pending plugs" counter changes label to **"Pending plugs across batch"**.

If only a single object/instance is selected, the panel renders as today — no extra controls, batch path is not taken.

### 1.2 Reference pick

The user hovers and clicks **one** reference hole exactly as in single-click mode. The hover preview (`render_hole_fill_hover`) is unchanged for the clicked face. On `LeftDown && !ShiftDown`:

1. `pick_mesh` + `find_nearest_hole` run on the picked volume as today.
2. Instead of immediately calling `generate_plug` and `add_volume`, the gizmo **captures** the reference hole's `HoleBoundary`, the owning `ModelVolume*`, and the world-space plane normal.
3. The gizmo transitions to a **preview** sub-state (`m_batch_state = Preview`).

### 1.3 Preview

In preview state, the gizmo runs the discovery phase (§2, phase 1) and populates `m_batch_preview` — a flat list of `{object_idx, instance_idx, volume_idx, HoleBoundary, world_plane_origin}` entries. Each entry renders as a translucent fill-colored cap with an outline, reusing the existing `GLModel` machinery from `render_hole_fill_hover`. The reference hole is highlighted in a distinct color (cyan) so the operator can visually verify the match propagated correctly.

The ImGui panel swaps the single **Apply** stub (currently disabled, `GLGizmoHoleFill.cpp:610`) for a three-button row:
- **Apply (M plugs)** — enabled, primary color
- **Re-pick** — clears preview, returns to reference-pick state
- **Cancel** — clears preview, exits gizmo cleanly

The operator can adjust **Fill depth**, **Angle tolerance**, **Apply to** scope, and **All instances** while in preview; each change triggers a lightweight re-discovery and re-renders the previews. Depth changes do NOT re-run discovery (the hole set is invariant under depth); they just re-render the cap visuals at the new offset.

### 1.4 Commit

On **Apply**:

1. A single `Plater::TakeSnapshot snapshot("Batch hole fill color")` wraps the entire commit (§2, phase 3).
2. A blocking, non-dismissable progress dialog appears: "Generating N plugs… [====    ] 42 %" — updated from the parallel plug-gen phase. The dialog exposes a **Cancel** button (see §4 item 10.4).
3. Once the serial commit phase finishes, the progress dialog closes and an aggregated notification appears: *"Batch hole fill: 478 plugs added across 81 objects (8 skipped — see log)."* The skipped count is a link that expands into a scrollable list of `{object_name, reason}` pairs.
4. `Plater::update()` fires once, not per plug.

### 1.5 Undo / redo

Because the entire commit sits under a single `TakeSnapshot`, `Ctrl+Z` rewinds the whole batch in one step. `Ctrl+Y` re-applies it. This is the single most important UX property: a 486-hole operation must not leave 486 undo entries in the stack (which would push genuine prior history out of the ring buffer — Orca's default undo stack is 100 entries).

### 1.6 Summary panel (post-commit)

After commit, the gizmo stays active in a **Summary** sub-state. The panel lists:
- Plugs added (count)
- Objects touched (count)
- Holes skipped (duplicate, degenerate, or error) with one-line reason per skip
- A "Clear and pick again" button that returns to reference-pick state without re-opening the gizmo

Leaving the gizmo (ESC, tool switch) dismisses the summary.

---

## 2. Algorithm outline

### 2.1 Three-phase pipeline

| Phase | Parallelism | Responsibility | Thread-safety posture |
|-------|-------------|----------------|-----------------------|
| 1. Discovery | TBB `parallel_for` over `(object, instance, volume)` tuples | Find all `HoleBoundary`s whose *world-space* plane matches the reference | Read-only access to `indexed_triangle_set`; write into a per-tuple local vector, merge at the end |
| 2. Plug generation | TBB `parallel_for` over discovery results | Call `generate_plug(boundary, depth)` + optional `generate_island_negative` for each match | Local computation; writes to a `std::vector<PendingPlug>` indexed by match id (preallocated, no contention) |
| 3. Commit | **Serial, main thread** | Under one `TakeSnapshot`: for each pending plug, `mo->add_volume(std::move(mesh), MODEL_PART, false)` + `set_new_unique_id()` + name + extruder + transformation | `ModelObject::add_volume` mutates `volumes` — must be single-threaded |

Phase 3 is serial because `ModelObject::add_volume` is not thread-safe and because `ObjectBase`'s unique-id allocator uses a shared counter. Phase 3 also triggers a single `Plater::update()` at the end.

### 2.2 New helper: `find_all_holes_matching_normal`

Added to `src/libslic3r/HoleFinder.hpp`/`.cpp`. Signature:

```cpp
struct HoleMatch {
    size_t       seed_facet;   // first facet found in the coplanar region
    HoleBoundary boundary;     // in volume-local coords
};

// Find every hole boundary on `its` whose plane normal matches `target_world_normal`
// after transformation by `volume_world_trafo`. The returned boundaries are in the
// volume's local coordinate space (ready to feed to generate_plug). Already-filled
// holes (see §4 item 3.x) are NOT filtered here — that is the caller's job.
std::vector<HoleMatch> find_all_holes_matching_normal(
    const indexed_triangle_set& its,
    const Transform3d&          volume_world_trafo,
    const Vec3f&                target_world_normal,
    float                       angle_tolerance_deg,
    NormalMatchScope            scope,           // see §3
    const HoleBoundary*         reference_for_single_surface); // nullable
```

Implementation sketch:

1. Compute `world_normal_matrix = volume_world_trafo.linear().inverse().transpose()` once.
2. For each facet `f`, compute `world_n = (world_normal_matrix * local_n).normalized()`.
3. Keep facet `f` as a *candidate seed* iff `dot(world_n, target_world_normal) > cos(tol)`.
4. Build a `seen_facet` bit-set. For each candidate seed not yet in `seen_facet`, run the existing `find_hole_boundaries` flood-fill from that seed. Mark all traversed facets in `seen_facet`. Append any returned boundaries to the result.
5. Step 4 is itself parallelizable at the per-seed level (TBB `parallel_for` with a concurrent bit-set), but we keep it single-threaded per volume for v1 — v1 parallelism is across volumes/instances, which is already ample.

The reused `find_hole_boundaries` function already does local-normal flood-fill with an angle tolerance; we simply gate its *seeds* by world-space normal match.

### 2.3 Projected timing (Boggle 81 × 6)

On a 16-core Threadripper-class machine, with Boggle cubes ≈ 2k triangles each:

- Phase 1: 81 objects × 6 faces × 1 ms local hole-find ≈ 486 ms wall-clock / 16 cores ≈ **~35 ms**.
- Phase 2: 486 × ~0.5 ms per plug (CDT on a simple cube hole) ≈ 243 ms / 16 cores ≈ **~20 ms**.
- Phase 3: 486 × ~1 ms `add_volume` (serial, dominated by id allocation + vector growth) ≈ **~500 ms**.
- `Plater::update()` + GL rebuild: **~300–500 ms** (one-shot, same cost as today's single plug).

**Total wall-clock projection: ~0.9–1.1 s.** Progress bar is useful but not strictly needed; we still want it for the 4× Boggle (324 cubes, ~4 s) and for plug-gen-dominated cases (thousands of triangles per hole).

---

## 3. Normal filtering strategy

### 3.1 Why world-space, not local-space

The naive implementation would compare `local_normal` on each candidate face against the reference facet's `local_normal` and call it a day. This is wrong: two instances of the same ModelObject placed on the plate with different rotations have the same *local* normals but different *world* normals. The user picked a face pointing **up in world space** — they do not want every face that happens to be +Z in the model's local frame. Likewise, two *different* ModelObjects with different base orientations would fail to match despite having identically-oriented faces in world space.

Therefore the canonical representation is **world-space normal**, and each volume applies its own `inverse_transpose(volume_world_trafo.linear())` to go from local to world.

### 3.2 Instance handling

`volume_world_trafo = instance.get_matrix() * volume.get_matrix()` (exactly the pattern at `GLGizmoHoleFill.cpp:148`). Different instances of the same `ModelObject` therefore have different `volume_world_trafo`, and thus different `world_normal_matrix`. The discovery phase iterates `(object, instance, volume)` tuples, not just `(object, volume)`.

### 3.3 Scope dropdown

Three options, reflecting three levels of "what counts as a match":

| Option | Match predicate | Example use |
|--------|-----------------|-------------|
| **All faces** | Every face on every selected object/instance, regardless of normal | "Paint every recessed feature on every cube." Useful when the operator wants *all* 486 Boggle holes in one shot. |
| **Matching normal** *(default)* | World-space normal within `angle_tolerance` of the reference | "Paint only the top face's letter on every cube." This is the typical batch case. |
| **Single surface** | Same coplanar region as the reference *on the same volume*, plus the corresponding region (same local-space normal) on every instance | "Paint only this specific face on all instances." Most conservative; matches HF-32 shift-click semantics scaled to the batch. |

"Single surface" requires we pass the reference `HoleBoundary` so that, on the reference volume, we only return holes whose plane is coplanar with it. On *other* volumes (same ModelObject), we apply the same local-space test (since instances share geometry). Cross-ModelObject batches cannot use "Single surface" — the option is disabled in that case.

### 3.4 Angle tolerance

The existing slider (1°–30°, default 5°) governs both (a) the coplanar flood-fill within `find_hole_boundaries` and (b) the world-space seed-gate. We keep a single slider for v1. If users complain that the world-gate is too loose vs. the flood-fill too tight, we can split the two.

---

## 4. Edge cases and mitigations

> Ten categories, 55 entries. **[SHIP-BLOCKER]** tags mark the five items that must be fixed before the feature goes out.

### 4.1 Geometry / topology

1. **Non-manifold edge touched by a hole.** `find_hole_boundaries` already returns empty when it can't chain a closed loop; surface as a "skipped: non-manifold" entry in the summary.
2. **Degenerate face (zero-area triangle) at the seed.** Reject seed, try the next candidate in the flood-fill; if none work, skip.
3. **Multi-shell volume where reference shell has the hole.** Flood-fill is topology-limited, so we only find holes on the seeded shell per seed — correct behavior, no mitigation needed.
4. **Holes that span two co-planar volumes joined by a negative volume.** Discovery runs per-volume, so a hole split across two MODEL_PART volumes would be missed. Skip silently in v1; document as a known limitation.
5. **Through-hole (hole pierces the volume).** `generate_plug` extrudes `depth` mm along `-plane_normal`; a through-hole would plug the entrance but the exit remains open, *and* if batch mode later picks the exit face we get two plugs in the same hole from opposite sides. **[SHIP-BLOCKER]** — see 4.1.5a.
6. **Through-hole double-fill.** Mitigation: after phase 1, deduplicate by projecting each boundary's centroid along the world normal onto a shared axis; if two boundaries on the same volume have near-identical projections along orthogonal axes and opposite normals, treat as a through-hole pair and emit only the one facing the reference normal. **[SHIP-BLOCKER]**
7. **Hole with a boundary loop that is a figure-eight (self-intersecting).** HoleFinder shouldn't emit these, but if it does, CDT in the plug generator fails — caller should drop and log.
8. **Cylindrical reference face.** The user clicks on the inside of a cylindrical bore (e.g. a counterbore). The flood-fill rejects non-coplanar faces, so the seeded coplanar region is just one curved-facet strip, producing nonsense boundaries. **[SHIP-BLOCKER]** — detect reference-region curvature in phase 0 and refuse to enter batch mode with a clear error: "Reference face is curved; pick a flat face with a hole."
9. **Reference hole is actually the outer silhouette of the object (object has no hole, user clicked the edge).** `find_nearest_hole` returns false today — we keep that gate.
10. **Two coplanar holes very close together.** Discovery finds both; both get plugged. Correct.
11. **Hole with an island that has its own sub-hole.** Nested islands aren't supported by `HoleBoundary` (one level only). Skip and log.
12. **Mesh with inverted winding** (normals point inward). World-normal comparison still works; plug generator extrudes the wrong direction. Detect via signed volume and reject with "inverted mesh" skip reason.

### 4.2 Scale and coordinate frames

13. **`m_depth` is in mm.** `generate_plug` interprets it as local-space units along `plane_normal`. If the instance is scaled non-uniformly (say 2× in X), the effective world-space plug depth varies per axis. For a Boggle batch where all instances have `scale == 1`, fine. For heterogeneous scales, the plug's world thickness diverges per instance. **[SHIP-BLOCKER]** — document and, in v1, divide `m_depth` by the instance scale component along the plane normal so world depth is consistent. Surface this math in the tooltip: *"Depth is measured in world mm."*
14. **Mirrored instances.** `get_matrix().linear().determinant() < 0`. World normal flips; we handle it naturally via inverse-transpose. Plug winding is preserved because we operate in local space.
15. **World vs. local normal ambiguity in the reference capture.** We must store the reference normal as *world-space* (after applying the reference volume's inverse-transpose), not the raw `boundary.plane_normal` which is local. **[SHIP-BLOCKER]** — easy to get wrong; add a unit test.
16. **Floating-point drift when comparing world normals across many instances.** Use `dot > cos(tol_rad)` not `acos(dot) < tol_rad` to avoid `acos` near ±1 instability.
17. **Very small holes (< 0.5 mm diameter).** `find_nearest_hole` may emit a loop with 3–4 vertices that CDT refuses. Skip with reason.

### 4.3 Volume and object state

18. **Volume is a NEGATIVE_VOLUME / SUPPORT / MODIFIER.** Only iterate `is_model_part()` volumes, mirroring the existing gizmo (`GLGizmoHoleFill.cpp:139`).
19. **Object has zero instances.** Skip object.
20. **Object with `printable == false`.** Include anyway — the user explicitly selected it; the printable flag is about slicing, not editing.
21. **Object currently hidden by the cut gizmo (part of an in-progress cut).** Skip; we can't safely mutate a volume mid-cut.
22. **Source hit-volume already has a `HoleFill_*` duplicate covering the reference hole.** Today's gizmo detects via bounding-box center (`GLGizmoHoleFill.cpp:288–305`). Reuse the same predicate per-match in phase 1; emit "already filled" skip.
23. **Batch would create duplicate names.** Current naming is `HoleFill_<raw_vol_idx>_f<facet>`. For batch, append a running counter: `HoleFill_<raw_vol_idx>_f<facet>_b<k>` so parallel phase-2 ids don't collide even if phase 3 is serial.
24. **User removes one of the selected objects via ObjectList while gizmo is in preview.** Selection changes fire `data_changed` — we invalidate the preview and return to reference-pick state.
25. **Selection changes during phase 3 commit (shouldn't happen — main thread blocked by progress dialog — but defensive).** Snapshot the `object_idx` list before phase 1 and use it for phase 3, ignoring live selection.
26. **Object's mesh is dirty (edits from another gizmo pending).** Force `wxGetApp().plater()->take_snapshot()` before discovery so any pending state is committed.
27. **Sub-part hierarchy deeper than one level** (Orca doesn't really have this, but part assemblies via "Merge" can). Use `mo->volumes` as the flat list — same as today.

### 4.4 Performance and cancellation

28. **User picks "All faces" on 1000-cube selection (6000 candidate holes).** Set a batch-size cap (§7) and refuse with a dialog, or degrade to a chunked commit with sub-snapshots.
29. **Phase 2 plug generation is CPU-bound; no cancellation.** Progress dialog's Cancel button sets an atomic flag polled at the top of each `parallel_for` iteration; we abort and return without committing. **[SHIP-BLOCKER]**
30. **Progress dialog flicker for small batches.** If `N < 20`, skip the dialog and commit inline.
31. **`Plater::update()` is expensive and redundant if called per add.** Call exactly once, after all `add_volume`s (current single-click path already does this).
32. **GL raycaster rebuild every `add_volume`.** `add_volume(..., modify_to_center_geometry=false)` avoids the worst of it, but the raycaster still invalidates. We defer raycaster rebuild until after the commit loop by wrapping the loop in a `Plater::SuppressUpdates` guard (add if missing).
33. **Memory: 486 plugs × ~1 KB each = ~500 KB; no concern.** Thousand-cube cases (~6 MB of plugs) still fine.
34. **Discovery phase re-runs on every depth slider tick.** It shouldn't (depth doesn't affect discovery). Cache discovery results until angle/tolerance/scope/selection changes.
35. **Preview render cost with 486 translucent caps.** Batch the cap geometry into one `GLModel` per frame, not 486. Today's hover render creates one GLModel per hover — OK for N=1, dies at N=500.

### 4.5 UI and input

36. **User double-clicks the reference hole (triggers two LeftDowns).** Debounce: only the first LeftDown after entering reference-pick state transitions to preview.
37. **User presses ESC during preview.** Exit preview; do NOT exit the gizmo. A second ESC exits the gizmo.
38. **User presses ESC during phase-3 commit.** Ignored — commit is atomic under the snapshot.
39. **Ctrl+scroll to change depth while in preview.** Valid. Re-render preview caps at new depth; do NOT re-discover.
40. **Shift+click in batch preview.** Reserved for HF-32 interaction (see §5.2).
41. **Right-click in batch preview.** Context menu with "Exclude this hole from batch" — stretch goal, not v1.
42. **Filament picker changed while in preview.** Re-tint preview caps; do not re-discover.
43. **User switches away to another gizmo mid-preview.** Discard preview silently; no implicit commit.
44. **Re-pick button after a commit.** Returns to reference-pick and clears the preview, but does not undo the prior commit (that's what Ctrl+Z is for).

### 4.6 Undo / redo

45. **Snapshot name too generic.** Use `"Batch hole fill ({N} plugs)"` so the undo dropdown is informative.
46. **Interaction with auto-snapshots.** `Plater::TakeSnapshot` stacks; a nested auto-snapshot from `add_volume` would break atomicity. Verify `add_volume` doesn't auto-snapshot (it doesn't in the current code path).
47. **Undo → Redo → Undo.** Standard undo-redo; nothing special because the snapshot is atomic.
48. **Memory of undo snapshot for very large batches.** A 10,000-plug batch is ~10 MB of mesh data in the undo buffer. Acceptable for v1; add a warning dialog above 5,000.

### 4.7 Persistence and export

49. **Saving to 3MF.** Plugs are plain `MODEL_PART` volumes — save/load works today. No changes needed.
50. **Slicing result.** Each plug is assigned to its selected extruder via `config.set("extruder", ...)`. Slicing picks it up via existing per-volume extruder assignment.
51. **Export with "Cut" applied.** HF-33 interaction (see §5.4).

### 4.8 Preview fidelity

52. **Preview cap renders through walls** (depth-test disabled). Keep today's behavior (`glDepthMask(GL_FALSE)` during translucent pass).
53. **Preview color matches selected extruder, not the final slicer color.** Correct — user controls both.
54. **Preview does not show negative island volumes.** We only preview the outer fill cap; island negatives are generated at commit time and are invisible anyway. Document.

### 4.9 Error paths

55. **Phase 1 finds zero matches.** Notification: "No matching holes found in the selection. Try increasing angle tolerance or changing the scope." Stay in reference-pick state.
56. **Phase 2 throws (CGAL exception, bad geometry).** Catch per-match, log, mark that match as skipped, continue.
57. **Phase 3 `add_volume` returns nullptr.** Log; skip that entry. Continue.

### 4.10 Concurrency

58. **TBB `parallel_for` with nested gizmo code.** The gizmo's main thread owns all GL and GUI state. Phase 1 and 2 touch only `indexed_triangle_set` (read-only) and their own output buffers. No GL calls from worker threads.
59. **Plater update during phase 1/2.** Shouldn't happen — we block the GUI thread on the progress dialog, which pumps its own message loop. Be careful: the progress dialog's message pump must NOT dispatch canvas-paint events that would call into our half-mutated state. Use a `wxBusyCursor` + modal `wxProgressDialog` and skip user idle events.
60. **Cancellation polling granularity.** Poll once per outer `parallel_for` tuple, not inside plug-gen. Worst case: one plug's worth of extra work after cancel. Acceptable.

---

## 5. Interaction with existing features

### 5.1 Single-click (HF-30)

Single-click is the special case `N = 1` on the selection path. We do not route single-click through the 3-phase pipeline — that would add a (tiny) overhead and a needless progress dialog. Instead:
- If selection is single-object/single-instance/single-volume → existing `perform_hole_fill` path.
- If selection is multi → batch path with reference-pick → preview → Apply.

Both paths share `find_nearest_hole`, `generate_plug`, and the duplicate-fill guard.

### 5.2 Shift+click (HF-32)

HF-32 gives shift+click a "scope" meaning: shift+click on a single object extends fill to all coplanar holes on that object. In batch mode, shift+click on the reference pick sets the scope dropdown to **Single surface** for this click (a one-shot override) and proceeds to preview. This makes HF-32 the natural gateway into batch mode for users who already know the shift+click muscle memory: they shift-click on object A, see previews across B, C, D light up, confirm.

Shift+click *during* preview is ignored in v1 (or reserved for "toggle include/exclude this preview cap").

### 5.3 Remove (HF-31)

HF-31 adds a "remove plug" action: click on an existing `HoleFill_*` volume while in the gizmo to delete it. Batch interaction:
- If the user activates batch mode but their reference click lands on an existing `HoleFill_*` volume, we route to the HF-31 remove path (not batch). Batch only triggers on raw-mesh hits.
- A future enhancement: batch-remove via the same reference → preview → Apply loop, using volume-name matching. Tracked separately.

### 5.4 Cut mode (HF-33)

HF-33 changes plug placement to interact with the cut gizmo's plane. Interaction:
- If the cut gizmo is currently previewing a plane on any of the batch-selected objects, **refuse** batch mode with a clear error: "Commit or cancel the pending cut before batch painting." Otherwise the plug geometry would be partially occluded by a transform that hasn't been applied.
- Batch-applied plugs respect HF-33's "embed under cut plane" logic on a per-object basis during phase 2 — the plug generator takes the per-object cut context, not a global one.

---

## 6. Performance budget

Reference target: **Boggle 81 × 6 = 486 holes**, on a mid-range 16-core workstation. Budget allocated per phase below; numbers are projections, to be validated with a micro-benchmark before merge.

| Phase | Wall-clock (ms) | % of total | Notes |
|-------|-----------------|------------|-------|
| Phase 0: reference capture | 5 | 0.5 % | Single `find_nearest_hole`, already measured |
| Phase 1: discovery | 35 | 3.5 % | 486 seed attempts, parallel, ~1 ms each |
| Phase 2: plug generation | 20 | 2 % | 486 `generate_plug`, parallel, ~0.5 ms each |
| Phase 3: serial commit | 500 | 50 % | 486 × `add_volume` + unique-id alloc |
| `Plater::update()` | 300 | 30 % | One-shot GL rebuild |
| GL raycaster rebuild | 140 | 14 % | Inside `Plater::update` |
| **Total** | **~1000** | **100 %** | Operator-visible end-to-end |

**Stretch target: ≤1.5 s at 1000 plugs.** Above 1000, phase 3 dominates and we may need to invest in batched `add_volume` (add N volumes, allocate N ids once, invalidate once). Defer to a follow-up issue.

Micro-benchmark harness (to be added to `tests/slic3rutils/`): synthesize an N-copy grid of a cube-with-hole mesh, run the batch pipeline, assert total wall-clock within budget. Run in CI with `N ∈ {1, 10, 100, 500}`.

---

## 7. Open questions for the product owner

The following nine decisions are product calls, not engineering calls. Flagging them now so review can resolve them before implementation starts.

1. **All-instances default.** Should "All instances of each object" be ON or OFF by default? Recommend ON — the Boggle use case absolutely requires it, and the opposite default would confuse the first-time batch user.
2. **Normal semantics default.** Scope dropdown default: *Matching normal* or *Single surface*? Recommend *Matching normal* — it's the more forgiving option; *Single surface* can be promoted later if users complain about over-matching.
3. **Batch size cap.** Hard cap, soft warning, or none? Recommend soft warning at 500, hard refusal at 5000. Reason: undo-buffer memory + phase-3 wall-clock.
4. **Replace-existing toggle.** When discovery finds a hole that is already filled (duplicate-fill guard hits), the current single-click path just skips and notifies. In batch, do we want an optional **"Replace existing fills"** checkbox that deletes the old plug before adding the new one (useful when the user changed extruders and wants to repaint)? Recommend: yes, as a checkbox OFF by default.
5. **Cancellation policy.** Can the user cancel mid-commit (phase 3)? Recommend: no — phase 3 is atomic. Cancel is only available in phases 1–2, before any state mutation. If users ask for "cancel after some are applied," we can partially commit under a named snapshot — but the atomic-snapshot story is cleaner.
6. **Depth-unit migration.** Today `m_depth` is interpreted as local-space mm. The fix in 4.2.13 converts it to world-space mm. Is this a breaking change for existing `HoleFill_*` volumes (nope — they're already committed) or for muscle memory on future single-click? Recommend: yes, migrate to world-space mm, and add a one-time notification on first run. Label the slider "Depth (mm, world)".
7. **HF-32 composition.** Does shift+click auto-enter batch mode (see 5.2)? If HF-32 ships before HF-34, we need a no-op fallback. If HF-34 ships first, the shift+click gateway is a bonus. Recommend HF-34 be built assuming HF-32 lands in parallel — coordinate with that author.
8. **Telemetry.** Do we want to log batch size, wall-clock time, and skip reasons to the Orca telemetry channel (opt-in)? Recommend: yes, anonymized counts + wall-clock. Critical for prioritizing future perf work.
9. **Default plug per-batch color.** Should batch default to the filament picked in the gizmo (current behavior) or prompt for a color each batch? Recommend: current behavior. The picker is already prominent and already governs the preview tint.

---

## Appendix A — state machine

```
          ┌─────────────────┐
          │   Idle / pick   │ ← entry
          └────────┬────────┘
                   │ LeftDown on raw mesh (no shift)
                   ▼
          ┌─────────────────┐
          │ Reference hole  │
          │ captured        │
          └────────┬────────┘
                   │ immediately (if batch selection)
                   ▼
          ┌─────────────────┐
          │ Discovery +     │  (phases 1+2, threaded)
          │ Plug preview    │
          └────┬──────┬─────┘
         Apply │      │ Re-pick / selection change / ESC
               │      ▼
               │   back to Idle
               ▼
          ┌─────────────────┐
          │ Commit          │  (phase 3, under snapshot)
          └────────┬────────┘
                   │ done
                   ▼
          ┌─────────────────┐
          │ Summary         │
          └────────┬────────┘
                   │ Clear / tool switch
                   ▼
                 Idle
```

## Appendix B — file-by-file change plan

| File | Change |
|------|--------|
| `src/libslic3r/HoleFinder.hpp/.cpp` | Add `NormalMatchScope` enum + `find_all_holes_matching_normal`. |
| `src/slic3r/GUI/Gizmos/GLGizmoHoleFill.hpp` | Add batch state enum, preview struct vector, scope/all-instances config, cancellation flag. |
| `src/slic3r/GUI/Gizmos/GLGizmoHoleFill.cpp` | Branch in `perform_hole_fill` for batch; add `discover_batch`, `commit_batch`, progress dialog, summary panel. |
| `src/slic3r/GUI/Plater.cpp` | (Possibly) add a `SuppressUpdates` RAII guard. Verify `TakeSnapshot` composes with nested `add_volume` correctly. |
| `tests/slic3rutils/` | Add `test_hole_finder_batch.cpp` (unit) + perf harness. |
| `tests/fff_print/` or similar | Golden-file test: Boggle-lite (4 cubes), batch paint, verify plug count + extruder assignment. |

---

*End of HF-34 design document.*
