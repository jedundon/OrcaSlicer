# HoleFill Gizmo Review — 2026-04-20

**Branch**: `feature/hole-fill-color` @ `d2bea211`
**Reviewers**: UX Reviewer (Agent), Code Reviewer (Agent)
**Requested by**: James Dundon (TPM, NVIDIA Omniverse)

> **Memo**: Diagnostic logging tagged with `[HF40-diag]`, `[GizmoMgr]`, `[HoleFill]` will be removed in a cleanup pass. Items flagged for permanent retention are noted in CODE-10.

---

## Executive Summary

1. **The `deselect_all()` short-circuit (HF-40) is a global behavioral change affecting ALL gizmos, not just HoleFill.** Any active gizmo that reports `is_activable() == true` — including Move, Rotate, Scale, Flatten, and Emboss — now silently suppresses `deselect_all()`. This is the highest-risk item and should be narrowed immediately.

2. **HoleFill is invisible to single-extruder users.** The `on_is_selectable()` gate on `filaments_cnt() > 1` hides the toolbar icon entirely, even though Cut mode (negative volumes) is extruder-count-agnostic. This excludes the majority of OrcaSlicer's user base from the feature.

3. **The keyboard shortcut `Ctrl+H` collides with FuzzySkin and Hollow.** HoleFill can never be activated via keyboard. Trivial fix, critical impact.

4. **~240 lines of dead code** (`commit_batch()`, `enter_batch_preview()`, `BatchState::Preview/Committing/Summary`) and **~20 warning-level diagnostic logs** should be removed. One diagnostic uses `_ReturnAddress()` which is MSVC-only and will break Linux/macOS builds.

5. **The gizmo survival pattern works but is fragile and over-applied.** The `m_state == On -> always activable` hack + `m_pending_reselect_objects` + `open_gizmo()` self-re-open is repeated in 5+ mutation sites. A single `survive_model_mutation()` helper and the BrimEars-style batched-commit model would eliminate the entire class of problems.

---

## UX Findings

### UX-1 [Severity: Critical] Shortcut Key Collision — Ctrl+H Used by Three Gizmos
**What**: `GLGizmoHoleFill`, `GLGizmoFuzzySkin`, and `GLGizmoHollow` all set `m_shortcut_key = WXK_CONTROL_H`. GizmosManager dispatches to the first match in iteration order. Since HoleFill is registered after FuzzySkin and Hollow, pressing Ctrl+H never activates HoleFill.
**Where**: `GLGizmoHoleFill.cpp:45`, `GLGizmoFuzzySkin.cpp:32`, `GLGizmoHollow.cpp:28`
**Comparison**: Every other gizmo has a unique shortcut (Move=M, Rotate=R, Cut=C, Emboss=T, Seam=P, FdmSupports=L, etc.).
**Recommendation**: Assign a unique shortcut. `J` or `O` appear to be free.
**Impact**: Users cannot activate HoleFill via keyboard at all.

### UX-2 [Severity: High] Gizmo Hidden from Single-Extruder Users
**What**: `on_is_selectable()` returns false unless `filaments_cnt() > 1`, hiding the toolbar icon entirely for single-extruder setups. Cut mode (negative volumes) works fine with one extruder.
**Where**: `GLGizmoHoleFill.cpp:110-114`
**Comparison**: FdmSupports is visible regardless of extruder count. Only MmuSegmentation (which fundamentally needs multi-material) has the same gate.
**Recommendation**: Always show the gizmo for ptFFF. Auto-select extruder 1 and disable the color picker when `filaments_cnt() == 1`.
**Impact**: Majority of users with single-extruder printers cannot see or use this feature.

### UX-3 [Severity: High] Apply Button is Permanently Disabled — Dead UI Element
**What**: The "Apply" button is always disabled (`disabled_begin(true)`) with comment "wired up in a later step." Cancel calls `reset_hover_state()` which does nothing meaningful. "Pending plugs" always shows "0".
**Where**: `GLGizmoHoleFill.cpp:1997-2006`
**Comparison**: No other gizmo shows disabled placeholder buttons. Paint gizmos apply immediately. BrimEars applies on close. Cut has a functional "Perform cut" button.
**Recommendation**: Remove Apply/Cancel/Pending count entirely. The click-to-commit model works and matches BrimEars.
**Impact**: Users see broken-looking UI that undermines confidence in the feature.

### UX-4 [Severity: High] Fragile Gizmo Survival Architecture
**What**: HoleFill modifies the model (adds/removes volumes), which triggers `reload_scene()` -> rebuilds GLVolumes -> clears selection -> triggers `refresh_on_off_state()` -> calls `is_activable()` with empty selection -> would kill the gizmo. The HF-40 fix chain (5+ commits, diagnostic logging, pending-reselect, re-open calls) papers over this.
**Where**: `GLGizmoHoleFill.cpp:72-82, 559-572, 614-623, 755-761, 902-908, 1088-1096`
**Comparison**: BrimEars avoids the problem entirely by using `enter_gizmos_stack()`/`leave_gizmos_stack()` to batch edits and commit only on close. Emboss never overrides `is_activable()` and handles volume loss gracefully in `data_changed()`.
**Recommendation**: Long-term: adopt the BrimEars batched-commit pattern. Short-term: extract the survival pattern into a single helper method.
**Impact**: Works after extensive fixes but is fragile. Any future code path triggering `deselect_all()` or `reload_scene()` at the wrong moment could break it.

### UX-5 [Severity: Medium] No ESC Handling — Cannot Cancel Batch Preview
**What**: `on_mouse()` does not handle ESC. In batch mode, Cancel button is not even shown. ESC delegates to `reset_all_states()`, which closes the gizmo entirely rather than stepping back.
**Where**: `GLGizmoHoleFill.cpp:397-434` (on_mouse), `GLGizmoHoleFill.cpp:1987-2006` (Cancel)
**Comparison**: Cut handles ESC via `gizmo_event(SLAGizmoEventType::Escape)`. Paint gizmos consume ESC to cancel current stroke.
**Recommendation**: Intercept ESC to cancel batch preview (if active) rather than closing entirely.
**Impact**: Users must close and reopen the entire gizmo to abort, losing configuration state.

### UX-6 [Severity: Medium] No Undo/Redo Grouping — Every Click is a Separate Undo Entry
**What**: Each fill/remove operation takes its own `TakeSnapshot`. 10 individual clicks = 10 undo entries. No use of `enter_gizmos_stack()`/`leave_gizmos_stack()`.
**Where**: `GLGizmoHoleFill.cpp:543, 609, 707, 896, 1014, 1415`
**Comparison**: BrimEars batches all edits into one undo step via `enter/leave_gizmos_stack`. Paint gizmos use internal undo stacks merged on close.
**Recommendation**: If batched-commit model is adopted, undo grouping comes for free. Otherwise, consider grouping rapid successive fills.
**Impact**: Undo becomes tedious for users who fill many holes individually.

### UX-7 [Severity: Medium] No Cursor Change or Tooltip on Hover
**What**: No `get_tooltip()` override. No cursor change when hovering over detected holes. Shift+Click shortcut only documented in panel text, not in hover context.
**Where**: `GLGizmoHoleFill.hpp` — no `get_tooltip()` override
**Comparison**: Measure shows detailed hover tooltips. Paint gizmos show brush cursor. Emboss shows tooltips on handles.
**Recommendation**: Override `get_tooltip()` with contextual text ("Click to fill hole" / "Shift+Click to fill all on surface" / "Click to remove plug"). Consider crosshair cursor.
**Impact**: Users must rely solely on outline highlight to understand affordances.

### UX-8 [Severity: Medium] Verbose Diagnostic Logging in Production
**What**: 15+ `BOOST_LOG_TRIVIAL(warning)` calls with `[HoleFill]` prefixes throughout. Warning-level logging fires during normal hover/click operations.
**Where**: Throughout `GLGizmoHoleFill.cpp` and `GLGizmosManager.cpp:374-386`
**Comparison**: No other gizmo has diagnostic logging at warning level.
**Recommendation**: Remove or downgrade to debug level. See CODE-10 for specific retention recommendations.
**Impact**: Pollutes user log files, degrades signal-to-noise for bug reports.

### UX-9 [Severity: Low] `data_changed()` Clears Batch State on Any Selection Change
**What**: `data_changed()` unconditionally calls `clear_batch_preview()`. Any background event triggering a selection update discards batch preview state.
**Where**: `GLGizmoHoleFill.cpp:160-162`
**Comparison**: Paint gizmos preserve triangle selector state across `data_changed()`. Emboss preserves text/volume state.
**Recommendation**: Only clear batch state if selection content actually changed. Currently masked by instant-commit batch model, but would be a data loss bug if Preview state is ever made persistent.

### UX-10 [Severity: Low] Picking Not Disabled When Active
**What**: HoleFill does not call `m_parent.enable_picking(false)` when activated. Mouse clicks could potentially leak through to canvas selection.
**Where**: `GLGizmoHoleFill.cpp:123-130`
**Comparison**: `GLGizmoPainterBase::on_set_state()` disables picking when On, re-enables on Off.
**Recommendation**: Disable picking when HoleFill is active to prevent accidental object deselection.
**Impact**: Edge case — `on_mouse()` returning true for LeftDown likely prevents most issues.

---

## Code Findings

### CODE-1 [Severity: Critical] `deselect_all()` Short-Circuit Affects ALL Gizmos
**What**: The HF-40 fix at `GLCanvas3D.cpp:2319-2339` skips the entire deselect (including `selection.remove_all()` and `reset_all_states()`) whenever *any* active gizmo reports `is_activable() == true`. Because Emboss inherits the base `return true`, it is always protected. More critically, Move, Rotate, Scale, and Flatten also return `true` from base `on_is_activable()` when a selection exists. Any code path calling `deselect_all()` while a transform gizmo is active will silently do nothing.
**Where**: `/home/ubuntu/OrcaSlicer/src/slic3r/GUI/GLCanvas3D.cpp` lines 2319-2339
**Pattern comparison**: The original `deselect_all()` unconditionally cleared selection. No other gizmo has ever needed to patch this framework method.
**Recommendation**: Replace with a targeted mechanism: (a) add `virtual bool wants_deselect_protection() const` that only HoleFill overrides, (b) use `m_pending_reselect_objects` as the guard, or (c) remove the guard entirely and rely on `data_changed()` re-selection. **This is the highest-risk item.**
**Risk**: Breaking Ctrl+A, Escape-to-deselect, or other features that call `deselect_all()` while any gizmo is active.

### CODE-2 [Severity: High] `commit_batch()` is Dead Code — 120 Lines of Duplicated Logic
**What**: `commit_batch()` (lines 1404-1525) is declared, fully implemented, but never called. Meanwhile `perform_batch_fill()` (lines 920-1116) contains nearly identical logic with comment "same logic as commit_batch."
**Where**: `GLGizmoHoleFill.cpp` lines 1404-1525, header line 154
**Pattern comparison**: No other gizmo has unreachable methods like this.
**Recommendation**: Delete `commit_batch()` entirely. If two-phase flow is needed later, extract shared logic into a helper.
**Risk**: Dead code with HF-40 comments will mislead future developers. Fixes applied to one path forgotten in the other.

### CODE-3 [Severity: High] `m_state == On -> always activable` Breaks Framework Contract
**What**: `on_is_activable()` returns `true` when `m_state == On` regardless of selection state. Comment says "exactly like Emboss" but Emboss simply inherits the base default `return true` (unconditional, not state-dependent). HoleFill's version creates a conditional lie — activable when On, correctly gated when Off.
**Where**: `GLGizmoHoleFill.cpp` lines 71-101
**Pattern comparison**: Paint gizmos require `is_single_full_instance()` and do NOT have this hack. They survive through `data_changed()`.
**Recommendation**: Either don't override `on_is_activable()` at all (like Emboss), or remove the `m_state == On` early return and fix root cause via re-selection in `data_changed()`.
**Risk**: If last object is deleted while gizmo is On, it stays alive with dangling selection.

### CODE-4 [Severity: Medium] `on_set_state()` Nearly Empty — Missing Lifecycle Management
**What**: HoleFill's `on_set_state()` only logs and resets hover on Off. Missing: picking disable, `m_old_state` tracking, gizmos stack enter/leave, batch state cleanup on deactivation.
**Where**: `GLGizmoHoleFill.cpp` lines 123-130
**Pattern comparison**: `GLGizmoPainterBase::on_set_state()` tracks `m_old_state`, toggles picking, calls `on_opening()/on_shutdown()`. BrimEars enters/leaves gizmos stack.
**Recommendation**: Add `m_old_state` tracking, clean up batch state on Off, consider disabling picking.
**Risk**: Batch state leaks across open/close cycles.

### CODE-5 [Severity: Medium] Massive Duplication in Volume Resolution (mesh_id -> ModelVolume)
**What**: The pattern of mapping `m_rr.mesh_id` to `ModelVolume*` by iterating `mo->volumes` and counting `is_model_part()` is copy-pasted in 7+ locations.
**Where**: Lines 462-475, 658-671, 953-964, 1150-1163, 379-394, 1651-1661, and implicitly in `discover_batch_matches`
**Pattern comparison**: Paint gizmos use higher-level abstractions via `m_c->selection_info()` and `m_c->raycaster()`.
**Recommendation**: Extract `ModelVolume* resolve_model_part_volume(const ModelObject* mo, int mesh_id)` helper. Eliminates ~70 lines of duplication.
**Risk**: If mesh_id counting logic drifts between copies, raycasting could hit the wrong volume.

### CODE-6 [Severity: Medium] Re-Selection Pattern Repeated in 5+ Mutation Sites
**What**: The `m_pending_reselect_objects` + `add_object()` + `plater()->update()` + `open_gizmo()` ceremony is duplicated in `perform_hole_fill`, `perform_hole_remove`, `perform_fill_all_on_surface`, `perform_remove_all_on_surface`, and `perform_batch_fill`.
**Where**: Lines 559-574, 614-627, 755-763, 902-911, 1088-1101
**Pattern comparison**: Emboss's `data_changed()` is 4 lines. Paint gizmos check `mo->id() != m_old_mo_id`.
**Recommendation**: Extract `void survive_model_mutation(const std::vector<int>& object_idxs)`. Single call site per mutation.
**Risk**: If survival pattern needs to change (e.g., to fix CODE-1), all 5+ sites must be updated in sync.

### CODE-7 [Severity: Medium] `open_gizmo()` Self-Re-Open is Semantically Fragile
**What**: Multiple sites call `open_gizmo(HoleFill)` from within HoleFill's own methods. But `open_gizmo()` is a toggle — if the gizmo IS the current type, it CLOSES it (`GLGizmosManager.cpp:401`). The guard `get_current_type() != HoleFill` prevents the toggle, but the semantic coupling is fragile.
**Where**: Lines 571-572, 622-623, 759-760, 907-908, 1094-1095, 153-154
**Pattern comparison**: No other gizmo calls `open_gizmo()` on itself. Emboss and SVG use `activate_gizmo()` directly or rely on framework state management.
**Recommendation**: Add a non-toggling `ensure_gizmo_open(EType)` to the manager, or rely on the activable hack without re-opening.
**Risk**: If the guard is ever removed or races with framework state change, gizmo toggles itself off.

### CODE-8 [Severity: Low] `_ReturnAddress()` is MSVC-Only in Cross-Platform Code
**What**: `GLGizmosManager.cpp:385` uses `_ReturnAddress()` which is MSVC-only. Will break Linux/macOS builds.
**Where**: `GLGizmosManager.cpp` line 385
**Recommendation**: This is diagnostic logging tagged for removal. Remove it. If it must stay, use `#ifdef _MSC_VER` branching.
**Risk**: Build failure on Linux/macOS.

### CODE-9 [Severity: Low] BatchState Enum Has 4 States but Only `Inactive` is Used
**What**: `BatchState::Preview`, `Committing`, `Summary` are never set from any reachable code path. `enter_batch_preview()` (~80 lines) is never called.
**Where**: `GLGizmoHoleFill.hpp` line 49, `GLGizmoHoleFill.cpp` lines 1118-1200
**Recommendation**: Remove unused enum values and `enter_batch_preview()`. Simplify to a boolean or remove entirely.
**Risk**: Dead state machine code creates false complexity.

### CODE-10 [Severity: Low] Diagnostic Logging — What to Keep, What to Remove
**What**: ~20 warning-level diagnostic logs throughout. Two have permanent value:
- `[HoleFill] Boundary:` log (line ~497) — useful for diagnosing mesh issues
- Batch discovery summary (line ~997) — useful for user-facing troubleshooting

All others are HF-40 debugging artifacts.
**Where**: Throughout `GLGizmoHoleFill.cpp`, `GLGizmosManager.cpp:380-386`, `GLCanvas3D.cpp:2329,2335`
**Recommendation**: Remove all `[HF40-diag]` and `[GizmoMgr]` diagnostic logs and the `_ReturnAddress` call. Keep the two above but downgrade to `debug` level.
**Risk**: Warning-level logging on every frame pollutes log files.

### CODE-11 [Severity: Low] `on_is_activable()` Accepts Overly Broad Selection Types
**What**: HoleFill accepts `is_any_volume()` in addition to `is_single_full_instance()` and multi-object selections. Volume-level selection may produce confusing behavior since HoleFill operates on the parent object.
**Where**: `GLGizmoHoleFill.cpp` line 88
**Pattern comparison**: Paint gizmos only accept `is_single_full_instance()`.
**Recommendation**: Verify volume-level selection works correctly with raycasting and volume mutation logic.
**Risk**: Edge case bugs with sub-volume selection.

---

## Consistency Matrix

| Dimension | HoleFill | Emboss | FdmSupports | MmuSegmentation | BrimEars | Cut |
|---|---|---|---|---|---|---|
| **Entry method** | Toolbar + Ctrl+H (**BROKEN**: collision) | Toolbar + Ctrl+T | Toolbar + Ctrl+L | Toolbar + Ctrl+N | Toolbar + Ctrl+E | Toolbar + Ctrl+C |
| **Exit method** | ESC closes entirely | ESC closes entirely | ESC closes entirely | ESC closes entirely | ESC commits + closes | ESC closes (special handling) |
| **Panel type** | ImGui floating, auto-resize | ImGui floating, large | ImGui floating | ImGui floating | ImGui floating | ImGui floating, large |
| **Selection survival** | Yes (via HF-40 re-select hack) | Yes (native) | N/A (no volume changes) | N/A (no volume changes) | Commits on close only | No (new objects created) |
| **Multi-object support** | Yes (batch mode) | No (single volume) | No (single instance) | No (single instance) | No (single instance) | No (single object) |
| **Undo/redo** | Per-click TakeSnapshot | Per-op TakeSnapshot | Internal stack, merged | Internal stack, merged | Batched enter/leave | Per-op TakeSnapshot |
| **Interaction model** | Click-to-place | Text editing | Brush paint (drag) | Brush paint (drag) | Click-to-place | Plane manipulation |
| **Empty state** | Notification toast | Creates new text | N/A (brush valid) | N/A (brush valid) | Auto-generates | Plane always shown |
| **Visual feedback** | Outline + translucent fill | Text preview + handle | Brush sphere/circle | Brush + colors | 3D cylinder markers | Plane + connectors |
| **Gizmo stack** | No | No | No (PainterBase) | No (PainterBase) | **Yes** | No |
| **Picking disabled** | No | No | **Yes** | **Yes** | No | No |
| **Requires multi-extruder** | **Yes** | No | No | Yes | No | No |
| **`is_activable` override** | Yes (hack: true when On) | No (base default) | Yes (single instance) | Yes (single instance) | Yes (single instance) | Yes (single instance) |
| **Calls `open_gizmo()` on self** | **Yes** (5+ sites) | No | No | No | No | No |

---

## Recommended Next Steps

Ordered by impact (risk x effort ratio):

### Immediate (before next release)

1. **Narrow the `deselect_all()` short-circuit** (CODE-1, Critical) — Replace the blanket `is_activable()` guard with a targeted `wants_deselect_protection()` virtual or use `m_pending_reselect_objects` as the condition. This is the only change that could break other gizmos in production. Estimated: 1-2 hours.

2. **Fix shortcut collision** (UX-1, Critical) — Change `m_shortcut_key` to an unused key. One-line change. Estimated: 5 minutes.

3. **Delete dead code** (CODE-2, CODE-9) — Remove `commit_batch()`, `enter_batch_preview()`, unused `BatchState` values. ~240 lines. Estimated: 30 minutes.

4. **Remove diagnostic logging** (CODE-8, CODE-10, UX-8) — Remove all `[HF40-diag]`/`[GizmoMgr]` logs and `_ReturnAddress()`. Keep boundary and batch-summary logs at debug level. Estimated: 30 minutes.

### Short-term (next sprint)

5. **Remove multi-extruder gate** (UX-2) — Make gizmo visible for single-extruder users. Auto-select extruder 1, disable color picker when `filaments_cnt() == 1`. Estimated: 2-4 hours.

6. **Remove dead Apply/Cancel/Pending UI** (UX-3) — Clean up the placeholder controls. Estimated: 1 hour.

7. **Extract `resolve_model_part_volume()` helper** (CODE-5) — Eliminate 7x duplication of mesh_id resolution. Estimated: 1-2 hours.

8. **Extract `survive_model_mutation()` helper** (CODE-6) — Consolidate the 5x duplicated re-selection ceremony. Estimated: 2-3 hours.

### Medium-term (next milestone)

9. **Add `get_tooltip()` override** (UX-7) — Contextual hover text for fill/remove/batch. Estimated: 2-3 hours.

10. **Flesh out `on_set_state()`** (CODE-4) — Add `m_old_state` tracking, batch state cleanup, consider picking disable. Estimated: 3-4 hours.

11. **Evaluate batched-commit model** (UX-4, UX-6, CODE-3, CODE-7) — The BrimEars pattern (accumulate edits, commit on close) would eliminate the survival hack, the `deselect_all` short-circuit, the `open_gizmo()` self-re-open, and the per-click undo problem all at once. This is the single highest-leverage architectural change but requires significant rework. Recommend a design doc before implementing.

---

*Review generated 2026-04-20 by automated UX + Code review agents. No source code was modified.*
