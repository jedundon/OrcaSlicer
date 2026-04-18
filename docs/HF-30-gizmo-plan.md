# GLGizmoHoleFill Extraction — Implementation Plan

## 1. Anchor Points in Existing Code

### Files Currently Touching Hole-Fill

| File | Lines | What lives there |
|------|-------|------------------|
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.hpp` | 5 | `#include "libslic3r/HoleFinder.hpp"` |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.hpp` | 125–133 | `m_hole_fill_depth`, `m_hole_fill_angle_tolerance`, `HoleFillDepthMin/Max/Step` consts, hover state (`m_hover_facet`, `m_hover_mesh_id`, `m_hover_hole_valid`, `m_hover_boundary`, `m_hover_outline_mesh`, `m_hover_fill_mesh`) |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.hpp` | 159–169 | `perform_hole_fill()`, `render_hole_fill_hover()` private methods |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 16 | `#include "libslic3r/PlugGenerator.hpp"` |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 29 | `static const wchar_t HoleFillToolIcon = 0xF0FF;` |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 132–135 | UI strings: `"hole_fill_depth"`, `"hole_fill_depth_caption"`, `"tool_hole_fill"` |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 185–186 | Call to `render_hole_fill_hover()` from top-level render |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 249–251 | `'J'` shortcut → activates HoleFill tool |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 487, 490, 492 | Tool button arrays include `HoleFillToolIcon` |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 698–712 | ImGui depth slider + drag-float UI |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 829–831 | Cleanup of `"HoleFill_"`-prefixed volumes on re-open |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 1431–1443 | Left-click + mouse-wheel dispatch for hole-fill mode |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 1451–1590 | `perform_hole_fill()` full body |
| `src/slic3r/GUI/Gizmos/GLGizmoMmuSegmentation.cpp` | 1592–1723 | `render_hole_fill_hover()` full body |
| `src/libslic3r/PlugGenerator.hpp` / `.cpp` | — | `generate_plug()`, `generate_island_negative()`, `build_plane_frame()`, `project_to_2d()`, `unproject_to_3d()` — reused as-is |
| `src/libslic3r/HoleFinder.hpp` | — | `HoleBoundary`, `find_nearest_hole()` — reused as-is |

### Manager / Registration Anchors

| File | Lines | Anchor |
|------|-------|--------|
| `src/slic3r/GUI/Gizmos/GLGizmosManager.hpp` | 74–98 | `enum EType` — insert `HoleFill` before `Undefined` |
| `src/slic3r/GUI/Gizmos/GLGizmosManager.cpp` | 202–221 | `m_gizmos.emplace_back(...)` construction block |
| `src/slic3r/GUI/Gizmos/GLGizmosManager.cpp` | 123–182 | `switch_gizmos_icon_filename()` case table |
| `src/slic3r/GUI/Gizmos/GLGizmosManager.cpp` | 467–493 | `handle_shortcut()` — no edits needed (auto-discovered) |
| `src/slic3r/CMakeLists.txt` | 127–174 | Gizmo source-file list |

---

## 2. New Files: GLGizmoHoleFill.hpp / .cpp Skeleton

### `src/slic3r/GUI/Gizmos/GLGizmoHoleFill.hpp` (~180 lines)

```cpp
#pragma once
#include "GLGizmoBase.hpp"
#include "libslic3r/HoleFinder.hpp"
#include "slic3r/GUI/GLModel.hpp"
#include <vector>

namespace Slic3r {
class ModelVolume;
namespace GUI {

class GLGizmoHoleFill : public GLGizmoBase
{
public:
    GLGizmoHoleFill(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id);
    ~GLGizmoHoleFill() override;

    bool on_mouse(const wxMouseEvent& mouse_event) override;

protected:
    // Core overrides
    bool on_init() override;
    std::string on_get_name() const override;
    bool on_is_activable() const override;
    bool on_is_selectable() const override;
    CommonGizmosDataID on_get_requirements() const override;

    void on_set_state() override;
    void data_changed(bool is_serializing) override;

    void on_render() override;
    void on_render_input_window(float x, float y, float bottom_limit) override;

    void on_register_raycasters_for_picking() override;
    void on_unregister_raycasters_for_picking() override;

    bool wants_enter_leave_snapshots() const override { return true; }
    std::string get_gizmo_entering_text() const override;
    std::string get_gizmo_leaving_text()   const override;
    std::string get_action_snapshot_name() const override;

private:
    // UI + config state
    float  m_depth                  = 1.0f;
    float  m_angle_tolerance        = 5.0f;
    size_t m_selected_extruder_idx  = 0;
    std::vector<ColorRGBA> m_extruders_colors;

    // Raycast / hover cache
    int  m_hover_facet       = -1;
    int  m_hover_mesh_id     = -1;
    bool m_hover_hole_valid  = false;
    HoleBoundary m_hover_boundary;
    GLModel      m_hover_outline_mesh;
    GLModel      m_hover_fill_mesh;

    // Pending-apply list (hover-confirm workflow)
    struct PendingPlug {
        int           source_volume_idx;
        int           facet_idx;
        HoleBoundary  boundary;
        size_t        extruder_idx;
    };
    std::vector<PendingPlug> m_pending;

    // Static UI bounds
    static constexpr float DepthMin  = 0.2f;
    static constexpr float DepthMax  = 5.0f;
    static constexpr float DepthStep = 0.1f;

    // Helpers
    void  update_hover(const Vec2d& mouse_position);
    bool  apply_plug_at_hover();
    void  apply_all_pending();
    void  clear_pending();
    void  remove_preview_volumes();
    void  render_hover_preview();
    ModelVolume* find_source_volume(int mesh_id, int& out_raw_idx) const;
    bool  is_duplicate_fill(const TriangleMesh& plug, const ModelObject* mo) const;
};

}} // namespace Slic3r::GUI
```

### `src/slic3r/GUI/Gizmos/GLGizmoHoleFill.cpp` (~1,400–1,700 lines)

High-level skeleton:

```cpp
#include "GLGizmoHoleFill.hpp"
#include "libslic3r/PlugGenerator.hpp"
#include "libslic3r/HoleFinder.hpp"
#include "libslic3r/Model.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/GUI_ObjectList.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/Selection.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosCommon.hpp"
#include "slic3r/GUI/I18N.hpp"

namespace Slic3r { namespace GUI {

GLGizmoHoleFill::GLGizmoHoleFill(GLCanvas3D& parent,
                                 const std::string& icon_filename,
                                 unsigned int sprite_id)
    : GLGizmoBase(parent, icon_filename, sprite_id) {}

bool GLGizmoHoleFill::on_init()
{
    m_shortcut_key = WXK_CONTROL_H; // "H" for Hole — see §9
    return true;
}

std::string GLGizmoHoleFill::on_get_name() const { return _u8L("Hole Fill"); }
bool GLGizmoHoleFill::on_is_activable() const { /* require exactly 1 ModelObject selected */ }
CommonGizmosDataID GLGizmoHoleFill::on_get_requirements() const {
    return CommonGizmosDataID(int(CommonGizmosDataID::SelectionInfo)
                            | int(CommonGizmosDataID::Raycaster));
}

void GLGizmoHoleFill::on_set_state() { /* on close: clear_pending(); remove_preview_volumes(); */ }
void GLGizmoHoleFill::data_changed(bool) { /* reload extruder colors, clear hover */ }

void GLGizmoHoleFill::on_render() { render_hover_preview(); /* draw previews for m_pending */ }
void GLGizmoHoleFill::on_render_input_window(float x, float y, float bottom_limit) { /* §5 */ }

bool GLGizmoHoleFill::on_mouse(const wxMouseEvent& evt) { /* §6 */ }

// ... perform / preview impls lifted from GLGizmoMmuSegmentation.cpp 1451–1723 ...
}}
```

---

## 3. Registration in `GLGizmosManager`

### 3.1 `GLGizmosManager.hpp` line 74–98 — add enum entry

```cpp
enum EType : unsigned char
{
    Move, Rotate, Scale, Flatten, Cut, MeshBoolean,
    FdmSupports, Seam, FuzzySkin, MmSegmentation,
    Emboss, Svg, Measure, Assembly, Simplify, BrimEars,
    HoleFill,      // <-- NEW (insert before Undefined; must match m_gizmos order)
    Undefined,
};
```

### 3.2 `GLGizmosManager.cpp` ~line 221 — add constructor

```cpp
#include "GLGizmoHoleFill.hpp"
// ...
m_gizmos.emplace_back(new GLGizmoBrimEars(m_parent,
    m_is_dark ? "toolbar_brimears_dark.svg" : "toolbar_brimears.svg", EType::BrimEars));
m_gizmos.emplace_back(new GLGizmoHoleFill(m_parent,
    m_is_dark ? "toolbar_hole_fill_dark.svg" : "toolbar_hole_fill.svg", EType::HoleFill));
```

### 3.3 `GLGizmosManager.cpp` ~line 177 — add icon case

```cpp
case(EType::HoleFill):
    gizmo->set_icon_filename(m_is_dark ? "toolbar_hole_fill_dark.svg"
                                       : "toolbar_hole_fill.svg");
    break;
```

### 3.4 No change required to `handle_shortcut()` (lines 467–493) — it auto-discovers via `get_shortcut_key()`.

---

## 4. CMake Additions

### `src/slic3r/CMakeLists.txt` (insert in alphabetical block, ~line 142, just after `GLGizmoFuzzySkin.hpp`)

```cmake
GUI/Gizmos/GLGizmoHoleFill.cpp
GUI/Gizmos/GLGizmoHoleFill.hpp
```

No change needed to `src/libslic3r/CMakeLists.txt` — `PlugGenerator` is already registered at lines 336–337.

---

## 5. ImGui Sidebar UI

Rendered from `on_render_input_window()`; lifted + reshaped from `GLGizmoMmuSegmentation.cpp:698–712` plus new color / list / action controls.

```cpp
void GLGizmoHoleFill::on_render_input_window(float x, float y, float bottom_limit)
{
    m_imgui->begin(_u8L("Hole Fill"), ImGuiWindowFlags_AlwaysAutoResize);

    // --- Depth slider (from 1451–1590 / 698–712) ---
    ImGui::AlignTextToFramePadding();
    m_imgui->text(_u8L("Fill depth") + ":");
    ImGui::SameLine(sliders_left);
    m_imgui->bbl_slider_float_style("##depth", &m_depth, DepthMin, DepthMax, "%.1f mm", 1.0f, true);
    ImGui::SameLine();
    ImGui::BBLDragFloat("##depth_drag", &m_depth, 0.05f, 0.0f, 0.0f, "%.1f");
    m_depth = std::clamp(m_depth, DepthMin, DepthMax);

    // --- Angle tolerance ---
    m_imgui->text(_u8L("Angle tolerance") + ":");
    ImGui::SliderFloat("##angle", &m_angle_tolerance, 1.0f, 30.0f, "%.1f°");

    // --- Extruder / color picker (filament palette) ---
    m_imgui->text(_u8L("Filament") + ":");
    for (size_t i = 0; i < m_extruders_colors.size(); ++i) {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(int(i));
        ImVec4 col = to_imvec4(m_extruders_colors[i]);
        if (ImGui::ColorButton("##e", col,
                ImGuiColorEditFlags_NoTooltip,
                ImVec2(20, 20)))
            m_selected_extruder_idx = i;
        if (m_selected_extruder_idx == i) /* draw highlight ring */;
        ImPop();
    }

    ImGui::Separator();

    // --- Pending-plug list ---
    m_imgui->text(_u8L("Pending plugs") + ": " + std::to_string(m_pending.size()));
    for (size_t i = 0; i < m_pending.size(); ++i) {
        ImGui::PushID(int(i));
        ImGui::Text("#%zu  vol=%d facet=%d  ext=%zu",
                    i, m_pending[i].source_volume_idx,
                    m_pending[i].facet_idx, m_pending[i].extruder_idx + 1);
        ImGui::SameLine();
        if (m_imgui->button(_L("X"))) { m_pending.erase(m_pending.begin()+i); /* rebuild preview */ }
        ImGui::PopID();
    }

    ImGui::Separator();

    // --- Apply / Cancel ---
    if (m_imgui->button(_L("Apply")))   apply_all_pending();
    ImGui::SameLine();
    if (m_imgui->button(_L("Cancel")))  { clear_pending(); remove_preview_volumes(); }

    m_imgui->end();
}
```

---

## 6. 3D Viewport Interaction

### Hover highlight (every frame while hovering)
Lifted from `render_hole_fill_hover()` at `GLGizmoMmuSegmentation.cpp:1592–1723`:

- On mouse move, query current raycast hit (`m_c->raycaster()` → facet + mesh_id + point).
- If `(facet_idx, mesh_id)` changed, call `find_nearest_hole(its, facet_idx, hit_point, boundary, m_angle_tolerance)`.
- Build two `GLModel`s: outline (green lines) + semi-transparent fill (selected filament color, α=0.35).
- Call `on_render()` each frame → draws outline + fill quads.

### Click-to-select (add to pending list)
In `on_mouse()`:

```cpp
if (evt.LeftDown() && m_hover_hole_valid) {
    PendingPlug p { m_hover_source_vol_idx,
                    m_hover_facet,
                    m_hover_boundary,
                    m_selected_extruder_idx };
    m_pending.push_back(std::move(p));
    // Re-render preview with this plug added
    return true;
}
if (evt.GetWheelRotation() != 0 && evt.ControlDown()) {
    m_depth = std::clamp(m_depth + (evt.GetWheelRotation() > 0 ? DepthStep : -DepthStep),
                         DepthMin, DepthMax);
    return true;
}
```

### Preview rendering
For each `PendingPlug`, generate the plug mesh in memory (do not yet attach to ModelObject) and draw as a translucent `GLModel` overlay. This keeps preview cheap and Cancel trivial.

---

## 7. Model Integration

Happens only in `apply_all_pending()` (user clicked Apply) — lifted from `GLGizmoMmuSegmentation.cpp:1451–1590`:

```cpp
Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Hole fill plugs");
ModelObject* mo = wxGetApp().model().objects[obj_idx];

for (const PendingPlug& p : m_pending) {
    const ModelVolume* src = mo->volumes[p.source_volume_idx];
    TriangleMesh plug = generate_plug(p.boundary, m_depth);
    if (plug.empty()) continue;
    if (is_duplicate_fill(plug, mo)) continue;

    ModelVolume* nv = mo->add_volume(std::move(plug),
                                     ModelVolumeType::MODEL_PART, false);
    nv->set_new_unique_id();
    nv->name = "HoleFill_" + std::to_string(p.source_volume_idx)
             + "_f"       + std::to_string(p.facet_idx);
    nv->config.set("extruder", int(p.extruder_idx) + 1);
    nv->set_transformation(src->get_transformation());
}

wxGetApp().plater()->update();
wxGetApp().obj_list()->update_after_undo_redo();
m_pending.clear();
```

**Inner-loop (island) negatives**: For boundaries with non-empty `inner_loops`, also call `generate_island_negative(...)` and add as a `ModelVolumeType::NEGATIVE_VOLUME` with name `"HoleFill_neg_..."` (preserving duplicate-guard prefix convention).

---

## 8. Undo/Redo Integration

- Wrap `apply_all_pending()` in `Plater::TakeSnapshot snapshot(plater, "Hole fill plugs")` — same pattern as line 1538.
- Override `wants_enter_leave_snapshots() → true`, `get_gizmo_entering_text()/leaving_text()/action_snapshot_name()` so gizmo entry/exit participates in the timeline.
- Hover previews are ephemeral — never touch `ModelVolume`s, so no snapshot contamination.
- On gizmo close without Apply, `on_set_state()` drops `m_pending` silently (nothing to undo).

---

## 9. Icon & Keyboard Shortcut

### Icons
Create two SVGs in `resources/images/`:
- `toolbar_hole_fill.svg` (light)
- `toolbar_hole_fill_dark.svg` (dark)

Style should match BrimEars / Seam toolbar family (32×32 stroke icon; a circle with a plug/dot glyph).

### Shortcut
- `GLGizmoHoleFill::on_init()` sets `m_shortcut_key = WXK_CONTROL_H`.
- `'J'` is freed — it's currently grabbed at `GLGizmoMmuSegmentation.cpp:249–251` for activating the HoleFill sub-tool. After extraction, that branch is deleted, so `'J'` becomes available.
- Check for collisions with existing gizmos; if `H` collides, fall back to `'U'`.

---

## 10. What to REMOVE from `GLGizmoMmuSegmentation`

### `.hpp`
- Line 5: `#include "libslic3r/HoleFinder.hpp"` (move to new gizmo)
- Lines 125–133: depth/angle config + `HoleFillDepth*` constants
- Lines 125–133: hover state members (`m_hover_facet`, `m_hover_mesh_id`, `m_hover_hole_valid`, `m_hover_boundary`, `m_hover_outline_mesh`, `m_hover_fill_mesh`)
- Lines 159–169: `perform_hole_fill()` and `render_hole_fill_hover()` declarations

### `.cpp`
- Line 16: `#include "libslic3r/PlugGenerator.hpp"`
- Line 29: `static const wchar_t HoleFillToolIcon = 0xF0FF;`
- Lines 132–135: `m_desc["hole_fill_*"]` strings
- Lines 185–186: call to `render_hole_fill_hover()`
- Lines 249–251: `'J' → HoleFillToolIcon` case in `on_key_down_select_tool_type`
- Lines 487, 490, 492: `HoleFillToolIcon` entries in tool-button arrays
- Lines 698–712: depth-slider UI branch
- Lines 829–831: `"HoleFill_"` volume-cleanup loop (NOTE: audit whether MMU still wants to hide these from its painter — likely yes; keep the read-only filter but drop the delete)
- Lines 1431–1443: left-click + mouse-wheel dispatch branches
- Lines 1451–1590: full `perform_hole_fill()`
- Lines 1592–1723: full `render_hole_fill_hover()`

### Side effects to verify after removal
- MMU's tool-selector must still compile after removing three array entries (count constants may need decrement).
- MMU's painter should continue to ignore `"HoleFill_"`-named volumes so its triangle selector does not treat them as paint targets.

---

## 11. Size Estimates

| Component | Size |
|-----------|------|
| `GLGizmoHoleFill.hpp` | ~180 lines |
| `GLGizmoHoleFill.cpp` | ~1,400–1,700 lines (most lifted verbatim from MMU seg's two methods, plus ImGui panel + state machine) |
| Removals from `GLGizmoMmuSegmentation.{hpp,cpp}` | ~310 lines deleted |
| `GLGizmosManager.hpp` edits | +1 line |
| `GLGizmosManager.cpp` edits | +6 lines (emplace_back + icon case) |
| `src/slic3r/CMakeLists.txt` | +2 lines |
| SVG icons | 2 files, ~3 KB each |
| Net LOC change | roughly +1,300 lines (duplication-free — mostly file relocation + new UI scaffolding) |

---

## 12. Implementation Order (quickest testable result first)

1. **Stub gizmo + register (compiles, shows up, does nothing)** — ~30 min
   - Create `GLGizmoHoleFill.{hpp,cpp}` with empty `on_render()`, `on_render_input_window()` showing only a "Hello" label.
   - Add `HoleFill` to `EType`, `emplace_back` line, icon `switch` case, CMake entry.
   - Drop placeholder SVG icons (copy `toolbar_brimears.svg`).
   - **Test**: build, open app, confirm icon appears on toolbar and gizmo opens.

2. **Move `perform_hole_fill()` verbatim, wire to left-click** — ~1 h
   - Copy body from MMU `cpp:1451–1590` into new gizmo.
   - In `on_mouse()`, dispatch left-click → `perform_hole_fill(mouse_pos)` using `m_c->raycaster()` for the hit.
   - Leave MMU's copy intact for now (parallel working implementations).
   - **Test**: open model with holes, click in new gizmo, confirm plug volumes appear with correct names.

3. **Move `render_hole_fill_hover()` verbatim, wire to `on_render()`** — ~45 min
   - Copy body from MMU `cpp:1592–1723`.
   - Track hover state via `update_hover()` called from `on_mouse()` motion events.
   - **Test**: hover over a hole, confirm green outline + translucent fill render.

4. **Build the ImGui sidebar** — ~1.5 h
   - Depth slider + drag-float (from MMU 698–712).
   - Filament color picker row driven by `wxGetApp().plater()->get_extruders_colors()`.
   - Pending-plug list placeholder (empty for now).
   - Apply / Cancel buttons stubbed.
   - **Test**: slider changes depth, color click changes selected extruder (verified via `qDebug` print).

5. **Convert from immediate-apply to pending-list workflow** — ~1.5 h
   - Left-click appends to `m_pending` instead of applying.
   - `on_render()` also iterates `m_pending` drawing ghost plugs.
   - `Apply` button drains `m_pending` via `apply_all_pending()` (wraps `TakeSnapshot`).
   - `Cancel` / close drops list.
   - **Test**: click 3 holes → see 3 ghosts → Apply → 3 volumes; click 3 → Cancel → none.

6. **Undo/redo polish** — ~30 min
   - Override snapshot-text methods; verify entering/leaving the gizmo with pending-applied state shows up in history.
   - **Test**: Apply, Ctrl-Z reverts all plugs; Ctrl-Y re-adds.

7. **Remove all hole-fill code from `GLGizmoMmuSegmentation`** — ~45 min
   - Delete items enumerated in §10.
   - Verify MMU segmentation still compiles and paints normally.
   - Adjust tool-button array counts / constants as required.
   - Ensure `"HoleFill_"` volumes are still excluded from MMU painter targeting.
   - **Test**: MMU segmentation gizmo works unchanged; HoleFill gizmo still works.

8. **Icon + shortcut finalization** — ~30 min
   - Produce real `toolbar_hole_fill.svg` + `_dark.svg`.
   - Set `m_shortcut_key` (default `H`; fall back if collision).
   - Register i18n strings (`on_get_name()`, snapshot labels).
   - **Test**: Ctrl-H (or chosen key) opens gizmo; icons render correctly in both themes.

---

## 13. Out of Scope

- **Algorithmic changes** to `PlugGenerator` or `HoleFinder` (kept as a black box).
- **Multiple-object** hole-fill (like MMU, gizmo requires single-object selection).
- **SLA print** support — hole fill is FFF-only and inherits MMU's `on_is_selectable()` gating.
- **Profile / config schema additions** — depth/angle are per-session UI state, not persisted config.
- **3MF serialization of pending state** — only applied plug volumes (already native `ModelVolume`s) serialize; unconfirmed hover/pending state is ephemeral.
- **Automatic hole detection** (scan whole model up front) — remains click-driven.
- **Assembly-view `Selection::fill_color()`** (`src/slic3r/GUI/Selection.cpp:2130`, `Plater.cpp:5222,16919`) — unrelated feature despite name collision; leave alone.
- **Removal of `HoleFinder` or `PlugGenerator`** from libslic3r — they are reused verbatim, not refactored.
- **Negative-volume inner-loop handling refinements** — carry forward the existing MMU behavior exactly; any fix lands in a later PR.
- **Tests** — defer fff_print / libslic3r regression tests for the gizmo itself; `PlugGenerator` coverage is unaffected.
