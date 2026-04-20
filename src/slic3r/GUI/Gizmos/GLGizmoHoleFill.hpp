#ifndef slic3r_GLGizmoHoleFill_hpp_
#define slic3r_GLGizmoHoleFill_hpp_

#include "GLGizmoBase.hpp"
#include "libslic3r/HoleFinder.hpp"
#include "slic3r/GUI/GLModel.hpp"

#include <vector>
#include <map>
#include <memory>

namespace Slic3r {

class ModelVolume;

namespace GUI {

class MeshRaycaster;

class GLGizmoHoleFill : public GLGizmoBase
{
public:
    GLGizmoHoleFill(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id);
    ~GLGizmoHoleFill() override = default;

    bool on_mouse(const wxMouseEvent& mouse_event) override;
    void data_changed(bool is_serializing) override;

    bool wants_enter_leave_snapshots() const override { return true; }
    std::string get_gizmo_entering_text() const override { return _u8L("Entering Hole Fill"); }
    std::string get_gizmo_leaving_text() const override  { return _u8L("Leaving Hole Fill"); }
    std::string get_action_snapshot_name() const override { return _u8L("Hole fill editing"); }

protected:
    bool on_init() override;
    std::string on_get_name() const override;
    bool on_is_activable() const override;
    bool on_is_selectable() const override;
    CommonGizmosDataID on_get_requirements() const override;

    void on_set_state() override;
    void on_render() override;
    void on_render_input_window(float x, float y, float bottom_limit) override;

public:
    enum class Mode : int { Fill = 0, Cut = 1 };

    // Batch mode state machine
    enum class BatchState { Inactive, Preview, Committing, Summary };

    struct BatchPreviewEntry {
        int          object_idx   = -1;
        int          instance_idx = -1;
        int          volume_idx   = -1;
        int          seed_facet   = -1;
        HoleBoundary boundary;
        Vec3d        world_center = Vec3d::Zero();
        bool         is_reference = false;
    };

    enum class BatchScope { AllFaces = 0, MatchingNormal = 1, SingleSurface = 2 };

private:
    // UI / config state
    Mode   m_mode                  = Mode::Fill;
    float  m_depth                 = 1.0f;
    float  m_angle_tolerance       = 5.0f;
    size_t m_selected_extruder_idx = 0;
    std::vector<ColorRGBA> m_extruders_colors;

    // Batch mode state
    BatchState                     m_batch_state = BatchState::Inactive;
    std::vector<BatchPreviewEntry> m_batch_preview;
    BatchScope                     m_batch_scope          = BatchScope::MatchingNormal;
    bool                           m_batch_all_instances  = false;

    // Reference hole (captured on first click in batch mode)
    HoleBoundary m_batch_ref_boundary;
    Vec3f        m_batch_ref_world_normal = Vec3f::Zero();
    int          m_batch_ref_object_idx   = -1;
    int          m_batch_ref_volume_idx   = -1;
    int          m_batch_ref_facet        = -1;

    // Hover preview state
    int          m_hover_facet      = -1;
    int          m_hover_mesh_id    = -1;
    Mode         m_hover_mode       = Mode::Fill;
    bool         m_hover_hole_valid = false;
    HoleBoundary m_hover_boundary;
    GLModel      m_hover_outline_mesh;
    GLModel      m_hover_fill_mesh;

    // Remove-mode hover state: set when the cursor is over an existing
    // HoleFill_ plug volume. In that case we show a red outline and left
    // click removes the plug instead of filling.
    bool    m_hover_is_plug             = false;
    int     m_hover_plug_raw_idx        = -1;  // index into ModelObject::volumes
    int     m_remove_outline_cached_idx = -1;
    GLModel m_remove_outline_mesh;

    // Last raycast hit (populated from on_mouse Motion events).
    // Mirrors GLGizmoPainterBase::m_rr for the subset of fields we use.
    struct RaycastResult {
        int   mesh_id = -1;
        int   facet   = -1;
        Vec3f hit     = Vec3f::Zero();
        int   object_idx   = -1;  // set by pick_mesh_multi
        int   instance_idx = -1;  // set by pick_mesh_multi
    };
    RaycastResult m_rr;

    // Track which object the hover is on (for multi-select rendering).
    int m_hover_object_idx   = -1;
    int m_hover_instance_idx = -1;

    // Raycaster cache for multi-object picking (BVH is expensive to build).
    // Keyed by (object_idx, volume_raw_idx). Cleared in data_changed().
    std::map<std::pair<int,int>, std::unique_ptr<MeshRaycaster>> m_multi_raycasters;


    static const constexpr float HoleFillDepthMin    = 0.2f;
    static const constexpr float HoleFillDepthMax    = 5.0f;
    static const constexpr float HoleCutDepthMax     = 50.0f;
    static const constexpr float HoleFillDepthStep   = 0.1f;

    // Translated UI strings, populated on_init().
    std::map<std::string, wxString> m_desc;

    // Helpers
    void update_hover(const Vec2d& mouse_position);
    void perform_hole_fill(const Vec2d& mouse_position);
    void perform_hole_remove();
    void perform_fill_all_on_surface(const Vec2d& mouse_position);
    void perform_remove_all_on_surface(const Vec2d& mouse_position);
    void render_hole_fill_hover();
    void render_remove_hover();
    bool pick_mesh(const Vec2d& mouse_position, RaycastResult& out) const;
    bool pick_mesh_multi(const Vec2d& mouse_position, RaycastResult& out);
    void reset_hover_state();
    void init_extruders_data();

    // Whether the current selection qualifies for batch mode.
    bool is_batch_selection() const;

    // Batch actions
    void perform_batch_fill(const Vec2d& mouse_position);
    void enter_batch_preview(const Vec2d& mouse_position);
    void discover_batch_matches();
    void commit_batch();
    void cancel_batch();
    void clear_batch_preview();
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GLGizmoHoleFill_hpp_
