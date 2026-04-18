#ifndef slic3r_GLGizmoHoleFill_hpp_
#define slic3r_GLGizmoHoleFill_hpp_

#include "GLGizmoBase.hpp"
#include "libslic3r/HoleFinder.hpp"
#include "slic3r/GUI/GLModel.hpp"

#include <vector>
#include <map>

namespace Slic3r {

class ModelVolume;

namespace GUI {

class GLGizmoHoleFill : public GLGizmoBase
{
public:
    GLGizmoHoleFill(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id);
    ~GLGizmoHoleFill() override = default;

    bool on_mouse(const wxMouseEvent& mouse_event) override;
    void data_changed(bool is_serializing) override;

    bool wants_enter_leave_snapshots() const override { return true; }
    std::string get_gizmo_entering_text() const override { return "Entering Hole Fill"; }
    std::string get_gizmo_leaving_text() const override  { return "Leaving Hole Fill"; }
    std::string get_action_snapshot_name() const override { return "Hole fill editing"; }

protected:
    bool on_init() override;
    std::string on_get_name() const override;
    bool on_is_activable() const override;
    bool on_is_selectable() const override;
    CommonGizmosDataID on_get_requirements() const override;

    void on_set_state() override;
    void on_render() override;
    void on_render_input_window(float x, float y, float bottom_limit) override;

private:
    // UI / config state
    float  m_depth                 = 1.0f;
    float  m_angle_tolerance       = 5.0f;
    size_t m_selected_extruder_idx = 0;
    std::vector<ColorRGBA> m_extruders_colors;

    // Hover preview state
    int          m_hover_facet      = -1;
    int          m_hover_mesh_id    = -1;
    bool         m_hover_hole_valid = false;
    HoleBoundary m_hover_boundary;
    GLModel      m_hover_outline_mesh;
    GLModel      m_hover_fill_mesh;

    // Last raycast hit (populated from on_mouse Motion events).
    // Mirrors GLGizmoPainterBase::m_rr for the subset of fields we use.
    struct RaycastResult {
        int   mesh_id = -1;
        int   facet   = -1;
        Vec3f hit     = Vec3f::Zero();
    };
    RaycastResult m_rr;

    static const constexpr float HoleFillDepthMin  = 0.2f;
    static const constexpr float HoleFillDepthMax  = 5.0f;
    static const constexpr float HoleFillDepthStep = 0.1f;

    // Translated UI strings, populated on_init().
    std::map<std::string, wxString> m_desc;

    // Helpers
    void update_hover(const Vec2d& mouse_position);
    void perform_hole_fill(const Vec2d& mouse_position);
    void render_hole_fill_hover();
    bool pick_mesh(const Vec2d& mouse_position, RaycastResult& out) const;
    void reset_hover_state();
    void init_extruders_data();
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GLGizmoHoleFill_hpp_
