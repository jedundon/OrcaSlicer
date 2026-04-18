#include "GLGizmoHoleFill.hpp"

#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/GUI_ObjectList.hpp"
#include "slic3r/GUI/Selection.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/NotificationManager.hpp"
#include "slic3r/GUI/OpenGLManager.hpp"
#include "slic3r/GUI/format.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosCommon.hpp"
#include "slic3r/Utils/UndoRedo.hpp"

#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PlugGenerator.hpp"
#include "libslic3r/HoleFinder.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Tesselate.hpp"

#include <glad/gl.h>

#include <boost/log/trivial.hpp>
#include <wx/event.h>

namespace Slic3r {
namespace GUI {

GLGizmoHoleFill::GLGizmoHoleFill(GLCanvas3D& parent, const std::string& icon_filename, unsigned int sprite_id)
    : GLGizmoBase(parent, icon_filename, sprite_id)
{
}

bool GLGizmoHoleFill::on_init()
{
    m_shortcut_key = WXK_CONTROL_H;

    m_desc["hole_fill_depth"] = _L("Fill depth");
    m_desc["angle_tolerance"] = _L("Angle tolerance");
    m_desc["filament"]        = _L("Filament");
    m_desc["pending_plugs"]   = _L("Pending plugs");
    m_desc["apply"]           = _L("Apply");
    m_desc["cancel"]          = _L("Cancel");
    m_desc["tool_hole_fill"]  = _L("Hole fill");

    init_extruders_data();
    return true;
}

std::string GLGizmoHoleFill::on_get_name() const
{
    return _u8L("Hole Fill");
}

bool GLGizmoHoleFill::on_is_activable() const
{
    const Selection& selection = m_parent.get_selection();
    return !selection.is_empty() && (selection.is_single_full_instance() || selection.is_any_volume());
}

bool GLGizmoHoleFill::on_is_selectable() const
{
    return (wxGetApp().preset_bundle->printers.get_edited_preset().printer_technology() == ptFFF
            && wxGetApp().filaments_cnt() > 1);
}

CommonGizmosDataID GLGizmoHoleFill::on_get_requirements() const
{
    return CommonGizmosDataID(
          int(CommonGizmosDataID::SelectionInfo)
        | int(CommonGizmosDataID::Raycaster));
}

void GLGizmoHoleFill::on_set_state()
{
    if (m_state == Off)
        reset_hover_state();
}

void GLGizmoHoleFill::data_changed(bool /*is_serializing*/)
{
    init_extruders_data();
    reset_hover_state();
}

void GLGizmoHoleFill::init_extruders_data()
{
    m_extruders_colors = wxGetApp().plater()->get_extruders_colors();
    if (m_selected_extruder_idx >= m_extruders_colors.size())
        m_selected_extruder_idx = 0;
}

void GLGizmoHoleFill::reset_hover_state()
{
    m_hover_hole_valid = false;
    m_hover_facet      = -1;
    m_hover_mesh_id    = -1;
    m_hover_outline_mesh.reset();
    m_hover_fill_mesh.reset();
    m_rr = RaycastResult{};
}

bool GLGizmoHoleFill::pick_mesh(const Vec2d& mouse_position, RaycastResult& out) const
{
    out = RaycastResult{};

    if (!m_c || !m_c->raycaster() || !m_c->selection_info())
        return false;

    const Selection& selection = m_parent.get_selection();
    if (selection.is_empty())
        return false;

    const ModelObject* mo = m_c->selection_info()->model_object();
    if (!mo || mo->instances.empty())
        return false;

    int inst_idx = selection.get_instance_idx();
    if (inst_idx < 0 || inst_idx >= int(mo->instances.size()))
        return false;

    const ModelInstance* mi     = mo->instances[inst_idx];
    const Camera&        camera = wxGetApp().plater()->get_camera();

    std::vector<const MeshRaycaster*> raycasters = m_c->raycaster()->raycasters();
    if (raycasters.empty())
        return false;

    // Iterate model-part volumes (matches raycaster indexing) and keep the closest hit.
    float best_depth = std::numeric_limits<float>::max();
    int   best_mesh  = -1;
    size_t best_facet = 0;
    Vec3f  best_hit  = Vec3f::Zero();

    int mesh_id = -1;
    for (const ModelVolume* mv : mo->volumes) {
        if (!mv->is_model_part())
            continue;
        ++mesh_id;
        if (mesh_id >= int(raycasters.size()))
            break;
        const MeshRaycaster* rc = raycasters[mesh_id];
        if (!rc)
            continue;

        Transform3d trafo = mi->get_transformation().get_matrix() * mv->get_matrix();
        Vec3f  hit_pos;
        Vec3f  hit_normal;
        size_t facet_idx = 0;
        if (!rc->unproject_on_mesh(mouse_position, trafo, camera, hit_pos, hit_normal, nullptr, &facet_idx))
            continue;

        Vec3d world_hit = trafo * hit_pos.cast<double>();
        Vec3d view_pt   = camera.get_view_matrix() * world_hit;
        float depth     = float(-view_pt.z());
        if (depth < best_depth) {
            best_depth = depth;
            best_mesh  = mesh_id;
            best_facet = facet_idx;
            best_hit   = hit_pos;
        }
    }

    if (best_mesh < 0)
        return false;

    out.mesh_id = best_mesh;
    out.facet   = int(best_facet);
    out.hit     = best_hit;
    return true;
}

void GLGizmoHoleFill::update_hover(const Vec2d& mouse_position)
{
    RaycastResult rr;
    if (!pick_mesh(mouse_position, rr)) {
        m_hover_hole_valid = false;
        m_hover_facet      = -1;
        m_hover_mesh_id    = -1;
        m_rr = RaycastResult{};
        return;
    }
    m_rr = rr;
}

bool GLGizmoHoleFill::on_mouse(const wxMouseEvent& mouse_event)
{
    if (mouse_event.Moving()) {
        const Vec2d mp(mouse_event.GetX(), mouse_event.GetY());
        update_hover(mp);
        m_parent.set_as_dirty();
        return false;
    }

    if (mouse_event.LeftDown() && !mouse_event.ShiftDown()) {
        const Vec2d mp(mouse_event.GetX(), mouse_event.GetY());
        update_hover(mp);
        perform_hole_fill(mp);
        return true;
    }

    if (mouse_event.GetWheelRotation() != 0 && mouse_event.ControlDown()) {
        m_depth = mouse_event.GetWheelRotation() > 0
            ? std::min(m_depth + HoleFillDepthStep, HoleFillDepthMax)
            : std::max(m_depth - HoleFillDepthStep, HoleFillDepthMin);
        m_parent.set_as_dirty();
        return true;
    }

    return false;
}

void GLGizmoHoleFill::perform_hole_fill(const Vec2d& mouse_position)
{
    const Selection& selection = m_parent.get_selection();
    if (selection.is_empty())
        return;

    const ModelObject* mo = m_c->selection_info()->model_object();
    if (!mo)
        return;

    int object_idx = selection.get_object_idx();
    if (object_idx < 0)
        return;

    if (m_rr.mesh_id < 0) {
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::RegularNotificationLevel,
            _u8L("No surface detected under cursor."));
        return;
    }

    // m_rr.mesh_id is the index into model-part volumes. Map it back to the ModelVolume.
    int model_part_idx = 0;
    const ModelVolume* hit_volume = nullptr;
    int hit_volume_raw_idx = -1;
    for (int vi = 0; vi < (int)mo->volumes.size(); ++vi) {
        if (!mo->volumes[vi]->is_model_part())
            continue;
        if (model_part_idx == m_rr.mesh_id) {
            hit_volume         = mo->volumes[vi];
            hit_volume_raw_idx = vi;
            break;
        }
        ++model_part_idx;
    }

    if (!hit_volume)
        return;

    const indexed_triangle_set& its = hit_volume->mesh().its;
    if (its.indices.empty())
        return;

    int   facet_idx = m_rr.facet;
    Vec3f hit_point = m_rr.hit;

    HoleBoundary boundary;
    bool found = find_nearest_hole(its, facet_idx, hit_point, boundary, m_angle_tolerance);
    if (!found) {
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::RegularNotificationLevel,
            _u8L("No hole detected at the clicked location. Try clicking closer to a hole boundary."));
        return;
    }

    BOOST_LOG_TRIVIAL(debug) << "[HoleFill] Boundary: loop=" << boundary.loop.size()
        << " pts, inner_loops=" << boundary.inner_loops.size()
        << ", normal=(" << boundary.plane_normal.x() << "," << boundary.plane_normal.y()
        << "," << boundary.plane_normal.z() << ")";
    TriangleMesh plug = generate_plug(boundary, m_depth);
    if (plug.empty()) {
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::WarningNotificationLevel,
            _u8L("Failed to generate hole fill plug. The hole geometry may be too complex."));
        return;
    }

    // Duplicate fill guard: compare bounding-box centers against existing HoleFill_ volumes.
    {
        const auto  plug_center   = plug.bounding_box().center();
        const float dup_threshold = 0.1f;
        for (const ModelVolume* vol : mo->volumes) {
            if (vol->name.rfind("HoleFill_", 0) != 0)
                continue;
            if (vol->name.rfind("HoleFill_neg_", 0) == 0)
                continue;
            const auto existing_center = vol->mesh().bounding_box().center();
            if ((plug_center.cast<double>() - existing_center.cast<double>()).norm() < dup_threshold) {
                wxGetApp().plater()->get_notification_manager()->push_notification(
                    NotificationType::CustomNotification,
                    NotificationManager::NotificationLevel::RegularNotificationLevel,
                    _u8L("This hole is already filled. Remove the existing HoleFill volume from the object list to refill."));
                return;
            }
        }
    }

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Hole fill color");

    // Copy source transform before add_volume (which may invalidate hit_volume).
    const Geometry::Transformation source_trafo = hit_volume->get_transformation();

    ModelObject* mo_mut = wxGetApp().model().objects[object_idx];
    ModelVolume* new_vol = mo_mut->add_volume(std::move(plug), ModelVolumeType::MODEL_PART, false);
    new_vol->set_new_unique_id();
    new_vol->name = "HoleFill_" + std::to_string(hit_volume_raw_idx) + "_f" + std::to_string(facet_idx);

    new_vol->config.set("extruder", (int)m_selected_extruder_idx + 1);
    new_vol->set_transformation(source_trafo);

    wxGetApp().plater()->update();
    wxGetApp().obj_list()->update_after_undo_redo();

    wxGetApp().plater()->get_notification_manager()->push_notification(
        NotificationType::CustomNotification,
        NotificationManager::NotificationLevel::RegularNotificationLevel,
        _u8L("Hole filled successfully! A new volume has been added with the selected filament."));
}

void GLGizmoHoleFill::render_hole_fill_hover()
{
    if (m_rr.mesh_id < 0) {
        m_hover_hole_valid = false;
        return;
    }

    const ModelObject* mo = m_c->selection_info()->model_object();
    if (!mo) return;

    const Selection& selection = m_parent.get_selection();

    // Map m_rr.mesh_id (model-part index) to the actual ModelVolume.
    int model_part_idx = 0;
    const ModelVolume* hit_volume = nullptr;
    for (int vi = 0; vi < (int)mo->volumes.size(); ++vi) {
        if (!mo->volumes[vi]->is_model_part())
            continue;
        if (model_part_idx == m_rr.mesh_id) {
            hit_volume = mo->volumes[vi];
            break;
        }
        ++model_part_idx;
    }
    if (!hit_volume) {
        m_hover_hole_valid = false;
        return;
    }

    int facet_idx = m_rr.facet;

    // Only recompute if the hover target changed.
    if (facet_idx != m_hover_facet || m_rr.mesh_id != m_hover_mesh_id) {
        m_hover_facet      = facet_idx;
        m_hover_mesh_id    = m_rr.mesh_id;
        m_hover_hole_valid = false;

        const indexed_triangle_set& its = hit_volume->mesh().its;
        if (!its.indices.empty()) {
            HoleBoundary boundary;
            bool found = find_nearest_hole(its, facet_idx, m_rr.hit, boundary, m_angle_tolerance);
            if (found && boundary.loop.size() >= 3) {
                m_hover_boundary   = std::move(boundary);
                m_hover_hole_valid = true;

                // Outline: line segments around boundary loop + any inner loops.
                m_hover_outline_mesh.reset();
                {
                    GLModel::Geometry init_data;
                    init_data.format = {GLModel::Geometry::EPrimitiveType::Lines,
                                        GLModel::Geometry::EVertexLayout::P3};
                    init_data.color = ColorRGBA(0.0f, 1.0f, 0.3f, 1.0f); // bright green

                    const auto& loop = m_hover_boundary.loop;
                    int n = (int)loop.size();
                    init_data.reserve_vertices(n);
                    init_data.reserve_indices(n * 2);

                    Vec3f nudge = m_hover_boundary.plane_normal.normalized() * 0.05f;
                    for (int i = 0; i < n; ++i)
                        init_data.add_vertex(Vec3f(loop[i] + nudge));

                    for (int i = 0; i < n; ++i)
                        init_data.add_line((unsigned int)i, (unsigned int)((i + 1) % n));

                    for (const auto& inner : m_hover_boundary.inner_loops) {
                        int base = (int)init_data.vertices_count();
                        int in_n = (int)inner.size();
                        for (int i = 0; i < in_n; ++i)
                            init_data.add_vertex(Vec3f(inner[i] + nudge));
                        for (int i = 0; i < in_n; ++i)
                            init_data.add_line((unsigned int)(base + i), (unsigned int)(base + (i + 1) % in_n));
                    }

                    m_hover_outline_mesh.init_from(std::move(init_data));
                }

                // Fill: translucent cap in the selected filament color.
                m_hover_fill_mesh.reset();
                {
                    Vec3f normal = m_hover_boundary.plane_normal.normalized();
                    Vec3f origin = m_hover_boundary.plane_origin;
                    Vec3f nudge  = normal * 0.04f;

                    Vec3f u, v;
                    build_plane_frame(normal, u, v);

                    const auto& loop = m_hover_boundary.loop;
                    int n = (int)loop.size();

                    Polygon poly_2d;
                    poly_2d.points.reserve(n);
                    for (int i = 0; i < n; ++i) {
                        Vec2d p = project_to_2d(loop[i], origin, u, v);
                        poly_2d.points.emplace_back(Point(scale_(p.x()), scale_(p.y())));
                    }
                    if (poly_2d.is_clockwise())
                        poly_2d.reverse();

                    ExPolygon expoly;
                    expoly.contour = std::move(poly_2d);
                    for (const auto& inner : m_hover_boundary.inner_loops) {
                        Polygon hole_2d;
                        hole_2d.points.reserve(inner.size());
                        for (const Vec3f& pt : inner) {
                            Vec2d p = project_to_2d(pt, origin, u, v);
                            hole_2d.points.emplace_back(Point(scale_(p.x()), scale_(p.y())));
                        }
                        if (hole_2d.is_counter_clockwise())
                            hole_2d.reverse();
                        expoly.holes.push_back(std::move(hole_2d));
                    }

                    std::vector<Vec2d> tri_pts_2d = triangulate_expolygon_2d(expoly, NORMALS_UP);
                    int num_tris = (int)tri_pts_2d.size() / 3;
                    if (num_tris > 0) {
                        GLModel::Geometry init_data;
                        init_data.format = {GLModel::Geometry::EPrimitiveType::Triangles,
                                            GLModel::Geometry::EVertexLayout::P3};
                        ColorRGBA fill_color = m_extruders_colors.empty()
                            ? ColorRGBA(1.0f, 0.5f, 0.8f, 0.35f)
                            : m_extruders_colors[m_selected_extruder_idx % m_extruders_colors.size()];
                        fill_color.a(0.35f);
                        init_data.color = fill_color;

                        init_data.reserve_vertices(num_tris * 3);
                        init_data.reserve_indices(num_tris * 3);

                        for (const Vec2d& p : tri_pts_2d) {
                            Vec3f pt3d = unproject_to_3d(p, origin, u, v) + nudge;
                            init_data.add_vertex(pt3d);
                        }
                        for (int t = 0; t < num_tris; ++t) {
                            unsigned int base = (unsigned int)(t * 3);
                            init_data.add_triangle(base, base + 1, base + 2);
                        }

                        m_hover_fill_mesh.init_from(std::move(init_data));
                    }
                }
            }
        }
    }

    if (!m_hover_hole_valid)
        return;

    // Render using the volume's world transform.
    const ModelInstance* mi = mo->instances[selection.get_instance_idx()];
    Transform3d model_trafo = mi->get_transformation().get_matrix() * hit_volume->get_matrix();
    const Camera& camera = wxGetApp().plater()->get_camera();
    Transform3d view_model_matrix = camera.get_view_matrix() * model_trafo;

    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    glsafe(::glDepthMask(GL_FALSE));

    auto shader_fill = wxGetApp().get_shader("flat");
    if (shader_fill) {
        shader_fill->start_using();
        shader_fill->set_uniform("view_model_matrix", view_model_matrix);
        shader_fill->set_uniform("projection_matrix", camera.get_projection_matrix());
        m_hover_fill_mesh.render();
        shader_fill->stop_using();
    }

    glsafe(::glDepthMask(GL_TRUE));

#if !SLIC3R_OPENGL_ES
    if (!OpenGLManager::get_gl_info().is_core_profile())
        glsafe(::glLineWidth(2.5f));
#endif

    auto shader_line = wxGetApp().get_shader("flat");
    if (shader_line) {
        shader_line->start_using();
        shader_line->set_uniform("view_model_matrix", view_model_matrix);
        shader_line->set_uniform("projection_matrix", camera.get_projection_matrix());
        m_hover_outline_mesh.render();
        shader_line->stop_using();
    }

#if !SLIC3R_OPENGL_ES
    if (!OpenGLManager::get_gl_info().is_core_profile())
        glsafe(::glLineWidth(1.0f));
#endif

    glsafe(::glDisable(GL_BLEND));
}

void GLGizmoHoleFill::on_render()
{
    render_hole_fill_hover();
}

void GLGizmoHoleFill::on_render_input_window(float x, float y, float bottom_limit)
{
    if (!m_c || !m_c->selection_info() || !m_c->selection_info()->model_object())
        return;

    // Refresh extruder colors in case they changed since data_changed().
    const auto current_colors = wxGetApp().plater()->get_extruders_colors();
    if (current_colors != m_extruders_colors) {
        m_extruders_colors = current_colors;
        if (m_selected_extruder_idx >= m_extruders_colors.size())
            m_selected_extruder_idx = 0;
    }

    const float approx_height = m_imgui->scaled(14.0f);
    y = std::min(y, bottom_limit - approx_height);

    ImGuiWrapper::push_toolbar_style(m_parent.get_scale());
    GizmoImguiSetNextWIndowPos(x, y, ImGuiCond_Always);
    GizmoImguiBegin(get_name(), ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);

    const float slider_icon_width = m_imgui->get_slider_icon_size().x;
    const float sliders_width     = m_imgui->scaled(7.0f);
    const float drag_left_width   = m_imgui->scaled(0.5f);
    const float label_left_width  = std::max(
        m_imgui->calc_text_size(m_desc.at("hole_fill_depth")).x,
        m_imgui->calc_text_size(m_desc.at("angle_tolerance")).x) + m_imgui->scaled(1.5f);

    // Depth slider + drag.
    ImGui::AlignTextToFramePadding();
    m_imgui->text(m_desc.at("hole_fill_depth") + ":");
    ImGui::SameLine(label_left_width);
    ImGui::PushItemWidth(sliders_width);
    std::string depth_fmt = std::string("%.1f ") + I18N::translate_utf8("mm", "Hole fill depth");
    m_imgui->bbl_slider_float_style("##hole_fill_depth", &m_depth, HoleFillDepthMin, HoleFillDepthMax, depth_fmt.data(), 1.0f, true);
    ImGui::SameLine(drag_left_width + label_left_width);
    ImGui::PushItemWidth(1.5f * slider_icon_width);
    ImGui::BBLDragFloat("##hole_fill_depth_input", &m_depth, 0.05f, 0.0f, 0.0f, "%.1f");
    m_depth = std::clamp(m_depth, HoleFillDepthMin, HoleFillDepthMax);

    // Angle tolerance slider.
    ImGui::AlignTextToFramePadding();
    m_imgui->text(m_desc.at("angle_tolerance") + ":");
    ImGui::SameLine(label_left_width);
    ImGui::PushItemWidth(sliders_width);
    std::string angle_fmt = std::string("%.1f") + I18N::translate_utf8("°", "Angle tolerance");
    m_imgui->bbl_slider_float_style("##hole_fill_angle", &m_angle_tolerance, 1.0f, 30.0f, angle_fmt.data(), 1.0f, true);
    ImGui::SameLine(drag_left_width + label_left_width);
    ImGui::PushItemWidth(1.5f * slider_icon_width);
    ImGui::BBLDragFloat("##hole_fill_angle_input", &m_angle_tolerance, 0.05f, 0.0f, 0.0f, "%.1f");

    ImGui::Separator();

    // Filament color picker row.
    m_imgui->text(m_desc.at("filament") + ":");
    const int items_per_row = 8;
    for (size_t i = 0; i < m_extruders_colors.size(); ++i) {
        if (i > 0 && (i % items_per_row) != 0)
            ImGui::SameLine();
        ImGui::PushID((int)i);
        ImVec4 col = ImGuiWrapper::to_ImVec4(m_extruders_colors[i]);
        ImGui::PushStyleColor(ImGuiCol_Border,
            (m_selected_extruder_idx == i) ? ImGuiWrapper::COL_ORCA
                                           : ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, (m_selected_extruder_idx == i) ? 2.0f : 0.0f);
        if (ImGui::ColorButton("##filament", col, ImGuiColorEditFlags_NoTooltip, ImVec2(20, 20)))
            m_selected_extruder_idx = i;
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
        ImGui::PopID();
    }

    ImGui::Separator();

    // Pending plug list (placeholder — filled in in a later step).
    m_imgui->text(m_desc.at("pending_plugs") + ": 0");

    ImGui::Separator();

    // Apply / Cancel (stubs — wired up in a later step).
    if (m_imgui->button(m_desc.at("apply"))) {
        // Stubbed: no pending list yet; left-click currently applies immediately.
    }
    ImGui::SameLine();
    if (m_imgui->button(m_desc.at("cancel"))) {
        reset_hover_state();
    }

    GizmoImguiEnd();
    ImGuiWrapper::pop_toolbar_style();
}

} // namespace GUI
} // namespace Slic3r
