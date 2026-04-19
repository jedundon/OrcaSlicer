#include "GLGizmoHoleFill.hpp"

#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/GUI_ObjectList.hpp"
#include "slic3r/GUI/Selection.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/MeshUtils.hpp"
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

#include <array>
#include <cmath>
#include <cstdint>
#include <queue>
#include <set>

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
    m_desc["hole_cut_depth"]  = _L("Cut depth");
    m_desc["angle_tolerance"] = _L("Angle tolerance");
    m_desc["filament"]        = _L("Filament");
    m_desc["pending_plugs"]   = _L("Pending plugs");
    m_desc["apply"]           = _L("Apply");
    m_desc["cancel"]          = _L("Cancel");
    m_desc["tool_hole_fill"]  = _L("Hole fill");
    m_desc["mode"]            = _L("Mode");
    m_desc["mode_fill"]       = _L("Fill");
    m_desc["mode_cut"]        = _L("Cut");
    m_desc["island_warning"]  = _L("Warning: This hole has islands that may become disconnected after cutting. Consider using Fill mode instead.");
    m_desc["shift_fill_all"]  = _L("Shift+Click: Fill all holes on surface");
    m_desc["shift_remove_all"] = _L("Shift+Click plug: remove all on surface");

    init_extruders_data();
    return true;
}

std::string GLGizmoHoleFill::on_get_name() const
{
    return _u8L("Hole Fill");
}

bool GLGizmoHoleFill::on_is_activable() const
{
    // During batch fill update cycles, reload_scene() transiently empties
    // the multi-object selection.  Keep the gizmo alive until the deferred
    // CallAfter clears the flag.
    if (m_suppress_deactivation) {
        BOOST_LOG_TRIVIAL(warning) << "[HoleFill] on_is_activable: suppressed during batch update";
        return true;
    }
    const Selection& selection = m_parent.get_selection();
    bool result = !selection.is_empty()
        && (selection.is_single_full_instance()
            || selection.is_any_volume()
            || selection.is_multiple_full_instance()
            || selection.is_multiple_full_object());
    if (!result)
        BOOST_LOG_TRIVIAL(warning) << "[HoleFill] on_is_activable: FALSE"
            << " empty=" << selection.is_empty()
            << " single_full_inst=" << selection.is_single_full_instance()
            << " any_vol=" << selection.is_any_volume()
            << " multi_full_inst=" << selection.is_multiple_full_instance()
            << " multi_full_obj=" << selection.is_multiple_full_object()
            << " mixed=" << selection.is_mixed()
            << " content_size=" << selection.get_content().size()
            << " volume_count=" << selection.get_volume_idxs().size();
    return result;
}

bool GLGizmoHoleFill::is_batch_selection() const
{
    const Selection& selection = m_parent.get_selection();
    return selection.is_multiple_full_instance() || selection.is_multiple_full_object();
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
    BOOST_LOG_TRIVIAL(warning) << "[HoleFill] on_set_state: state=" << (int)m_state;
    if (m_state == Off)
        reset_hover_state();
}

void GLGizmoHoleFill::data_changed(bool /*is_serializing*/)
{
    init_extruders_data();

    BOOST_LOG_TRIVIAL(warning) << "[HoleFill] data_changed called, suppress=" << m_suppress_data_changed
                               << " batch_state=" << (int)m_batch_state;

    // When our own plater()->update() triggers a scene reload we still need
    // fresh extruder colors, but clearing the raycaster cache and batch state
    // would break the next click in a consecutive same-color batch fill.
    if (m_suppress_data_changed) {
        m_suppress_data_changed = false;
        return;
    }

    reset_hover_state();
    m_multi_raycasters.clear();
    // A selection change mid-batch would desync state; clear it.
    clear_batch_preview();
    m_batch_state = BatchState::Inactive;
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
    m_hover_object_idx   = -1;
    m_hover_instance_idx = -1;
    m_hover_outline_mesh.reset();
    m_hover_fill_mesh.reset();
    m_hover_is_plug             = false;
    m_hover_plug_raw_idx        = -1;
    m_remove_outline_cached_idx = -1;
    m_remove_outline_mesh.reset();
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

bool GLGizmoHoleFill::pick_mesh_multi(const Vec2d& mouse_position, RaycastResult& out)
{
    out = RaycastResult{};

    const Selection& selection = m_parent.get_selection();
    if (selection.is_empty())
        return false;

    const Model* model = selection.get_model();
    if (!model)
        return false;

    const Camera& camera = wxGetApp().plater()->get_camera();
    const auto& content = selection.get_content();

    float best_depth = std::numeric_limits<float>::max();
    int   best_mesh  = -1;
    size_t best_facet = 0;
    Vec3f  best_hit  = Vec3f::Zero();
    int    best_obj  = -1;
    int    best_inst = -1;

    for (const auto& [obj_idx, inst_idxs] : content) {
        if (obj_idx < 0 || obj_idx >= (int)model->objects.size())
            continue;
        const ModelObject* mo = model->objects[obj_idx];
        if (!mo || mo->instances.empty())
            continue;

        // Use the first selected instance for raycasting.
        int inst_idx = inst_idxs.empty() ? 0 : *inst_idxs.begin();
        if (inst_idx < 0 || inst_idx >= (int)mo->instances.size())
            continue;
        const ModelInstance* mi = mo->instances[inst_idx];

        int mesh_id = -1;
        for (int vi = 0; vi < (int)mo->volumes.size(); ++vi) {
            const ModelVolume* mv = mo->volumes[vi];
            if (!mv->is_model_part())
                continue;
            ++mesh_id;

            // Get or create cached raycaster for this volume.
            auto key = std::make_pair(obj_idx, vi);
            auto it = m_multi_raycasters.find(key);
            if (it == m_multi_raycasters.end()) {
                it = m_multi_raycasters.emplace(key,
                    std::make_unique<MeshRaycaster>(std::make_shared<const TriangleMesh>(mv->mesh()))).first;
            }
            const MeshRaycaster* rc = it->second.get();
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
                best_obj   = obj_idx;
                best_inst  = inst_idx;
            }
        }
    }

    if (best_mesh < 0)
        return false;

    out.mesh_id      = best_mesh;
    out.facet        = int(best_facet);
    out.hit          = best_hit;
    out.object_idx   = best_obj;
    out.instance_idx = best_inst;
    return true;
}

void GLGizmoHoleFill::update_hover(const Vec2d& mouse_position)
{
    RaycastResult rr;
    const bool multi = is_batch_selection();
    bool hit = multi ? pick_mesh_multi(mouse_position, rr)
                     : pick_mesh(mouse_position, rr);
    if (!hit) {
        m_hover_hole_valid   = false;
        m_hover_facet        = -1;
        m_hover_mesh_id      = -1;
        m_hover_is_plug      = false;
        m_hover_plug_raw_idx = -1;
        m_hover_object_idx   = -1;
        m_hover_instance_idx = -1;
        m_rr = RaycastResult{};
        return;
    }
    m_rr = rr;
    m_hover_object_idx   = rr.object_idx;
    m_hover_instance_idx = rr.instance_idx;

    // Detect whether the hit is on an existing HoleFill_ plug volume.
    // HoleFill_neg_ volumes are negatives and should not enter remove mode.
    m_hover_is_plug      = false;
    m_hover_plug_raw_idx = -1;

    const ModelObject* mo = nullptr;
    if (multi) {
        const Model* model = m_parent.get_selection().get_model();
        if (model && rr.object_idx >= 0 && rr.object_idx < (int)model->objects.size())
            mo = model->objects[rr.object_idx];
    } else if (m_c && m_c->selection_info()) {
        mo = m_c->selection_info()->model_object();
    }

    if (mo) {
        int model_part_idx = 0;
        for (int vi = 0; vi < (int)mo->volumes.size(); ++vi) {
            const ModelVolume* mv = mo->volumes[vi];
            if (!mv->is_model_part())
                continue;
            if (model_part_idx == m_rr.mesh_id) {
                if (mv->name.rfind("HoleFill_", 0) == 0 &&
                    mv->name.rfind("HoleFill_neg_", 0) != 0) {
                    m_hover_is_plug      = true;
                    m_hover_plug_raw_idx = vi;
                }
                break;
            }
            ++model_part_idx;
        }
    }
}

bool GLGizmoHoleFill::on_mouse(const wxMouseEvent& mouse_event)
{
    if (mouse_event.Moving()) {
        const Vec2d mp(mouse_event.GetX(), mouse_event.GetY());
        update_hover(mp);
        m_parent.set_as_dirty();
        return false;
    }

    if (mouse_event.LeftDown()) {
        const Vec2d mp(mouse_event.GetX(), mouse_event.GetY());
        update_hover(mp);
        if (m_hover_is_plug) {
            if (mouse_event.ShiftDown())
                perform_remove_all_on_surface(mp);
            else
                perform_hole_remove();
        } else if (mouse_event.ShiftDown())
            perform_fill_all_on_surface(mp);
        else if (is_batch_selection())
            perform_batch_fill(mp);
        else
            perform_hole_fill(mp);
        reset_hover_state();  // W3: clear stale hover after fill/remove
        return true;
    }

    if (mouse_event.GetWheelRotation() != 0 && mouse_event.ControlDown()) {
        const float depth_max = (m_mode == Mode::Cut) ? HoleCutDepthMax : HoleFillDepthMax;
        m_depth = mouse_event.GetWheelRotation() > 0
            ? std::min(m_depth + HoleFillDepthStep, depth_max)
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

    // W4: re-pick in case raycaster rebuilt since hover (e.g. prior fill added a volume)
    if (m_rr.mesh_id < 0 && !pick_mesh(mouse_position, m_rr)) {
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::RegularNotificationLevel,
            _u8L("No surface detected under cursor."));
        return;
    }
    if (m_rr.mesh_id < 0)
        return;

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

    const bool is_cut = (m_mode == Mode::Cut);

    // Island warning: a cut through a hole with inner loops leaves disconnected geometry.
    if (is_cut && !boundary.inner_loops.empty()) {
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::WarningNotificationLevel,
            _u8L("Warning: This hole has islands that may become disconnected after cutting. Consider using Fill mode instead."));
    }

    TriangleMesh plug = generate_plug(boundary, m_depth);
    if (plug.empty()) {
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::WarningNotificationLevel,
            is_cut ? _u8L("Failed to generate hole cut volume. The hole geometry may be too complex.")
                   : _u8L("Failed to generate hole fill plug. The hole geometry may be too complex."));
        return;
    }

    // Duplicate guard: compare bounding-box centers against existing volumes of the same kind.
    {
        const auto  plug_center   = plug.bounding_box().center();
        const float dup_threshold = 0.1f;
        for (const ModelVolume* vol : mo->volumes) {
            if (vol->name.rfind("HoleFill_", 0) != 0)
                continue;
            const bool vol_is_neg = vol->name.rfind("HoleFill_neg_", 0) == 0;
            if (vol_is_neg != is_cut)
                continue;
            const auto existing_center = vol->mesh().bounding_box().center();
            if ((plug_center.cast<double>() - existing_center.cast<double>()).norm() < dup_threshold) {
                wxGetApp().plater()->get_notification_manager()->push_notification(
                    NotificationType::CustomNotification,
                    NotificationManager::NotificationLevel::RegularNotificationLevel,
                    is_cut ? _u8L("This hole is already cut. Remove the existing HoleFill_neg volume from the object list to re-cut.")
                           : _u8L("This hole is already filled. Remove the existing HoleFill volume from the object list to refill."));
                return;
            }
        }
    }

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), is_cut ? "Hole cut" : "Hole fill color");

    // Copy source transform before add_volume (which may invalidate hit_volume).
    const Geometry::Transformation source_trafo = hit_volume->get_transformation();

    ModelObject* mo_mut = wxGetApp().model().objects[object_idx];
    const ModelVolumeType new_type = is_cut ? ModelVolumeType::NEGATIVE_VOLUME : ModelVolumeType::MODEL_PART;
    ModelVolume* new_vol = mo_mut->add_volume(std::move(plug), new_type, false);
    new_vol->set_new_unique_id();
    const std::string name_prefix = is_cut ? "HoleFill_neg_" : "HoleFill_";
    new_vol->name = name_prefix + std::to_string(hit_volume_raw_idx) + "_f" + std::to_string(facet_idx);

    if (!is_cut)
        new_vol->config.set("extruder", (int)m_selected_extruder_idx + 1);
    new_vol->set_transformation(source_trafo);

    wxGetApp().plater()->update();
    wxGetApp().obj_list()->update_info_items((size_t)object_idx);

    wxGetApp().plater()->get_notification_manager()->push_notification(
        NotificationType::CustomNotification,
        NotificationManager::NotificationLevel::RegularNotificationLevel,
        is_cut ? _u8L("Hole cut successfully! A negative volume has been added to subtract from the object.")
               : _u8L("Hole filled successfully! A new volume has been added with the selected filament."));
}

void GLGizmoHoleFill::perform_hole_remove()
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

    if (m_hover_plug_raw_idx < 0 || m_hover_plug_raw_idx >= (int)mo->volumes.size())
        return;

    // Re-validate that the resolved volume is still a fill plug. Indices can
    // shift between hover and click if the model was edited elsewhere.
    const ModelVolume* vol = mo->volumes[m_hover_plug_raw_idx];
    if (!vol
        || vol->name.rfind("HoleFill_", 0) != 0
        || vol->name.rfind("HoleFill_neg_", 0) == 0)
        return;

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Remove hole fill");

    ModelObject* mo_mut = wxGetApp().model().objects[object_idx];
    mo_mut->delete_volume((size_t)m_hover_plug_raw_idx);

    wxGetApp().plater()->update();
    // Re-select the object so reload_scene doesn't find an empty selection
    // and deactivate the gizmo (the deleted GLVolume invalidated old indices).
    m_parent.get_selection().add_object((unsigned int)object_idx, true);
    wxGetApp().obj_list()->update_info_items((size_t)object_idx);

    wxGetApp().plater()->get_notification_manager()->push_notification(
        NotificationType::CustomNotification,
        NotificationManager::NotificationLevel::RegularNotificationLevel,
        _u8L("Hole fill removed."));
}

void GLGizmoHoleFill::perform_fill_all_on_surface(const Vec2d& mouse_position)
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

    if (m_rr.mesh_id < 0 && !pick_mesh(mouse_position, m_rr)) {
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::RegularNotificationLevel,
            _u8L("No surface detected under cursor."));
        return;
    }
    if (m_rr.mesh_id < 0)
        return;

    int model_part_idx = 0;
    const ModelVolume* hit_volume     = nullptr;
    int                hit_volume_raw_idx = -1;
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

    const int facet_idx = m_rr.facet;
    const bool is_cut   = (m_mode == Mode::Cut);

    // find_hole_boundaries already returns ALL holes on the coplanar region
    // surrounding the seed facet — flood-fills coplanar faces, chains boundary
    // edges into loops, and classifies perimeter vs holes vs islands.
    std::vector<HoleBoundary> boundaries =
        find_hole_boundaries(its, facet_idx, m_angle_tolerance);
    if (boundaries.empty()) {
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::RegularNotificationLevel,
            _u8L("No holes detected on this surface."));
        return;
    }

    // Collect existing HoleFill_ plug centers (of the same kind as the current
    // mode — fill plugs vs negative cut volumes) for duplicate rejection.
    std::vector<Vec3d> existing_centers;
    for (const ModelVolume* vol : mo->volumes) {
        if (vol->name.rfind("HoleFill_", 0) != 0)
            continue;
        const bool vol_is_neg = vol->name.rfind("HoleFill_neg_", 0) == 0;
        if (vol_is_neg != is_cut)
            continue;
        existing_centers.push_back(vol->mesh().bounding_box().center());
    }
    const double dup_threshold = 0.1; // 100 µm, matches single-hole path

    Plater::TakeSnapshot snapshot(wxGetApp().plater(),
        is_cut ? "Cut all holes on surface" : "Fill all holes on surface");

    const Geometry::Transformation source_trafo = hit_volume->get_transformation();
    ModelObject* mo_mut = wxGetApp().model().objects[object_idx];

    int filled  = 0;
    int skipped = 0;
    int failed  = 0;

    const ModelVolumeType new_type = is_cut ? ModelVolumeType::NEGATIVE_VOLUME : ModelVolumeType::MODEL_PART;
    const std::string name_prefix  = is_cut ? "HoleFill_neg_" : "HoleFill_";

    for (size_t i = 0; i < boundaries.size(); ++i) {
        const HoleBoundary& boundary = boundaries[i];
        TriangleMesh plug = generate_plug(boundary, m_depth);
        if (plug.empty()) {
            ++failed;
            continue;
        }

        const Vec3d plug_center = plug.bounding_box().center();
        bool dupe = false;
        for (const Vec3d& ec : existing_centers) {
            if ((plug_center - ec).norm() < dup_threshold) {
                dupe = true;
                break;
            }
        }
        if (dupe) {
            ++skipped;
            continue;
        }

        ModelVolume* new_vol = mo_mut->add_volume(std::move(plug), new_type, false);
        new_vol->set_new_unique_id();
        new_vol->name = name_prefix + std::to_string(hit_volume_raw_idx)
                        + "_surf" + std::to_string(facet_idx)
                        + "_" + std::to_string(i);
        if (!is_cut)
            new_vol->config.set("extruder", (int)m_selected_extruder_idx + 1);
        new_vol->set_transformation(source_trafo);

        // Guard against in-batch duplicates (e.g. overlapping detections).
        existing_centers.push_back(plug_center);
        ++filled;
    }

    wxGetApp().plater()->update();
    wxGetApp().obj_list()->update_info_items((size_t)object_idx);

    std::string msg;
    if (filled > 0 && skipped > 0)
        msg = is_cut
            ? Slic3r::GUI::format(_L("Cut %1% holes (%2% already cut)."), filled, skipped)
            : Slic3r::GUI::format(_L("Filled %1% holes (%2% already filled)."), filled, skipped);
    else if (filled > 0)
        msg = is_cut
            ? Slic3r::GUI::format(_L("Cut %1% holes on surface."), filled)
            : Slic3r::GUI::format(_L("Filled %1% holes on surface."), filled);
    else if (skipped > 0)
        msg = is_cut
            ? Slic3r::GUI::format(_L("All %1% holes on this surface are already cut."), skipped)
            : Slic3r::GUI::format(_L("All %1% holes on this surface are already filled."), skipped);
    else
        msg = is_cut
            ? _u8L("No holes could be cut on this surface.")
            : _u8L("No holes could be filled on this surface.");
    if (failed > 0)
        msg += " " + Slic3r::GUI::format(_L("(%1% failed.)"), failed);

    wxGetApp().plater()->get_notification_manager()->push_notification(
        NotificationType::CustomNotification,
        NotificationManager::NotificationLevel::RegularNotificationLevel,
        msg);
}

void GLGizmoHoleFill::perform_remove_all_on_surface(const Vec2d& /*mouse_position*/)
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

    if (m_hover_plug_raw_idx < 0 || m_hover_plug_raw_idx >= (int)mo->volumes.size())
        return;
    const ModelVolume* hover_plug = mo->volumes[m_hover_plug_raw_idx];
    if (!hover_plug
        || hover_plug->name.rfind("HoleFill_", 0) != 0
        || hover_plug->name.rfind("HoleFill_neg_", 0) == 0)
        return;

    // Parse "HoleFill_[neg_]{vol_idx}_(f{facet}|surf{facet}_{hole})".
    auto parse_src_and_facet = [](const std::string& name, int& out_vol, int& out_facet) -> bool {
        const std::string prefix = "HoleFill_";
        if (name.rfind(prefix, 0) != 0)
            return false;
        size_t pos = prefix.size();
        if (name.compare(pos, 4, "neg_") == 0)
            pos += 4;
        size_t us = name.find('_', pos);
        if (us == std::string::npos || us == pos)
            return false;
        try {
            out_vol = std::stoi(name.substr(pos, us - pos));
        } catch (...) { return false; }
        pos = us + 1;
        if (pos < name.size() && name[pos] == 'f')
            pos += 1;
        else if (name.compare(pos, 4, "surf") == 0)
            pos += 4;
        else
            return false;
        size_t end = name.find('_', pos);
        std::string num = (end == std::string::npos) ? name.substr(pos) : name.substr(pos, end - pos);
        if (num.empty())
            return false;
        try {
            out_facet = std::stoi(num);
        } catch (...) { return false; }
        return true;
    };

    int src_vol_idx = -1;
    int src_facet   = -1;
    if (!parse_src_and_facet(hover_plug->name, src_vol_idx, src_facet))
        return;
    if (src_vol_idx < 0 || src_vol_idx >= (int)mo->volumes.size())
        return;
    const ModelVolume* src_vol = mo->volumes[src_vol_idx];
    if (!src_vol || !src_vol->is_model_part()
        || src_vol->name.rfind("HoleFill_", 0) == 0)
        return;

    const indexed_triangle_set& src_its = src_vol->mesh().its;
    if (src_facet < 0 || src_facet >= (int)src_its.indices.size())
        return;

    const auto& tri = src_its.indices[src_facet];
    const Vec3d v0  = src_its.vertices[tri[0]].cast<double>();
    const Vec3d v1  = src_its.vertices[tri[1]].cast<double>();
    const Vec3d v2  = src_its.vertices[tri[2]].cast<double>();
    Vec3d normal = (v1 - v0).cross(v2 - v0);
    if (normal.norm() < 1e-9)
        return;
    normal.normalize();

    const Vec3d ref_center = hover_plug->mesh().bounding_box().center();
    const double tol = 0.5; // 0.5 mm plane tolerance

    std::vector<int> to_delete;
    to_delete.reserve(8);
    for (int vi = 0; vi < (int)mo->volumes.size(); ++vi) {
        const ModelVolume* v = mo->volumes[vi];
        if (!v)
            continue;
        if (v->name.rfind("HoleFill_", 0) != 0)
            continue;
        int vol_idx = -1, facet = -1;
        if (!parse_src_and_facet(v->name, vol_idx, facet))
            continue;
        if (vol_idx != src_vol_idx)
            continue;
        const Vec3d c = v->mesh().bounding_box().center();
        const double d = std::abs((c - ref_center).dot(normal));
        if (d < tol)
            to_delete.push_back(vi);
    }

    if (to_delete.empty())
        return;

    // Delete in descending order so earlier deletions don't shift later indices.
    std::sort(to_delete.begin(), to_delete.end());

    Plater::TakeSnapshot snapshot(wxGetApp().plater(), "Remove all hole fills on surface");

    ModelObject* mo_mut = wxGetApp().model().objects[object_idx];
    for (auto it = to_delete.rbegin(); it != to_delete.rend(); ++it)
        mo_mut->delete_volume((size_t)*it);

    wxGetApp().plater()->update();
    m_parent.get_selection().add_object((unsigned int)object_idx, true);
    wxGetApp().obj_list()->update_info_items((size_t)object_idx);

    const int removed = (int)to_delete.size();
    wxGetApp().plater()->get_notification_manager()->push_notification(
        NotificationType::CustomNotification,
        NotificationManager::NotificationLevel::RegularNotificationLevel,
        Slic3r::GUI::format(_L("Removed %1% hole fill(s) from surface."), removed));
}

void GLGizmoHoleFill::perform_batch_fill(const Vec2d& mouse_position)
{
    // Instant batch fill: discover matches + generate plugs in one shot.
    // Stays in Inactive state so the gizmo remains open for the next click.
    RaycastResult rr;
    const bool multi = is_batch_selection();
    bool hit = multi ? pick_mesh_multi(mouse_position, rr)
                     : pick_mesh(mouse_position, rr);
    if (!hit)
        return;

    const Selection& selection = m_parent.get_selection();

    // Resolve the ModelObject for the hit.
    const ModelObject* mo = nullptr;
    int object_idx = -1;
    int inst_idx   = -1;
    if (multi && rr.object_idx >= 0) {
        const Model* model = selection.get_model();
        if (model && rr.object_idx < (int)model->objects.size()) {
            mo = model->objects[rr.object_idx];
            object_idx = rr.object_idx;
            inst_idx   = rr.instance_idx;
        }
    } else {
        mo = m_c->selection_info()->model_object();
        object_idx = selection.get_object_idx();
        inst_idx   = selection.get_instance_idx();
    }
    if (!mo || object_idx < 0)
        return;

    // Map mesh_id (model-part index) back to a ModelVolume.
    int model_part_idx = 0;
    int hit_volume_raw_idx = -1;
    for (int vi = 0; vi < (int)mo->volumes.size(); ++vi) {
        if (!mo->volumes[vi]->is_model_part())
            continue;
        if (model_part_idx == rr.mesh_id) {
            hit_volume_raw_idx = vi;
            break;
        }
        ++model_part_idx;
    }
    if (hit_volume_raw_idx < 0)
        return;

    // Ignore clicks on existing plugs.
    const ModelVolume* vol = mo->volumes[hit_volume_raw_idx];
    if (vol->name.rfind("HoleFill_", 0) == 0)
        return;

    const auto& its = vol->mesh().its;
    if (rr.facet < 0 || rr.facet >= (int)its.indices.size())
        return;

    // Capture reference info for discover_batch_matches.
    m_batch_ref_object_idx = object_idx;
    m_batch_ref_volume_idx = hit_volume_raw_idx;
    m_batch_ref_facet      = rr.facet;

    const auto& tri = its.indices[rr.facet];
    Vec3f v0 = its.vertices[tri[0]];
    Vec3f v1 = its.vertices[tri[1]];
    Vec3f v2 = its.vertices[tri[2]];
    Vec3f local_n = (v1 - v0).cross(v2 - v0).normalized();

    if (inst_idx < 0) inst_idx = 0;
    if (inst_idx >= (int)mo->instances.size())
        return;
    Transform3d trafo = mo->instances[inst_idx]->get_transformation().get_matrix() * vol->get_matrix();
    m_batch_ref_world_normal = (trafo.linear().inverse().transpose().cast<float>() * local_n).normalized();

    // Phase 1: discover all matching holes across selected objects.
    m_batch_preview.clear();
    discover_batch_matches();

    BOOST_LOG_TRIVIAL(warning) << "[HoleFill] perform_batch_fill: discovered " << m_batch_preview.size() << " holes";

    if (m_batch_preview.empty()) {
        clear_batch_preview();
        wxGetApp().plater()->get_notification_manager()->push_notification(
            NotificationType::CustomNotification,
            NotificationManager::NotificationLevel::RegularNotificationLevel,
            _u8L("No matching holes found across selected objects."));
        m_parent.set_as_dirty();
        return;
    }

    // Phase 2: generate plugs immediately (same logic as commit_batch).
    const bool is_cut = (m_mode == Mode::Cut);
    const ModelVolumeType new_type = is_cut ? ModelVolumeType::NEGATIVE_VOLUME : ModelVolumeType::MODEL_PART;
    const std::string name_prefix  = is_cut ? "HoleFill_neg_" : "HoleFill_";

    Plater::TakeSnapshot snapshot(wxGetApp().plater(),
        Slic3r::GUI::format("Batch hole fill (%1% plugs)", (int)m_batch_preview.size()));

    int filled = 0, skipped = 0, failed = 0;
    std::set<int> touched_objects;

    // Per-object cache of existing same-kind plug centers for duplicate rejection.
    std::map<int, std::vector<Vec3d>> existing_centers_by_obj;
    auto get_existing_centers = [&](int obj_idx) -> std::vector<Vec3d>& {
        auto it = existing_centers_by_obj.find(obj_idx);
        if (it != existing_centers_by_obj.end())
            return it->second;
        std::vector<Vec3d>& centers = existing_centers_by_obj[obj_idx];
        const ModelObject* m = wxGetApp().model().objects[obj_idx];
        for (const ModelVolume* v : m->volumes) {
            if (v->name.rfind("HoleFill_", 0) != 0)
                continue;
            const bool v_is_neg = v->name.rfind("HoleFill_neg_", 0) == 0;
            if (v_is_neg != is_cut)
                continue;
            centers.push_back(v->mesh().bounding_box().center());
        }
        return centers;
    };

    for (const auto& entry : m_batch_preview) {
        if (entry.object_idx < 0 || entry.object_idx >= (int)wxGetApp().model().objects.size())
            continue;
        ModelObject* mo_mut = wxGetApp().model().objects[entry.object_idx];
        if (!mo_mut)
            continue;
        if (entry.volume_idx < 0 || entry.volume_idx >= (int)mo_mut->volumes.size())
            continue;
        const ModelVolume* src_vol = mo_mut->volumes[entry.volume_idx];
        if (!src_vol)
            continue;

        TriangleMesh plug = generate_plug(entry.boundary, m_depth);
        if (plug.empty()) {
            ++failed;
            continue;
        }

        const Vec3d plug_center = plug.bounding_box().center();
        std::vector<Vec3d>& existing_centers = get_existing_centers(entry.object_idx);
        bool dupe = false;
        for (const Vec3d& ec : existing_centers) {
            if ((plug_center - ec).norm() < 0.1) {
                dupe = true;
                break;
            }
        }
        if (dupe) {
            ++skipped;
            continue;
        }

        const Geometry::Transformation source_trafo = src_vol->get_transformation();
        ModelVolume* new_vol = mo_mut->add_volume(std::move(plug), new_type, false);
        new_vol->set_new_unique_id();
        new_vol->name = name_prefix + std::to_string(entry.volume_idx)
                        + "_f" + std::to_string(entry.seed_facet)
                        + "_b" + std::to_string(filled);
        if (!is_cut)
            new_vol->config.set("extruder", (int)m_selected_extruder_idx + 1);
        new_vol->set_transformation(source_trafo);

        existing_centers.push_back(plug_center);
        touched_objects.insert(entry.object_idx);
        ++filled;
    }

    // Only call plater()->update() when we actually added volumes.
    if (filled > 0) {
        // Capture selected object indices BEFORE update — reload_scene may
        // empty the selection when new GLVolumes don't match old geometry_ids.
        std::vector<int> selected_obj_idxs;
        for (const auto& [obj_idx, inst_set] : selection.get_content())
            selected_obj_idxs.push_back(obj_idx);

        m_suppress_data_changed = true;
        m_suppress_deactivation = true;
        wxGetApp().plater()->update();

        // Re-assert multi-object selection immediately after the synchronous
        // part of update().
        m_parent.get_selection().add_object_from_idx(selected_obj_idxs);

        // Defer clearing the suppress flag until ALL pending async events
        // (EVT_GLCANVAS_OBJECT_SELECT etc.) have been processed.  Also do a
        // final re-select in case async events wiped the selection again.
        wxGetApp().CallAfter([this, selected_obj_idxs]() {
            if (m_parent.get_selection().is_empty()) {
                std::vector<int> idxs = selected_obj_idxs;
                m_parent.get_selection().add_object_from_idx(idxs);
            }
            m_suppress_deactivation = false;
        });

        for (int oi : touched_objects)
            wxGetApp().obj_list()->update_info_items((size_t)oi);
    }

    std::string msg = Slic3r::GUI::format(_L("Batch fill: %1% plugs added"), filled);
    if (skipped > 0)
        msg += Slic3r::GUI::format(_L(", %1% skipped (duplicates)"), skipped);
    if (failed > 0)
        msg += Slic3r::GUI::format(_L(", %1% failed"), failed);
    wxGetApp().plater()->get_notification_manager()->push_notification(
        NotificationType::CustomNotification,
        NotificationManager::NotificationLevel::RegularNotificationLevel,
        msg);

    // Stay in Inactive state — gizmo remains open for next click.
    clear_batch_preview();
    m_parent.set_as_dirty();
}

void GLGizmoHoleFill::enter_batch_preview(const Vec2d& mouse_position)
{
    // Phase 1: capture the reference hole and transition to Preview state.
    // Discovery across other objects/instances arrives in Phase 2.
    RaycastResult rr;
    const bool multi = is_batch_selection();
    bool hit = multi ? pick_mesh_multi(mouse_position, rr)
                     : pick_mesh(mouse_position, rr);
    if (!hit)
        return;

    const Selection& selection = m_parent.get_selection();

    // Resolve the ModelObject for the hit — either from multi-raycast result or SelectionInfo.
    const ModelObject* mo = nullptr;
    int object_idx = -1;
    int inst_idx   = -1;
    if (multi && rr.object_idx >= 0) {
        const Model* model = selection.get_model();
        if (model && rr.object_idx < (int)model->objects.size()) {
            mo = model->objects[rr.object_idx];
            object_idx = rr.object_idx;
            inst_idx   = rr.instance_idx;
        }
    } else {
        mo = m_c->selection_info()->model_object();
        object_idx = selection.get_object_idx();
        inst_idx   = selection.get_instance_idx();
    }
    if (!mo || object_idx < 0)
        return;

    // Map mesh_id (model-part index) back to a ModelVolume.
    int model_part_idx = 0;
    int hit_volume_raw_idx = -1;
    for (int vi = 0; vi < (int)mo->volumes.size(); ++vi) {
        if (!mo->volumes[vi]->is_model_part())
            continue;
        if (model_part_idx == rr.mesh_id) {
            hit_volume_raw_idx = vi;
            break;
        }
        ++model_part_idx;
    }
    if (hit_volume_raw_idx < 0)
        return;

    // Ignore clicks on existing plugs — those stay in single-object remove flow.
    const ModelVolume* vol = mo->volumes[hit_volume_raw_idx];
    if (vol->name.rfind("HoleFill_", 0) == 0)
        return;

    const auto& its = vol->mesh().its;
    if (rr.facet < 0 || rr.facet >= (int)its.indices.size())
        return;

    m_batch_ref_object_idx = object_idx;
    m_batch_ref_volume_idx = hit_volume_raw_idx;
    m_batch_ref_facet      = rr.facet;

    // World-space normal of the clicked triangle — used by Phase 2 to match faces.
    const auto& tri = its.indices[rr.facet];
    Vec3f v0 = its.vertices[tri[0]];
    Vec3f v1 = its.vertices[tri[1]];
    Vec3f v2 = its.vertices[tri[2]];
    Vec3f local_n = (v1 - v0).cross(v2 - v0).normalized();

    // Compute world transform from model data (works for both single and multi-select).
    if (inst_idx < 0) inst_idx = 0;
    if (inst_idx >= (int)mo->instances.size())
        return;
    Transform3d trafo = mo->instances[inst_idx]->get_transformation().get_matrix() * vol->get_matrix();
    m_batch_ref_world_normal = (trafo.linear().inverse().transpose().cast<float>() * local_n).normalized();

    m_batch_state = BatchState::Preview;
    m_batch_preview.clear();

    discover_batch_matches();

    BOOST_LOG_TRIVIAL(warning) << "[HoleFill] enter_batch_preview done: preview.size()=" << m_batch_preview.size();

    m_parent.set_as_dirty();
}

void GLGizmoHoleFill::discover_batch_matches()
{
    m_batch_preview.clear();

    const Selection& selection = m_parent.get_selection();
    const auto& content = selection.get_content();

    BOOST_LOG_TRIVIAL(warning) << "[HoleFill] discover_batch_matches: content.size()=" << content.size()
        << " ref_obj=" << m_batch_ref_object_idx
        << " ref_vol=" << m_batch_ref_volume_idx
        << " ref_facet=" << m_batch_ref_facet
        << " ref_normal=(" << m_batch_ref_world_normal.x() << "," << m_batch_ref_world_normal.y() << "," << m_batch_ref_world_normal.z() << ")"
        << " scope=" << (int)m_batch_scope
        << " all_instances=" << m_batch_all_instances;

    const float cos_tol = std::cos(m_angle_tolerance * (float)M_PI / 180.0f);
    // 0.5 mm plane distance tolerance for SingleSurface (matches remove-all heuristic).
    const double plane_dist_tol = 0.5;

    // World-space plane of the reference hole (for SingleSurface scope).
    // Computed from the reference boundary if available, else from the ref facet.
    Vec3d ref_plane_pt = Vec3d::Zero();
    bool  have_ref_plane = false;
    if (m_batch_ref_object_idx >= 0
        && m_batch_ref_object_idx < (int)wxGetApp().model().objects.size()) {
        const ModelObject* ref_mo = wxGetApp().model().objects[m_batch_ref_object_idx];
        if (ref_mo
            && m_batch_ref_volume_idx >= 0
            && m_batch_ref_volume_idx < (int)ref_mo->volumes.size()) {
            const ModelVolume* ref_vol = ref_mo->volumes[m_batch_ref_volume_idx];
            const indexed_triangle_set& ref_its = ref_vol->mesh().its;
            if (m_batch_ref_facet >= 0 && m_batch_ref_facet < (int)ref_its.indices.size()) {
                // Use the first (closest) selected instance of the reference object.
                int ref_inst = -1;
                auto it = content.find(m_batch_ref_object_idx);
                if (it != content.end() && !it->second.empty())
                    ref_inst = *it->second.begin();
                if (ref_inst >= 0 && ref_inst < (int)ref_mo->instances.size()) {
                    Transform3d ref_trafo =
                        ref_mo->instances[ref_inst]->get_transformation().get_matrix()
                        * ref_vol->get_matrix();
                    const auto& rf = ref_its.indices[m_batch_ref_facet];
                    Vec3d c = (ref_its.vertices[rf[0]].cast<double>()
                             + ref_its.vertices[rf[1]].cast<double>()
                             + ref_its.vertices[rf[2]].cast<double>()) / 3.0;
                    ref_plane_pt = ref_trafo * c;
                    have_ref_plane = true;
                }
            }
        }
    }

    for (const auto& kv : content) {
        const int obj_idx = kv.first;
        const auto& inst_idxs = kv.second;
        if (obj_idx < 0 || obj_idx >= (int)wxGetApp().model().objects.size())
            continue;
        const ModelObject* mo = wxGetApp().model().objects[obj_idx];
        if (!mo)
            continue;

        std::vector<int> instances_to_process;
        if (m_batch_all_instances) {
            instances_to_process.reserve(mo->instances.size());
            for (int i = 0; i < (int)mo->instances.size(); ++i)
                instances_to_process.push_back(i);
        } else {
            instances_to_process.reserve(inst_idxs.size());
            for (int idx : inst_idxs)
                instances_to_process.push_back(idx);
        }

        for (int inst_idx : instances_to_process) {
            if (inst_idx < 0 || inst_idx >= (int)mo->instances.size())
                continue;
            const ModelInstance* mi = mo->instances[inst_idx];

            for (int vi = 0; vi < (int)mo->volumes.size(); ++vi) {
                const ModelVolume* vol = mo->volumes[vi];
                if (!vol || !vol->is_model_part())
                    continue;
                if (vol->name.rfind("HoleFill_", 0) == 0)
                    continue;

                const indexed_triangle_set& its = vol->mesh().its;
                if (its.indices.empty())
                    continue;

                Transform3d    trafo       = mi->get_transformation().get_matrix() * vol->get_matrix();
                Eigen::Matrix3f normal_mat = trafo.linear().inverse().transpose().cast<float>();

                BOOST_LOG_TRIVIAL(warning) << "[HoleFill] discover obj=" << obj_idx << " inst=" << inst_idx
                    << " vol=" << vi << " name=" << vol->name << " tris=" << its.indices.size();

                // Build face neighbors once per volume for flood-fill region tracking.
                const std::vector<Vec3i32> face_neighbors = its_face_neighbors(its);
                const float cos_tol_local = std::cos(m_angle_tolerance * (float)M_PI / 180.0f);

                // Track which facets have been assigned to a coplanar region so we
                // visit each region at most once.
                std::vector<bool> visited(its.indices.size(), false);
                int regions_tried = 0, regions_with_holes = 0;

                for (int fi = 0; fi < (int)its.indices.size(); ++fi) {
                    if (visited[fi])
                        continue;

                    // Compute face normal.
                    const auto& tri = its.indices[fi];
                    const Vec3f& v0 = its.vertices[tri[0]];
                    const Vec3f& v1 = its.vertices[tri[1]];
                    const Vec3f& v2 = its.vertices[tri[2]];
                    Vec3f local_n = (v1 - v0).cross(v2 - v0);
                    const float len = local_n.norm();
                    if (len < 1e-9f) {
                        visited[fi] = true;
                        continue;
                    }
                    local_n /= len;

                    // Check if this face's world normal matches the reference.
                    bool matches = false;
                    if (m_batch_scope == BatchScope::AllFaces) {
                        matches = true;
                    } else {
                        Vec3f world_n = (normal_mat * local_n).normalized();
                        matches = world_n.dot(m_batch_ref_world_normal) >= cos_tol;
                    }

                    if (!matches) {
                        visited[fi] = true;
                        continue;
                    }

                    // Flood-fill from this facet to mark the entire coplanar region
                    // as visited, regardless of whether it contains holes.
                    std::queue<int> ff_queue;
                    ff_queue.push(fi);
                    visited[fi] = true;
                    while (!ff_queue.empty()) {
                        int cf = ff_queue.front();
                        ff_queue.pop();
                        for (int ni = 0; ni < 3; ++ni) {
                            int nb = face_neighbors[cf][ni];
                            if (nb < 0 || visited[nb])
                                continue;
                            // Check coplanarity with the seed normal.
                            const auto& ntri = its.indices[nb];
                            Vec3f nn = (its.vertices[ntri[1]] - its.vertices[ntri[0]])
                                .cross(its.vertices[ntri[2]] - its.vertices[ntri[0]]);
                            float nn_len = nn.norm();
                            if (nn_len > 1e-9f && (nn / nn_len).dot(local_n) >= cos_tol_local) {
                                visited[nb] = true;
                                ff_queue.push(nb);
                            }
                        }
                    }

                    ++regions_tried;

                    // Now call find_hole_boundaries with this seed.
                    std::vector<HoleBoundary> boundaries =
                        find_hole_boundaries(its, fi, m_angle_tolerance);
                    if (boundaries.empty())
                        continue;
                    ++regions_with_holes;

                    // SingleSurface: drop results whose plane doesn't match the reference.
                    bool skip_single_surface = false;
                    if (m_batch_scope == BatchScope::SingleSurface && have_ref_plane) {
                        // Representative plane point for this seed's region, in world space.
                        Vec3d seed_world = trafo * boundaries.front().plane_origin.cast<double>();
                        Vec3f world_n = (normal_mat * boundaries.front().plane_normal.normalized()).normalized();
                        double dist = std::abs((seed_world - ref_plane_pt).dot(world_n.cast<double>()));
                        if (dist > plane_dist_tol)
                            skip_single_surface = true;
                    }
                    if (skip_single_surface)
                        continue;

                    for (size_t bi = 0; bi < boundaries.size(); ++bi) {
                        BatchPreviewEntry entry;
                        entry.object_idx   = obj_idx;
                        entry.instance_idx = inst_idx;
                        entry.volume_idx   = vi;
                        entry.seed_facet   = fi;
                        entry.boundary     = boundaries[bi];
                        entry.world_center = trafo * boundaries[bi].plane_origin.cast<double>();
                        entry.is_reference = (obj_idx == m_batch_ref_object_idx
                                              && vi == m_batch_ref_volume_idx);
                        m_batch_preview.push_back(std::move(entry));
                    }
                }

                BOOST_LOG_TRIVIAL(warning) << "[HoleFill] discover obj=" << obj_idx << " vol=" << vi
                    << " regions_tried=" << regions_tried << " regions_with_holes=" << regions_with_holes
                    << " total_preview=" << m_batch_preview.size();
            }
        }
    }
}

void GLGizmoHoleFill::commit_batch()
{
    if (m_batch_preview.empty()) {
        cancel_batch();
        return;
    }

    const bool is_cut = (m_mode == Mode::Cut);
    const ModelVolumeType new_type = is_cut ? ModelVolumeType::NEGATIVE_VOLUME : ModelVolumeType::MODEL_PART;
    const std::string name_prefix  = is_cut ? "HoleFill_neg_" : "HoleFill_";

    Plater::TakeSnapshot snapshot(wxGetApp().plater(),
        Slic3r::GUI::format("Batch hole fill (%1% plugs)", (int)m_batch_preview.size()));

    int filled = 0, skipped = 0, failed = 0;
    std::set<int> touched_objects;

    // Per-object cache of existing same-kind plug centers for duplicate rejection.
    std::map<int, std::vector<Vec3d>> existing_centers_by_obj;
    auto get_existing_centers = [&](int obj_idx) -> std::vector<Vec3d>& {
        auto it = existing_centers_by_obj.find(obj_idx);
        if (it != existing_centers_by_obj.end())
            return it->second;
        std::vector<Vec3d>& centers = existing_centers_by_obj[obj_idx];
        const ModelObject* mo = wxGetApp().model().objects[obj_idx];
        for (const ModelVolume* v : mo->volumes) {
            if (v->name.rfind("HoleFill_", 0) != 0)
                continue;
            const bool v_is_neg = v->name.rfind("HoleFill_neg_", 0) == 0;
            if (v_is_neg != is_cut)
                continue;
            centers.push_back(v->mesh().bounding_box().center());
        }
        return centers;
    };

    for (const auto& entry : m_batch_preview) {
        if (entry.object_idx < 0 || entry.object_idx >= (int)wxGetApp().model().objects.size())
            continue;
        ModelObject* mo = wxGetApp().model().objects[entry.object_idx];
        if (!mo)
            continue;
        if (entry.volume_idx < 0 || entry.volume_idx >= (int)mo->volumes.size())
            continue;
        const ModelVolume* src_vol = mo->volumes[entry.volume_idx];
        if (!src_vol)
            continue;

        TriangleMesh plug = generate_plug(entry.boundary, m_depth);
        if (plug.empty()) {
            ++failed;
            continue;
        }

        const Vec3d plug_center = plug.bounding_box().center();
        std::vector<Vec3d>& existing_centers = get_existing_centers(entry.object_idx);
        bool dupe = false;
        for (const Vec3d& ec : existing_centers) {
            if ((plug_center - ec).norm() < 0.1) {
                dupe = true;
                break;
            }
        }
        if (dupe) {
            ++skipped;
            continue;
        }

        const Geometry::Transformation source_trafo = src_vol->get_transformation();
        ModelVolume* new_vol = mo->add_volume(std::move(plug), new_type, false);
        new_vol->set_new_unique_id();
        new_vol->name = name_prefix + std::to_string(entry.volume_idx)
                        + "_f" + std::to_string(entry.seed_facet)
                        + "_b" + std::to_string(filled);
        if (!is_cut)
            new_vol->config.set("extruder", (int)m_selected_extruder_idx + 1);
        new_vol->set_transformation(source_trafo);

        existing_centers.push_back(plug_center);
        touched_objects.insert(entry.object_idx);
        ++filled;
    }

    wxGetApp().plater()->update();
    for (int obj_idx : touched_objects)
        wxGetApp().obj_list()->update_info_items((size_t)obj_idx);

    std::string msg = Slic3r::GUI::format(_L("Batch fill: %1% plugs added"), filled);
    if (skipped > 0)
        msg += Slic3r::GUI::format(_L(", %1% skipped (duplicates)"), skipped);
    if (failed > 0)
        msg += Slic3r::GUI::format(_L(", %1% failed"), failed);
    wxGetApp().plater()->get_notification_manager()->push_notification(
        NotificationType::CustomNotification,
        NotificationManager::NotificationLevel::RegularNotificationLevel,
        msg);

    // Close the gizmo after batch apply so the user gets immediate visual
    // feedback (new volumes visible) and can re-open for the next face.
    m_batch_state = BatchState::Inactive;
    clear_batch_preview();
    m_parent.set_as_dirty();
    m_parent.get_gizmos_manager().reset_all_states();
}

void GLGizmoHoleFill::cancel_batch()
{
    clear_batch_preview();
    m_batch_state = BatchState::Inactive;
    m_parent.set_as_dirty();
}

void GLGizmoHoleFill::clear_batch_preview()
{
    m_batch_preview.clear();
    m_batch_ref_object_idx   = -1;
    m_batch_ref_volume_idx   = -1;
    m_batch_ref_facet        = -1;
    m_batch_ref_world_normal = Vec3f::Zero();
}

void GLGizmoHoleFill::render_remove_hover()
{
    if (!m_hover_is_plug || m_hover_plug_raw_idx < 0)
        return;

    const ModelObject* mo = nullptr;
    if (m_hover_object_idx >= 0) {
        const Model* model = m_parent.get_selection().get_model();
        if (model && m_hover_object_idx < (int)model->objects.size())
            mo = model->objects[m_hover_object_idx];
    } else if (m_c && m_c->selection_info()) {
        mo = m_c->selection_info()->model_object();
    }
    if (!mo || m_hover_plug_raw_idx >= (int)mo->volumes.size())
        return;

    const ModelVolume* hit_volume = mo->volumes[m_hover_plug_raw_idx];
    if (!hit_volume)
        return;

    const Selection& selection = m_parent.get_selection();
    int inst_idx = (m_hover_instance_idx >= 0) ? m_hover_instance_idx : selection.get_instance_idx();
    if (inst_idx < 0 || inst_idx >= int(mo->instances.size()))
        return;

    // Build a red wireframe of the plug's local AABB. Rebuild only when the
    // hovered plug changes so we don't thrash the GLModel every frame.
    if (m_hover_plug_raw_idx != m_remove_outline_cached_idx) {
        m_remove_outline_cached_idx = m_hover_plug_raw_idx;
        m_remove_outline_mesh.reset();

        const BoundingBoxf3 bbox = hit_volume->mesh().bounding_box();
        const Vec3f mn = bbox.min.cast<float>();
        const Vec3f mx = bbox.max.cast<float>();

        const Vec3f corners[8] = {
            { mn.x(), mn.y(), mn.z() }, { mx.x(), mn.y(), mn.z() },
            { mx.x(), mx.y(), mn.z() }, { mn.x(), mx.y(), mn.z() },
            { mn.x(), mn.y(), mx.z() }, { mx.x(), mn.y(), mx.z() },
            { mx.x(), mx.y(), mx.z() }, { mn.x(), mx.y(), mx.z() },
        };
        static const unsigned int edges[12][2] = {
            {0,1},{1,2},{2,3},{3,0},
            {4,5},{5,6},{6,7},{7,4},
            {0,4},{1,5},{2,6},{3,7},
        };

        GLModel::Geometry init_data;
        init_data.format = { GLModel::Geometry::EPrimitiveType::Lines,
                             GLModel::Geometry::EVertexLayout::P3 };
        init_data.color  = ColorRGBA(1.0f, 0.15f, 0.15f, 1.0f); // red
        init_data.reserve_vertices(8);
        init_data.reserve_indices(24);
        for (int i = 0; i < 8; ++i)
            init_data.add_vertex(corners[i]);
        for (int i = 0; i < 12; ++i)
            init_data.add_line(edges[i][0], edges[i][1]);

        m_remove_outline_mesh.init_from(std::move(init_data));
    }

    const ModelInstance* mi = mo->instances[inst_idx];
    Transform3d model_trafo = mi->get_transformation().get_matrix() * hit_volume->get_matrix();
    const Camera& camera = wxGetApp().plater()->get_camera();
    Transform3d view_model_matrix = camera.get_view_matrix() * model_trafo;

    glsafe(::glDisable(GL_DEPTH_TEST));
#if !SLIC3R_OPENGL_ES
    if (!OpenGLManager::get_gl_info().is_core_profile())
        glsafe(::glLineWidth(3.0f));
#endif

    auto shader = wxGetApp().get_shader("flat");
    if (shader) {
        shader->start_using();
        shader->set_uniform("view_model_matrix", view_model_matrix);
        shader->set_uniform("projection_matrix", camera.get_projection_matrix());
        m_remove_outline_mesh.render();
        shader->stop_using();
    }

#if !SLIC3R_OPENGL_ES
    if (!OpenGLManager::get_gl_info().is_core_profile())
        glsafe(::glLineWidth(1.0f));
#endif
    glsafe(::glEnable(GL_DEPTH_TEST));
}

void GLGizmoHoleFill::render_hole_fill_hover()
{
    if (m_rr.mesh_id < 0) {
        m_hover_hole_valid = false;
        return;
    }

    const ModelObject* mo = nullptr;
    if (m_hover_object_idx >= 0) {
        const Model* model = m_parent.get_selection().get_model();
        if (model && m_hover_object_idx < (int)model->objects.size())
            mo = model->objects[m_hover_object_idx];
    } else if (m_c && m_c->selection_info()) {
        mo = m_c->selection_info()->model_object();
    }
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

    // Only recompute if the hover target or mode changed.
    if (facet_idx != m_hover_facet || m_rr.mesh_id != m_hover_mesh_id || m_mode != m_hover_mode) {
        m_hover_facet      = facet_idx;
        m_hover_mesh_id    = m_rr.mesh_id;
        m_hover_mode       = m_mode;
        m_hover_hole_valid = false;

        const indexed_triangle_set& its = hit_volume->mesh().its;
        if (!its.indices.empty()) {
            HoleBoundary boundary;
            bool found = find_nearest_hole(its, facet_idx, m_rr.hit, boundary, m_angle_tolerance);
            if (found && boundary.loop.size() >= 3) {
                m_hover_boundary   = std::move(boundary);
                m_hover_hole_valid = true;

                // Outline: line segments around boundary loop + any inner loops.
                const bool hover_is_cut = (m_mode == Mode::Cut);
                m_hover_outline_mesh.reset();
                {
                    GLModel::Geometry init_data;
                    init_data.format = {GLModel::Geometry::EPrimitiveType::Lines,
                                        GLModel::Geometry::EVertexLayout::P3};
                    init_data.color = hover_is_cut
                        ? ColorRGBA(1.0f, 0.55f, 0.0f, 1.0f)   // amber for Cut
                        : ColorRGBA(0.0f, 1.0f,  0.3f, 1.0f);  // bright green for Fill

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

                // Fill: translucent cap in the selected filament color (Fill) or dark (Cut).
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
                        ColorRGBA fill_color;
                        if (hover_is_cut) {
                            fill_color = ColorRGBA(0.08f, 0.08f, 0.08f, 0.45f); // translucent dark for material removal
                        } else {
                            fill_color = m_extruders_colors.empty()
                                ? ColorRGBA(1.0f, 0.5f, 0.8f, 0.35f)
                                : m_extruders_colors[m_selected_extruder_idx % m_extruders_colors.size()];
                            fill_color.a(0.35f);
                        }
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
    int inst_idx = (m_hover_instance_idx >= 0) ? m_hover_instance_idx : selection.get_instance_idx();
    if (inst_idx < 0 || inst_idx >= int(mo->instances.size())) { m_hover_hole_valid = false; return; }
    const ModelInstance* mi = mo->instances[inst_idx];
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
    if (m_hover_is_plug)
        render_remove_hover();
    else
        render_hole_fill_hover();
}

void GLGizmoHoleFill::on_render_input_window(float x, float y, float bottom_limit)
{
    if (!m_c || !m_c->selection_info())
        return;
    // model_object() is null when multiple objects are selected (get_object_idx returns -1).
    // In batch mode we still need to render the UI, so only bail for single-object mode.
    const bool multi_select = is_batch_selection();
    if (!multi_select && !m_c->selection_info()->model_object())
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

    if (is_batch_selection()) {
        const Selection& sel   = m_parent.get_selection();
        const int        n_obj = (int)sel.get_content().size();

        ImGuiWrapper::text_colored(ImGuiWrapper::COL_ORANGE_LIGHT,
            Slic3r::GUI::format(_L("Batch mode: %1% objects selected"), n_obj));
        ImGui::Separator();

        // Scope dropdown — controls which faces are matched.
        int scope_idx = (int)m_batch_scope;
        static std::array<std::string, 3> scope_labels;
        scope_labels = {
            _u8L("All faces"), _u8L("Matching normal"), _u8L("Single surface")
        };
        if (ImGui::Combo("##batch_scope", &scope_idx,
                [](void* data, int idx, const char** out) {
                    auto* labels = static_cast<std::array<std::string, 3>*>(data);
                    *out = (*labels)[idx].c_str();
                    return true;
                },
                &scope_labels, 3)) {
            m_batch_scope = static_cast<BatchScope>(scope_idx);
        }

        ImGui::Checkbox(_u8L("All instances of each object").c_str(), &m_batch_all_instances);

        ImGui::Separator();
        m_imgui->text(_L("Click a hole to fill matching holes across all objects."));

        ImGui::Separator();
    }

    const float slider_icon_width = m_imgui->get_slider_icon_size().x;
    const float sliders_width     = m_imgui->scaled(7.0f);
    const float drag_left_width   = m_imgui->scaled(0.5f);
    const float label_left_width  = std::max({
        m_imgui->calc_text_size(m_desc.at("hole_fill_depth")).x,
        m_imgui->calc_text_size(m_desc.at("hole_cut_depth")).x,
        m_imgui->calc_text_size(m_desc.at("angle_tolerance")).x,
        m_imgui->calc_text_size(m_desc.at("mode")).x}) + m_imgui->scaled(1.5f);

    // Mode toggle: Fill vs Cut.
    ImGui::AlignTextToFramePadding();
    m_imgui->text(m_desc.at("mode") + ":");
    ImGui::SameLine(label_left_width);
    if (ImGui::RadioButton((m_desc.at("mode_fill") + "##hf_mode_fill").utf8_str().data(), m_mode == Mode::Fill))
        m_mode = Mode::Fill;
    ImGui::SameLine();
    if (ImGui::RadioButton((m_desc.at("mode_cut") + "##hf_mode_cut").utf8_str().data(), m_mode == Mode::Cut))
        m_mode = Mode::Cut;

    // Dynamic mode indicator: Remove when hovering a plug, otherwise Fill/Cut.
    if (m_hover_is_plug) {
        ImGui::TextColored(ImVec4(1.0f, 0.25f, 0.25f, 1.0f),
                           "%s", _u8L("Mode: Remove (click to delete)").c_str());
    } else if (m_mode == Mode::Cut) {
        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.0f, 1.0f),
                           "%s", _u8L("Mode: Cut").c_str());
    } else {
        m_imgui->text(_u8L("Mode: Fill"));
    }
    ImGui::Separator();

    const bool is_cut       = (m_mode == Mode::Cut);
    const float depth_max   = is_cut ? HoleCutDepthMax : HoleFillDepthMax;
    const wxString& depth_label = is_cut ? m_desc.at("hole_cut_depth") : m_desc.at("hole_fill_depth");

    // Depth slider + drag.
    ImGui::AlignTextToFramePadding();
    m_imgui->text(depth_label + ":");
    ImGui::SameLine(label_left_width);
    ImGui::PushItemWidth(sliders_width);
    std::string depth_fmt = std::string("%.1f ") + I18N::translate_utf8("mm", "Hole fill depth");
    m_imgui->bbl_slider_float_style("##hole_fill_depth", &m_depth, HoleFillDepthMin, depth_max, depth_fmt.data(), 1.0f, true);
    ImGui::SameLine(drag_left_width + label_left_width);
    ImGui::PushItemWidth(1.5f * slider_icon_width);
    ImGui::BBLDragFloat("##hole_fill_depth_input", &m_depth, 0.05f, 0.0f, 0.0f, "%.1f");
    m_depth = std::clamp(m_depth, HoleFillDepthMin, depth_max);

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
    m_angle_tolerance = std::clamp(m_angle_tolerance, 1.0f, 30.0f);

    ImGui::Separator();

    // Filament color picker row. Disabled in Cut mode (negative volumes have no filament).
    m_imgui->disabled_begin(is_cut);
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
            if (!is_cut) m_selected_extruder_idx = i;
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
        ImGui::PopID();
    }
    m_imgui->disabled_end();

    // Single-click UI: only show when NOT in batch mode.
    if (!is_batch_selection()) {
        ImGui::Separator();

        // Pending plug list (placeholder — filled in in a later step).
        m_imgui->text(m_desc.at("pending_plugs") + ": 0");

        m_imgui->text(m_desc.at("shift_fill_all"));

        ImGui::Separator();

        // Apply / Cancel (stubs — wired up in a later step).
        // Apply is disabled until pending-plug workflow is implemented (step 5).
        m_imgui->disabled_begin(true);
        m_imgui->button(m_desc.at("apply"));
        m_imgui->disabled_end();
        ImGui::SameLine();
        if (m_imgui->button(m_desc.at("cancel"))) {
            reset_hover_state();
        }
    }

    GizmoImguiEnd();
    ImGuiWrapper::pop_toolbar_style();
}

} // namespace GUI
} // namespace Slic3r
