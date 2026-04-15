#include "HoleFinder.hpp"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <boost/log/trivial.hpp>

namespace Slic3r {

// ─── helpers ──────────────────────────────────────────────────────────────────

static Vec3f face_normal(const indexed_triangle_set &its, int face_idx)
{
    const Vec3i32 &f = its.indices[face_idx];
    const Vec3f &v0 = its.vertices[f[0]];
    const Vec3f &v1 = its.vertices[f[1]];
    const Vec3f &v2 = its.vertices[f[2]];
    Vec3f n = (v1 - v0).cross(v2 - v0);
    float len = n.norm();
    if (len > 0.f)
        return Vec3f(n / len);
    return Vec3f::Zero();
}

// Canonical edge key: ordered pair (min, max) of vertex indices.
struct EdgeKey {
    int v0, v1;
    EdgeKey(int a, int b) : v0(std::min(a, b)), v1(std::max(a, b)) {}
    bool operator==(const EdgeKey &o) const { return v0 == o.v0 && v1 == o.v1; }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey &e) const {
        // Combine two ints into one hash.
        return std::hash<int64_t>()(((int64_t)e.v0 << 32) | (int64_t)(unsigned)e.v1);
    }
};

// Directed half-edge for chaining boundary loops.
struct HalfEdge {
    int from, to;      // vertex indices
    int face_idx;      // face this edge belongs to
};

// ─── main implementation ─────────────────────────────────────────────────────

std::vector<HoleBoundary> find_hole_boundaries(
    const indexed_triangle_set &its,
    int                         seed_facet_idx,
    float                       angle_tolerance_deg)
{
    std::vector<HoleBoundary> result;

    if (seed_facet_idx < 0 || seed_facet_idx >= (int)its.indices.size())
        return result;

    const float cos_tolerance = std::cos(angle_tolerance_deg * (float)M_PI / 180.f);

    // Step 1: Compute face neighbors.
    std::vector<Vec3i32> neighbors = its_face_neighbors(its);

    // Step 2: Flood-fill from seed to collect coplanar face region.
    Vec3f seed_normal = face_normal(its, seed_facet_idx);
    if (seed_normal.squaredNorm() < 1e-12f)
        return result;

    std::vector<bool> in_region(its.indices.size(), false);
    std::queue<int> queue;
    queue.push(seed_facet_idx);
    in_region[seed_facet_idx] = true;

    while (!queue.empty()) {
        int fi = queue.front();
        queue.pop();

        for (int ni = 0; ni < 3; ++ni) {
            int neighbor = neighbors[fi][ni];
            if (neighbor < 0 || in_region[neighbor])
                continue;
            Vec3f nn = face_normal(its, neighbor);
            if (nn.dot(seed_normal) >= cos_tolerance) {
                in_region[neighbor] = true;
                queue.push(neighbor);
            }
        }
    }

    // Step 3: Find boundary edges of the region.
    // A boundary edge is one where:
    //   (a) the adjacent face is not in the region, OR
    //   (b) there is no adjacent face (open mesh edge).
    // We store them as directed half-edges (oriented CCW with respect to
    // the face they belong to).
    std::vector<HalfEdge> boundary_edges;

    for (int fi = 0; fi < (int)its.indices.size(); ++fi) {
        if (!in_region[fi])
            continue;
        const Vec3i32 &f = its.indices[fi];
        for (int ei = 0; ei < 3; ++ei) {
            int neighbor = neighbors[fi][ei];
            if (neighbor < 0 || !in_region[neighbor]) {
                // This edge is a boundary of our region.
                int v_from = f[ei];
                int v_to   = f[(ei + 1) % 3];
                boundary_edges.push_back({v_from, v_to, fi});
            }
        }
    }

    if (boundary_edges.empty())
        return result;

    BOOST_LOG_TRIVIAL(warning) << "[HoleFinder] Seed facet " << seed_facet_idx
        << ", normal=(" << seed_normal.x() << "," << seed_normal.y() << "," << seed_normal.z() << ")"
        << ", region has " << (int)std::count(in_region.begin(), in_region.end(), true)
        << " faces, " << (int)boundary_edges.size() << " boundary edges";

    // Step 4: Chain boundary half-edges into closed loops.
    // Build a map: from_vertex -> list of half-edge indices.
    std::unordered_map<int, std::vector<int>> from_map;
    for (int i = 0; i < (int)boundary_edges.size(); ++i)
        from_map[boundary_edges[i].from].push_back(i);

    std::vector<bool> used(boundary_edges.size(), false);
    std::vector<std::vector<int>> loops; // Each loop is a list of vertex indices.

    for (int start = 0; start < (int)boundary_edges.size(); ++start) {
        if (used[start])
            continue;

        std::vector<int> loop_verts;
        int current = start;
        while (!used[current]) {
            used[current] = true;
            loop_verts.push_back(boundary_edges[current].from);
            int next_from = boundary_edges[current].to;

            // Find the next half-edge starting from next_from.
            int next = -1;
            auto it = from_map.find(next_from);
            if (it != from_map.end()) {
                for (int idx : it->second) {
                    if (!used[idx]) {
                        next = idx;
                        break;
                    }
                }
            }
            if (next < 0)
                break; // Should not happen for a proper closed loop.
            current = next;
        }

        if (loop_verts.size() >= 3)
            loops.push_back(std::move(loop_verts));
    }

    if (loops.empty())
        return result;

    {
        std::string msg = "[HoleFinder] Found " + std::to_string(loops.size()) + " loops:";
        for (int i = 0; i < (int)loops.size(); ++i) {
            // Also compute 3D centroid for spatial identification
            Vec3f c3d = Vec3f::Zero();
            for (int vi : loops[i]) c3d += its.vertices[vi];
            c3d /= (float)loops[i].size();
            msg += " [" + std::to_string(i) + "]=" + std::to_string(loops[i].size()) + "verts";
            msg += "(3d:" + std::to_string(c3d.x()) + "," + std::to_string(c3d.y()) + "," + std::to_string(c3d.z()) + ")";
        }
        BOOST_LOG_TRIVIAL(warning) << msg;
    }

    // Step 5: Classify loops.
    // The outermost loop (largest by bounding box area or signed area) is the
    // face perimeter. All others are holes.
    // We compute a simple "signed area" proxy by projecting onto the dominant
    // plane of the seed normal and measuring 2D signed area.

    // Find the two largest axes of the seed normal for projection.
    int axis0, axis1;
    {
        Vec3f abs_n = seed_normal.cwiseAbs();
        int dominant = 0;
        if (abs_n[1] > abs_n[dominant]) dominant = 1;
        if (abs_n[2] > abs_n[dominant]) dominant = 2;
        // Choose the two non-dominant axes.
        axis0 = (dominant + 1) % 3;
        axis1 = (dominant + 2) % 3;
    }

    // Compute signed area for each loop (projected onto the 2D plane).
    auto signed_area_2d = [&](const std::vector<int> &verts) -> float {
        float area = 0.f;
        int n = (int)verts.size();
        for (int i = 0; i < n; ++i) {
            const Vec3f &p0 = its.vertices[verts[i]];
            const Vec3f &p1 = its.vertices[verts[(i + 1) % n]];
            area += (p0[axis0] * p1[axis1] - p1[axis0] * p0[axis1]);
        }
        return 0.5f * area;
    };

    // Find the loop with the largest absolute area — that's the perimeter.
    int perimeter_idx = 0;
    float max_abs_area = 0.f;
    for (int i = 0; i < (int)loops.size(); ++i) {
        float a = std::abs(signed_area_2d(loops[i]));
        BOOST_LOG_TRIVIAL(warning) << "[HoleFinder] Loop " << i << ": abs_area=" << a << ", verts=" << (int)loops[i].size();
        if (a > max_abs_area) {
            max_abs_area = a;
            perimeter_idx = i;
        }
    }
    BOOST_LOG_TRIVIAL(warning) << "[HoleFinder] Perimeter = loop " << perimeter_idx << " (area " << max_abs_area << ")";

    // Step 6: Classify non-perimeter loops using nesting depth.
    //
    // Instead of relying on winding orientation (which can be unreliable
    // depending on mesh topology), we use geometric containment:
    //   - For each loop, count how many OTHER loops contain its centroid.
    //   - Nesting depth 0 = perimeter (outermost, already identified).
    //   - Nesting depth 1 = holes (contained only by the perimeter).
    //   - Nesting depth 2 = islands inside holes (contained by perimeter + one hole).
    //   - Depth 3+ would be holes-within-islands, etc. (rare, but handled correctly).
    //   - Odd depth = hole (recess to fill), even depth = island (surface to preserve).
    //     (Perimeter at depth 0 is even = not a hole, which is correct.)

    // Helper: 2D point-in-polygon using ray casting (projected coordinates).
    auto point_in_loop_2d = [&](float px, float py, const std::vector<int> &verts) -> bool {
        bool inside = false;
        int nv = (int)verts.size();
        for (int i = 0, j = nv - 1; i < nv; j = i++) {
            float yi = its.vertices[verts[i]][axis1];
            float yj = its.vertices[verts[j]][axis1];
            float xi = its.vertices[verts[i]][axis0];
            float xj = its.vertices[verts[j]][axis0];
            if (((yi > py) != (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
                inside = !inside;
        }
        return inside;
    };

    // Helper: compute centroid of a loop in projected 2D.
    auto loop_centroid_2d = [&](const std::vector<int> &verts) -> std::pair<float, float> {
        float cx = 0.f, cy = 0.f;
        for (int vi : verts) {
            cx += its.vertices[vi][axis0];
            cy += its.vertices[vi][axis1];
        }
        int nv = (int)verts.size();
        return {cx / nv, cy / nv};
    };

    // Compute nesting depth for each loop.
    // The perimeter (largest area) is always depth 0 by definition.
    // For non-perimeter loops, we count how many OTHER non-perimeter loops
    // contain them (via point-in-polygon). The perimeter is excluded from
    // containment tests because complex perimeter shapes (e.g., faces with
    // letter-channel cutouts) create non-convex polygons where point-in-polygon
    // gives false positives.
    //
    // Depth relative to perimeter:
    //   perimeter = 0
    //   direct children of perimeter = 1 (holes)
    //   children of holes = 2 (islands)
    //   etc.
    int num_loops = (int)loops.size();
    std::vector<int> nesting_depth(num_loops, 0);

    // Perimeter is depth 0.
    nesting_depth[perimeter_idx] = 0;

    // All non-perimeter loops start at depth 1 (inside the perimeter).
    // Then count containment by other non-perimeter loops.
    for (int i = 0; i < num_loops; ++i) {
        if (i == perimeter_idx) continue;
        nesting_depth[i] = 1; // Inside the perimeter
        auto [cx, cy] = loop_centroid_2d(loops[i]);
        for (int j = 0; j < num_loops; ++j) {
            if (j == i || j == perimeter_idx) continue;
            if (point_in_loop_2d(cx, cy, loops[j]))
                nesting_depth[i]++;
        }
    }

    {
        std::string msg = "[HoleFinder] Nesting depths:";
        for (int i = 0; i < num_loops; ++i)
            msg += " [" + std::to_string(i) + "]=" + std::to_string(nesting_depth[i]);
        BOOST_LOG_TRIVIAL(warning) << msg;
    }

    // Odd depth = hole, even depth (>0) = island.
    // Build a parent map: for each island (even depth > 0), find which
    // hole (odd depth) at depth-1 contains it.

    // First, collect all hole loops (odd depth).
    struct LoopInfo {
        int loop_idx;
        int depth;
    };
    std::vector<LoopInfo> hole_loops;    // odd depth
    std::vector<LoopInfo> island_loops;  // even depth > 0

    for (int i = 0; i < num_loops; ++i) {
        if (i == perimeter_idx) continue;
        if (nesting_depth[i] % 2 == 1)
            hole_loops.push_back({i, nesting_depth[i]});
        else if (nesting_depth[i] > 0)
            island_loops.push_back({i, nesting_depth[i]});
    }

    BOOST_LOG_TRIVIAL(warning) << "[HoleFinder] Classification: " << hole_loops.size() << " holes, " << island_loops.size() << " islands";
    for (const auto &hl : hole_loops)
        BOOST_LOG_TRIVIAL(warning) << "  Hole: loop " << hl.loop_idx << " (depth " << hl.depth << ", verts " << loops[hl.loop_idx].size() << ")";
    for (const auto &il : island_loops)
        BOOST_LOG_TRIVIAL(warning) << "  Island: loop " << il.loop_idx << " (depth " << il.depth << ", verts " << loops[il.loop_idx].size() << ")";

    // Build HoleBoundary for each hole loop.
    // Map loop_idx -> index in result for parent lookup.
    std::unordered_map<int, int> hole_loop_to_result;

    for (const auto &hl : hole_loops) {
        HoleBoundary hb;
        hb.plane_normal = seed_normal;
        hb.loop.reserve(loops[hl.loop_idx].size());

        Vec3f centroid = Vec3f::Zero();
        for (int vi : loops[hl.loop_idx]) {
            hb.loop.push_back(its.vertices[vi]);
            centroid += its.vertices[vi];
        }
        centroid /= (float)hb.loop.size();
        hb.plane_origin = centroid;

        hole_loop_to_result[hl.loop_idx] = (int)result.size();
        result.push_back(std::move(hb));
    }

    // Assign each island to its parent hole (the odd-depth loop at depth-1
    // that contains it).
    for (const auto &il : island_loops) {
        auto [cx, cy] = loop_centroid_2d(loops[il.loop_idx]);
        int target_depth = il.depth - 1; // The hole that directly contains this island

        for (const auto &hl : hole_loops) {
            if (hl.depth != target_depth) continue;
            if (!point_in_loop_2d(cx, cy, loops[hl.loop_idx])) continue;

            // Found the parent hole.
            auto it = hole_loop_to_result.find(hl.loop_idx);
            if (it != hole_loop_to_result.end()) {
                std::vector<Vec3f> inner;
                inner.reserve(loops[il.loop_idx].size());
                for (int vi : loops[il.loop_idx])
                    inner.push_back(its.vertices[vi]);
                result[it->second].inner_loops.push_back(std::move(inner));
            }
            break; // Each island belongs to exactly one parent hole.
        }
    }

    // ── Step 7: Detect disconnected coplanar islands ─────────────────────
    // Some islands (e.g., the counter inside the letter "A") are separate
    // coplanar face regions NOT connected to the main face region. Their
    // boundary loops won't appear in the main flood-fill. We scan for them
    // by flood-filling other coplanar faces and checking if their projected
    // boundaries fall inside any detected hole.
    if (!result.empty()) {
        // Collect faces already visited (in the main region).
        // in_region already marks the main region.

        // Compute the plane offset of the main face region.
        // d = seed_normal · vertex, for any vertex on the main face.
        // Disconnected islands must lie on the SAME plane (same offset),
        // not just have the same normal direction. This prevents channel
        // floor faces (e.g., bottom of engraved N or E) from being
        // incorrectly added as islands — they have the same normal but
        // are offset inward from the surface.
        float main_plane_d = its.vertices[its.indices[seed_facet_idx][0]].dot(seed_normal);
        const float plane_tolerance = 0.1f; // 0.1mm tolerance for same-plane check

        // We'll track all faces that belong to ANY island region we discover
        // to avoid re-processing.
        std::vector<bool> island_visited(its.indices.size(), false);
        for (int fi = 0; fi < (int)its.indices.size(); ++fi)
            if (in_region[fi]) island_visited[fi] = true;

        int regions_scanned = 0;
        int islands_found = 0;

        for (int fi = 0; fi < (int)its.indices.size(); ++fi) {
            if (island_visited[fi]) continue;

            Vec3f fn = face_normal(its, fi);
            // Must be coplanar with the seed face (same normal direction).
            if (fn.dot(seed_normal) < cos_tolerance) continue;

            // Must be on the same geometric plane (same offset along normal).
            // This filters out channel floor faces that have the same normal
            // but are recessed from the surface.
            float face_d = its.vertices[its.indices[fi][0]].dot(seed_normal);
            if (std::abs(face_d - main_plane_d) > plane_tolerance) {
                island_visited[fi] = true; // Don't revisit
                continue;
            }

            // Flood-fill this separate coplanar region.
            // Only grow to faces on the same plane (same normal AND same offset).
            std::vector<int> island_faces;
            std::queue<int> island_queue;
            island_queue.push(fi);
            island_visited[fi] = true;
            while (!island_queue.empty()) {
                int cur = island_queue.front();
                island_queue.pop();
                island_faces.push_back(cur);
                for (int ni = 0; ni < 3; ++ni) {
                    int nb = neighbors[cur][ni];
                    if (nb < 0 || island_visited[nb]) continue;
                    Vec3f nn = face_normal(its, nb);
                    if (nn.dot(seed_normal) < cos_tolerance) continue;
                    float nb_d = its.vertices[its.indices[nb][0]].dot(seed_normal);
                    if (std::abs(nb_d - main_plane_d) > plane_tolerance) continue;
                    island_visited[nb] = true;
                    island_queue.push(nb);
                }
            }

            regions_scanned++;

            if (island_faces.size() < 1) continue;

            // Build a set for fast lookup.
            std::unordered_set<int> island_set(island_faces.begin(), island_faces.end());

            // Find boundary edges of this island region.
            std::vector<HalfEdge> isl_boundary_edges;
            for (int ifi : island_faces) {
                const Vec3i32 &f = its.indices[ifi];
                for (int ei = 0; ei < 3; ++ei) {
                    int nb = neighbors[ifi][ei];
                    if (nb < 0 || island_set.find(nb) == island_set.end()) {
                        int v_from = f[ei];
                        int v_to   = f[(ei + 1) % 3];
                        isl_boundary_edges.push_back({v_from, v_to, ifi});
                    }
                }
            }

            if (isl_boundary_edges.empty()) continue;

            // Chain into loops (same algorithm as main region).
            std::unordered_map<int, std::vector<int>> isl_from_map;
            for (int i = 0; i < (int)isl_boundary_edges.size(); ++i)
                isl_from_map[isl_boundary_edges[i].from].push_back(i);

            std::vector<bool> isl_used(isl_boundary_edges.size(), false);
            std::vector<std::vector<int>> isl_loops;

            for (int start = 0; start < (int)isl_boundary_edges.size(); ++start) {
                if (isl_used[start]) continue;
                std::vector<int> loop_verts;
                int current = start;
                while (!isl_used[current]) {
                    isl_used[current] = true;
                    loop_verts.push_back(isl_boundary_edges[current].from);
                    int next_from = isl_boundary_edges[current].to;
                    int next = -1;
                    auto it = isl_from_map.find(next_from);
                    if (it != isl_from_map.end()) {
                        for (int idx : it->second) {
                            if (!isl_used[idx]) { next = idx; break; }
                        }
                    }
                    if (next < 0) break;
                    current = next;
                }
                if (loop_verts.size() >= 3)
                    isl_loops.push_back(std::move(loop_verts));
            }

            if (isl_loops.empty()) continue;

            // Use the outermost loop (largest area) as the island's boundary.
            int best_loop = 0;
            float best_area = 0.f;
            for (int li = 0; li < (int)isl_loops.size(); ++li) {
                float a = std::abs(signed_area_2d(isl_loops[li]));
                if (a > best_area) {
                    best_area = a;
                    best_loop = li;
                }
            }

            // Compute 2D centroid of this island's outer loop.
            const auto &isl_verts = isl_loops[best_loop];
            float icx = 0.f, icy = 0.f;
            for (int vi : isl_verts) {
                icx += its.vertices[vi][axis0];
                icy += its.vertices[vi][axis1];
            }
            icx /= (float)isl_verts.size();
            icy /= (float)isl_verts.size();

            // Check if this island falls inside any of our detected holes.
            for (int hi = 0; hi < (int)result.size(); ++hi) {
                // Project the hole loop to 2D for containment test.
                // We need the hole's vertices in terms of axis0/axis1.
                // The hole loop is stored as 3D Vec3f, so we use the same projection.
                bool inside = false;
                {
                    const auto &hloop = result[hi].loop;
                    int hn = (int)hloop.size();
                    // Ray-casting point-in-polygon with Vec3f loop.
                    for (int i = 0, j = hn - 1; i < hn; j = i++) {
                        float yi = hloop[i][axis1];
                        float yj = hloop[j][axis1];
                        float xi = hloop[i][axis0];
                        float xj = hloop[j][axis0];
                        if (((yi > icy) != (yj > icy)) &&
                            (icx < (xj - xi) * (icy - yi) / (yj - yi) + xi))
                            inside = !inside;
                    }
                }

                if (inside) {
                    // This disconnected region is an island inside this hole.
                    std::vector<Vec3f> inner;
                    inner.reserve(isl_verts.size());
                    for (int vi : isl_verts)
                        inner.push_back(its.vertices[vi]);
                    result[hi].inner_loops.push_back(std::move(inner));
                    islands_found++;

                    BOOST_LOG_TRIVIAL(warning) << "[HoleFinder] Found disconnected island ("
                        << island_faces.size() << " faces, " << isl_verts.size()
                        << " verts, plane_d=" << face_d << ") inside hole " << hi;
                    break; // Each island belongs to one hole.
                }
            }
        }

        BOOST_LOG_TRIVIAL(warning) << "[HoleFinder] Step 7: scanned "
            << regions_scanned << " coplanar regions, found "
            << islands_found << " disconnected islands"
            << " (main_plane_d=" << main_plane_d << ")";
    }

    return result;
}

bool find_nearest_hole(
    const indexed_triangle_set &its,
    int                         seed_facet_idx,
    const Vec3f                &hit_point,
    HoleBoundary               &out,
    float                       angle_tolerance_deg)
{
    auto holes = find_hole_boundaries(its, seed_facet_idx, angle_tolerance_deg);
    if (holes.empty())
        return false;

    // Find hole whose centroid is closest to the hit point.
    float best_dist_sq = std::numeric_limits<float>::max();
    int   best_idx     = -1;
    for (int i = 0; i < (int)holes.size(); ++i) {
        float d = (holes[i].plane_origin - hit_point).squaredNorm();
        if (d < best_dist_sq) {
            best_dist_sq = d;
            best_idx = i;
        }
    }

    if (best_idx >= 0) {
        out = std::move(holes[best_idx]);
        return true;
    }
    return false;
}

} // namespace Slic3r
