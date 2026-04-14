#include "HoleFinder.hpp"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

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
        if (a > max_abs_area) {
            max_abs_area = a;
            perimeter_idx = i;
        }
    }

    // Step 6: Build HoleBoundary for each non-perimeter loop.
    for (int i = 0; i < (int)loops.size(); ++i) {
        if (i == perimeter_idx)
            continue;

        HoleBoundary hb;
        hb.plane_normal = seed_normal;
        hb.loop.reserve(loops[i].size());

        Vec3f centroid = Vec3f::Zero();
        for (int vi : loops[i]) {
            hb.loop.push_back(its.vertices[vi]);
            centroid += its.vertices[vi];
        }
        centroid /= (float)hb.loop.size();
        hb.plane_origin = centroid;

        result.push_back(std::move(hb));
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
