#include <coverage/face_viewpoint_planner.hpp>

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace coverage {
namespace {

void buildFaceFrame(const Eigen::Vector3d &normal_in,
                    Eigen::Vector3d &n,
                    Eigen::Vector3d &u,
                    Eigen::Vector3d &v) {
    n = normal_in;
    if (n.norm() < 1e-6) {
        n = -Eigen::Vector3d::UnitX();
    }
    n.normalize();
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    if (std::abs(n.dot(up)) > 0.95) {
        up = Eigen::Vector3d::UnitY();
    }
    u = (up.cross(n)).normalized();
    v = n.cross(u);
}

std::vector<Eigen::Vector3d> sampleSurfacePoints(const mission::FaceObservation &face,
                                                 const Eigen::Vector3d &u,
                                                 const Eigen::Vector3d &v,
                                                 const double resolution) {
    std::vector<Eigen::Vector3d> points;
    if (face.surface_cloud && !face.surface_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(face.surface_cloud);
        const float leaf = static_cast<float>(std::max(0.1, resolution));
        voxel_filter.setLeafSize(leaf, leaf, leaf);
        pcl::PointCloud<pcl::PointXYZ> downsampled;
        voxel_filter.filter(downsampled);
        points.reserve(downsampled.size());
        for (const auto &pt : downsampled.points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                points.emplace_back(pt.x, pt.y, pt.z);
            }
        }
        return points;
    }

    const double half_w = 0.5 * std::max(0.5, face.width);
    const double half_h = 0.5 * std::max(0.5, face.height);
    const double step = std::max(0.2, resolution);
    for (double ou = -half_w; ou <= half_w + 1e-6; ou += step) {
        for (double ov = -half_h; ov <= half_h + 1e-6; ov += step) {
            points.push_back(face.center + ou * u + ov * v);
        }
    }
    return points;
}

rog_map::Vec3f toMapVec(const Eigen::Vector3d &p) {
    return rog_map::Vec3f(p.x(), p.y(), p.z());
}

bool isKnownFreeSafe(const general_planner::MapManager &map,
                     const Eigen::Vector3d &p,
                     const double safe_radius,
                     const double z_min,
                     const double z_max) {
    if (p.z() < z_min || p.z() > z_max) {
        return false;
    }
    const rog_map::Vec3f pf = toMapVec(p);
    if (!map.ready() || !map.insideLocalMap(pf)) {
        return false;
    }
    const auto gt = map.getInfGridType(pf);
    if (gt == rog_map::GridType::OCCUPIED || gt == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }
    if (map.hasESDF()) {
        double dist = 0.0;
        rog_map::Vec3f grad = rog_map::Vec3f::Zero();
        if (!map.evaluateESDF(pf, dist, grad) || !std::isfinite(dist) ||
            dist < safe_radius) {
            return false;
        }
    } else if (map.getGridType(pf) == rog_map::GridType::OCCUPIED) {
        return false;
    }
    return true;
}

bool rayFreeToFaceSurface(const general_planner::MapManager &map,
                          const Eigen::Vector3d &camera,
                          const Eigen::Vector3d &surface,
                          const Eigen::Vector3d &face_normal,
                          const bool unknown_as_occupied) {
    if (!map.ready()) {
        return false;
    }
    // The detected surface is intentionally an occupied plane.  Checking a
    // camera ray against the inflated map all the way to that plane always
    // classifies the terminal hit as an occluder, and leaves no usable views.
    // Flight-position safety remains checked on the inflated map above; for
    // optical visibility use the raw occupancy map and stop just in front of
    // the surface, on its observed free-space (normal) side.
    Eigen::Vector3d n = face_normal;
    if (n.norm() < 1e-6) {
        return false;
    }
    n.normalize();
    const double terminal_margin = std::max(0.05, 0.75 * map.getResolution());
    const Eigen::Vector3d free_side_terminal = surface + terminal_margin * n;
    return map.isLineFree(toMapVec(camera),
                          toMapVec(free_side_terminal),
                          false,
                          unknown_as_occupied);
}

double yawFromDirection(const Eigen::Vector3d &dir) {
    return std::atan2(dir.y(), dir.x());
}

double pitchToPoint(const Eigen::Vector3d &from, const Eigen::Vector3d &to) {
    const Eigen::Vector3d d = to - from;
    const double horiz = std::hypot(d.x(), d.y());
    return std::atan2(-d.z(), std::max(1e-3, horiz));
}

bool inFov(const Eigen::Vector3d &view_pos,
           double body_yaw,
           double camera_pitch,
           const Eigen::Vector3d &target,
           const CameraModel &camera) {
    const Eigen::Vector3d d = target - view_pos;
    const double range = d.norm();
    if (range < 0.3 || range > camera.capture_distance * 1.8) {
        return false;
    }
    const Eigen::AngleAxisd R_yaw(body_yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd R_pitch(camera_pitch, Eigen::Vector3d::UnitY());
    const Eigen::Vector3d look = R_yaw * R_pitch * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d dir = d.normalized();
    const double cos_angle = std::clamp(look.dot(dir), -1.0, 1.0);
    const double angle = std::acos(cos_angle);
    const double hfov = camera.hfov_deg * M_PI / 180.0;
    const double vfov = camera.vfov_deg * M_PI / 180.0;
    return angle <= 0.5 * std::hypot(hfov, vfov);
}

bool incidenceOk(const Eigen::Vector3d &view_pos,
                 const Eigen::Vector3d &target,
                 const Eigen::Vector3d &normal,
                 const double max_incidence_deg) {
    Eigen::Vector3d dir = (view_pos - target);
    if (dir.norm() < 1e-6) {
        return false;
    }
    dir.normalize();
    const double cos_inc = std::clamp(dir.dot(normal.normalized()), -1.0, 1.0);
    const double incidence = std::acos(cos_inc) * 180.0 / M_PI;
    return incidence <= max_incidence_deg;
}

std::vector<int> orderByNn2Opt(const std::vector<mission::CaptureViewpoint> &views,
                               const Eigen::Vector3d &start,
                               const Eigen::Vector3d &home) {
    const int n = static_cast<int>(views.size());
    std::vector<int> order;
    if (n == 0) {
        return order;
    }
    std::vector<bool> used(n, false);
    Eigen::Vector3d cur = start;
    for (int k = 0; k < n; ++k) {
        int best = -1;
        double best_d = std::numeric_limits<double>::infinity();
        for (int i = 0; i < n; ++i) {
            if (used[i]) {
                continue;
            }
            const double d = (views[static_cast<std::size_t>(i)].position - cur).norm();
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        used[static_cast<std::size_t>(best)] = true;
        order.push_back(best);
        cur = views[static_cast<std::size_t>(best)].position;
    }

    auto pathCost = [&](const std::vector<int> &ord) {
        double cost = 0.0;
        Eigen::Vector3d p = start;
        for (const int idx : ord) {
            cost += (views[static_cast<std::size_t>(idx)].position - p).norm();
            p = views[static_cast<std::size_t>(idx)].position;
        }
        cost += (home - p).norm();
        return cost;
    };

    bool improved = true;
    while (improved && n >= 4) {
        improved = false;
        for (int i = 0; i < n - 1; ++i) {
            for (int k = i + 1; k < n; ++k) {
                std::vector<int> candidate = order;
                std::reverse(candidate.begin() + i, candidate.begin() + k + 1);
                if (pathCost(candidate) + 1e-6 < pathCost(order)) {
                    order.swap(candidate);
                    improved = true;
                }
            }
        }
    }
    return order;
}

}  // namespace

FaceViewpointPlanner::FaceViewpointPlanner(Config config) : cfg_(std::move(config)) {}

void FaceViewpointPlanner::setConfig(const Config &config) {
    cfg_ = config;
}

mission::CoveragePlan FaceViewpointPlanner::plan(
        const mission::FaceObservation &face,
        const general_planner::MapManager &map,
        const Eigen::Vector3d &current_position,
        const Eigen::Vector3d &home) const {
    mission::CoveragePlan plan;
    if (!face.valid) {
        plan.detail = "invalid_face";
        return plan;
    }

    Eigen::Vector3d n, u, v;
    buildFaceFrame(face.normal, n, u, v);
    const auto surface = sampleSurfacePoints(face, u, v, cfg_.surface_sample_resolution);
    if (surface.empty()) {
        plan.detail = "empty_surface";
        return plan;
    }

    const double d = std::max(0.5, cfg_.camera.capture_distance);
    const double hfov = cfg_.camera.hfov_deg * M_PI / 180.0;
    const double vfov = cfg_.camera.vfov_deg * M_PI / 180.0;
    const double footprint_w = 2.0 * d * std::tan(0.5 * hfov);
    const double footprint_h = 2.0 * d * std::tan(0.5 * vfov);
    const double overlap = std::clamp(cfg_.camera.image_overlap, 0.0, 0.95);
    const double step_u = std::max(0.3, footprint_w * (1.0 - overlap));
    const double step_v = std::max(0.3, footprint_h * (1.0 - overlap));
    const double half_w = 0.5 * std::max(face.width, footprint_w);
    const double half_h = 0.5 * std::max(face.height, footprint_h);

    std::vector<mission::CaptureViewpoint> candidates;
    uint32_t next_id = 1;
    // A single lattice leaves edge samples with only one nearly frontal
    // observation.  Add a half-cell staggered lattice so the second image has
    // a meaningful photogrammetric baseline while retaining the requested
    // overlap.  Each candidate is still validated against the occupancy map.
    for (int lattice = 0; lattice < 2; ++lattice) {
        const double offset_u = lattice == 0 ? 0.0 : 0.5 * step_u;
        const double offset_v = lattice == 0 ? 0.0 : 0.5 * step_v;
        for (double ou = -half_w + offset_u; ou <= half_w + 1e-6; ou += step_u) {
            for (double ov = -half_h + offset_v; ov <= half_h + 1e-6; ov += step_v) {
            const Eigen::Vector3d aim = face.center + ou * u + ov * v;
            const Eigen::Vector3d pos = aim + d * n;
            if (!isKnownFreeSafe(map,
                                 pos,
                                 cfg_.safe_radius,
                                 cfg_.flight_height_min,
                                 cfg_.flight_height_max)) {
                continue;
            }
            const double yaw = yawFromDirection((aim - pos).normalized());
            const double pitch = pitchToPoint(pos, aim);
            const double pitch_deg = pitch * 180.0 / M_PI;
            if (pitch_deg < cfg_.camera.min_pitch_deg ||
                pitch_deg > cfg_.camera.max_pitch_deg) {
                continue;
            }

            mission::CaptureViewpoint vp;
            vp.id = next_id++;
            vp.position = pos;
            vp.body_yaw = yaw;
            vp.camera_pitch = pitch;
            for (int si = 0; si < static_cast<int>(surface.size()); ++si) {
                const auto &sp = surface[static_cast<std::size_t>(si)];
                if (!inFov(pos, yaw, pitch, sp, cfg_.camera)) {
                    continue;
                }
                if (!incidenceOk(pos, sp, n, cfg_.camera.max_incidence_angle_deg)) {
                    continue;
                }
                if (!rayFreeToFaceSurface(map, pos, sp, n,
                                          cfg_.unknown_as_occupied)) {
                    continue;
                }
                vp.visible_surface_ids.push_back(si);
            }
            if (vp.visible_surface_ids.empty()) {
                continue;
            }
            vp.expected_coverage_gain =
                    static_cast<double>(vp.visible_surface_ids.size()) /
                    static_cast<double>(surface.size());
                candidates.push_back(std::move(vp));
            }
        }
    }

    if (candidates.empty()) {
        // Fallback: single approach-like viewpoint on the face normal.
        mission::CaptureViewpoint vp;
        vp.id = 1;
        vp.position = face.center + d * n;
        vp.body_yaw = yawFromDirection(-n);
        vp.camera_pitch = 0.0;
        for (int si = 0; si < static_cast<int>(surface.size()); ++si) {
            vp.visible_surface_ids.push_back(si);
        }
        if (isKnownFreeSafe(map,
                            vp.position,
                            cfg_.safe_radius,
                            cfg_.flight_height_min,
                            cfg_.flight_height_max)) {
            candidates.push_back(vp);
        } else {
            plan.detail = "no_feasible_viewpoint";
            return plan;
        }
    }

    const int K = std::max(1, cfg_.min_observation_count);
    const int S = static_cast<int>(surface.size());
    std::vector<int> cover_count(static_cast<std::size_t>(S), 0);
    // A count only advances when the new camera center creates the configured
    // angular baseline to an earlier observation of the same surface point.
    // This prevents two nearly coincident frames from satisfying K-coverage.
    std::vector<std::vector<Eigen::Vector3d>> observation_dirs(
            static_cast<std::size_t>(S));
    std::vector<bool> selected(candidates.size(), false);
    std::vector<mission::CaptureViewpoint> chosen;
    const double min_baseline = cfg_.min_baseline_angle_deg * M_PI / 180.0;

    auto uncoveredLeft = [&]() {
        int left = 0;
        for (int c : cover_count) {
            if (c < K) {
                ++left;
            }
        }
        return left;
    };

    while (uncoveredLeft() > 0) {
        int best_i = -1;
        int best_gain = 0;
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
            if (selected[static_cast<std::size_t>(i)]) {
                continue;
            }
            const auto &cand = candidates[static_cast<std::size_t>(i)];
            int gain = 0;
            for (const int sid : cand.visible_surface_ids) {
                if (sid < 0 || sid >= S ||
                    cover_count[static_cast<std::size_t>(sid)] >= K) {
                    continue;
                }
                const Eigen::Vector3d view_dir =
                        (cand.position - surface[static_cast<std::size_t>(sid)]).normalized();
                bool adds_distinct_observation = observation_dirs[static_cast<std::size_t>(sid)].empty();
                for (const auto &previous_dir : observation_dirs[static_cast<std::size_t>(sid)]) {
                    const double angle = std::acos(std::clamp(view_dir.dot(previous_dir), -1.0, 1.0));
                    if (angle >= min_baseline) {
                        adds_distinct_observation = true;
                        break;
                    }
                }
                if (adds_distinct_observation) {
                    ++gain;
                }
            }
            if (gain > best_gain) {
                best_gain = gain;
                best_i = i;
            }
        }
        if (best_i < 0 || best_gain == 0) {
            break;
        }
        selected[static_cast<std::size_t>(best_i)] = true;
        auto vp = candidates[static_cast<std::size_t>(best_i)];
        vp.expected_coverage_gain =
                static_cast<double>(best_gain) / static_cast<double>(S);
        for (const int sid : vp.visible_surface_ids) {
            if (sid < 0 || sid >= S ||
                cover_count[static_cast<std::size_t>(sid)] >= K) {
                continue;
            }
            const Eigen::Vector3d view_dir =
                    (vp.position - surface[static_cast<std::size_t>(sid)]).normalized();
            bool adds_distinct_observation = observation_dirs[static_cast<std::size_t>(sid)].empty();
            for (const auto &previous_dir : observation_dirs[static_cast<std::size_t>(sid)]) {
                const double angle = std::acos(std::clamp(view_dir.dot(previous_dir), -1.0, 1.0));
                if (angle >= min_baseline) {
                    adds_distinct_observation = true;
                    break;
                }
            }
            if (adds_distinct_observation) {
                observation_dirs[static_cast<std::size_t>(sid)].push_back(view_dir);
                cover_count[static_cast<std::size_t>(sid)] += 1;
            }
        }
        chosen.push_back(std::move(vp));
        if (static_cast<int>(chosen.size()) >=
            std::max(std::max(K, 1), cfg_.max_viewpoints)) {
            break;
        }
    }

    if (chosen.empty()) {
        plan.detail = "k_coverage_failed";
        return plan;
    }

    const auto order = orderByNn2Opt(chosen, current_position, home);
    plan.ordered_viewpoints.reserve(order.size());
    for (const int idx : order) {
        plan.ordered_viewpoints.push_back(chosen[static_cast<std::size_t>(idx)]);
    }

    int covered_k = 0;
    for (int c : cover_count) {
        if (c >= K) {
            ++covered_k;
        }
    }
    plan.predicted_coverage =
            static_cast<double>(covered_k) / static_cast<double>(std::max(1, S));
    plan.valid = !plan.ordered_viewpoints.empty();
    plan.detail = plan.valid
                          ? "ok;candidates=" + std::to_string(candidates.size()) +
                                    ";views=" + std::to_string(chosen.size()) +
                                    ";samples=" + std::to_string(surface.size())
                          : "empty";
    return plan;
}

}  // namespace coverage
