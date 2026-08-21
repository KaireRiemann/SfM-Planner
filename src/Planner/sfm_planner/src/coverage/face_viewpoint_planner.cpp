#include <coverage/face_viewpoint_planner.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace coverage {
namespace {

bool buildFaceFrame(const mission::FaceObservation &face,
                    Eigen::Vector3d &n,
                    Eigen::Vector3d &u,
                    Eigen::Vector3d &v) {
    n = face.normal;
    if (n.norm() < 1e-6) {
        return false;
    }
    n.normalize();

    // Use exactly the detector's face frame when available.  Legacy/external
    // observations without tangents retain the deterministic world-up frame.
    u = face.tangent_u - face.tangent_u.dot(n) * n;
    if (u.norm() > 1e-6) {
        u.normalize();
        v = n.cross(u).normalized();
        if (face.tangent_v.norm() > 1e-6 && face.tangent_v.dot(v) < 0.0) {
            u = -u;
            v = -v;
        }
        return true;
    }
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    if (std::abs(n.dot(up)) > 0.95) {
        up = Eigen::Vector3d::UnitY();
    }
    u = (up.cross(n)).normalized();
    v = n.cross(u).normalized();
    return true;
}

std::vector<Eigen::Vector3d> sampleSurfacePoints(const mission::FaceObservation &face,
                                                 const Eigen::Vector3d &u,
                                                 const Eigen::Vector3d &v,
                                                 const double resolution) {
    // Coverage is evaluated on the entire detected rectangle, including its
    // four edges.  Do not use the sparse support cloud as the denominator:
    // that was the source of the former "97% of a small patch" result.
    std::vector<Eigen::Vector3d> points;
    const double width = std::max(0.5, face.width);
    const double height = std::max(0.5, face.height);
    const double step = std::max(0.2, resolution);
    const int count_u = std::max(1, static_cast<int>(std::ceil(width / step)));
    const int count_v = std::max(1, static_cast<int>(std::ceil(height / step)));
    points.reserve(static_cast<std::size_t>((count_u + 1) * (count_v + 1)));
    for (int iu = 0; iu <= count_u; ++iu) {
        const double ou = width * (static_cast<double>(iu) / count_u - 0.5);
        for (int iv = 0; iv <= count_v; ++iv) {
            const double ov = height * (static_cast<double>(iv) / count_v - 0.5);
            points.push_back(face.center + ou * u + ov * v);
        }
    }
    return points;
}

std::vector<double> makeLatticeAxis(const double half_extent,
                                    const double step,
                                    const double offset) {
    std::vector<double> offsets{-half_extent, half_extent};
    const double safe_step = std::max(0.05, step);
    for (double value = -half_extent + offset;
         value < half_extent - 1e-6;
         value += safe_step) {
        if (value > -half_extent + 1e-6) {
            offsets.push_back(value);
        }
    }
    std::sort(offsets.begin(), offsets.end());
    offsets.erase(std::unique(offsets.begin(), offsets.end(),
                              [](double a, double b) { return std::abs(a - b) < 1e-6; }),
                  offsets.end());
    return offsets;
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
    // The detected plane support has up to 0.45 m depth tolerance and the
    // raw lidar map represents that rough end-face as several occupied
    // voxels.  Stop 1.5 m (ten 15 cm voxels in the live map) in front of
    // the nominal plane so this target surface band is not interpreted as an
    // unrelated occluder.  The camera position itself remains checked in the
    // inflated map, and the rest of every optical ray remains raw-map free.
    const double terminal_margin = std::max(0.10, 10.0 * map.getResolution());
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
    const Eigen::Matrix3d R = R_yaw.toRotationMatrix() * R_pitch.toRotationMatrix();
    const Eigen::Vector3d look = R * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d camera_right = R * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d camera_up = R * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d dir = d.normalized();
    const double forward = look.dot(dir);
    if (forward <= 1e-6) {
        return false;
    }
    const double hfov = camera.hfov_deg * M_PI / 180.0;
    const double vfov = camera.vfov_deg * M_PI / 180.0;
    const double horizontal = std::atan2(std::abs(camera_right.dot(dir)), forward);
    const double vertical = std::atan2(std::abs(camera_up.dot(dir)), forward);
    return horizontal <= 0.5 * hfov + 1e-6 && vertical <= 0.5 * vfov + 1e-6;
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

struct TransitionBridgeResult {
    std::vector<mission::CaptureViewpoint> viewpoints;
    int candidate_bridges{0};
    int synthetic_bridges{0};
    int unresolved_transitions{0};
    double longest_transition{0.0};
};

TransitionBridgeResult bridgeLongCaptureTransitions(
        const std::vector<mission::CaptureViewpoint> &ordered,
        const std::vector<mission::CaptureViewpoint> &candidates,
        const mission::FaceObservation &face,
        const general_planner::MapManager &map,
        const FaceViewpointPlanner::Config &cfg) {
    TransitionBridgeResult result;
    if (ordered.empty()) {
        return result;
    }

    result.viewpoints.reserve(ordered.size());
    result.viewpoints.push_back(ordered.front());
    const double preferred_distance = cfg.preferred_capture_transition_distance;
    const int bridge_limit = std::max(0, cfg.max_transition_bridge_viewpoints);
    if (preferred_distance <= 1e-3 || bridge_limit <= 0 || ordered.size() < 2) {
        for (std::size_t i = 1; i < ordered.size(); ++i) {
            result.viewpoints.push_back(ordered[i]);
        }
        return result;
    }

    std::vector<bool> candidate_used(candidates.size(), false);
    uint32_t next_synthetic_id = 1;
    for (const auto &candidate : candidates) {
        next_synthetic_id = std::max(next_synthetic_id, candidate.id + 1U);
    }
    for (const auto &view : ordered) {
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (candidates[i].id == view.id) {
                candidate_used[i] = true;
                break;
            }
        }
    }

    auto bestBridgeCandidate = [&](const mission::CaptureViewpoint &from,
                                   const mission::CaptureViewpoint &to) {
        const double direct_distance = (to.position - from.position).norm();
        int best_index = -1;
        double best_score = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (candidate_used[i] || !candidates[i].position.allFinite()) {
                continue;
            }
            const double from_distance = (candidates[i].position - from.position).norm();
            const double to_distance = (to.position - candidates[i].position).norm();
            if (from_distance < 0.15 || from_distance > preferred_distance + 1e-6 ||
                to_distance >= direct_distance - 0.05) {
                continue;
            }
            // Prefer a balanced split first; a short first leg keeps the
            // executed sequence locally continuous even when several bridges
            // are needed for the remaining distance.
            const double score = std::max(from_distance, to_distance) +
                                 0.01 * (from_distance + to_distance);
            if (score < best_score) {
                best_score = score;
                best_index = static_cast<int>(i);
            }
        }
        return best_index;
    };

    Eigen::Vector3d face_normal = face.normal;
    if (face_normal.norm() > 1e-6) {
        face_normal.normalize();
    }
    for (std::size_t target_index = 1; target_index < ordered.size(); ++target_index) {
        const auto &target = ordered[target_index];
        bool unresolved = false;
        while ((target.position - result.viewpoints.back().position).norm() >
               preferred_distance + 1e-6) {
            const int bridges_used = result.candidate_bridges + result.synthetic_bridges;
            if (bridges_used >= bridge_limit) {
                unresolved = true;
                break;
            }

            const int candidate_index = bestBridgeCandidate(result.viewpoints.back(), target);
            if (candidate_index >= 0) {
                candidate_used[static_cast<std::size_t>(candidate_index)] = true;
                result.viewpoints.push_back(
                        candidates[static_cast<std::size_t>(candidate_index)]);
                ++result.candidate_bridges;
                continue;
            }

            const auto &from = result.viewpoints.back();
            const Eigen::Vector3d delta = target.position - from.position;
            const double direct_distance = delta.norm();
            if (!delta.allFinite() || direct_distance <= preferred_distance + 1e-6 ||
                face_normal.norm() < 1e-6) {
                unresolved = true;
                break;
            }
            const Eigen::Vector3d bridge_position =
                    from.position + (preferred_distance / direct_distance) * delta;
            if (!isKnownFreeSafe(map, bridge_position, cfg.safe_radius,
                                 cfg.viewpoint_height_min, cfg.viewpoint_height_max)) {
                unresolved = true;
                break;
            }

            mission::CaptureViewpoint bridge;
            bridge.id = next_synthetic_id++;
            bridge.position = bridge_position;
            bridge.body_yaw = yawFromDirection(face.center - bridge.position);
            bridge.camera_pitch = pitchToPoint(bridge.position, face.center);
            const double pitch_deg = bridge.camera_pitch * 180.0 / M_PI;
            if (pitch_deg < cfg.camera.min_pitch_deg ||
                pitch_deg > cfg.camera.max_pitch_deg ||
                !inFov(bridge.position, bridge.body_yaw, bridge.camera_pitch,
                       face.center, cfg.camera) ||
                !incidenceOk(bridge.position, face.center, face_normal,
                             cfg.camera.max_incidence_angle_deg)) {
                unresolved = true;
                break;
            }
            result.viewpoints.push_back(std::move(bridge));
            ++result.synthetic_bridges;
        }
        if (unresolved) {
            ++result.unresolved_transitions;
        }
        result.viewpoints.push_back(target);
    }

    for (std::size_t i = 1; i < result.viewpoints.size(); ++i) {
        result.longest_transition = std::max(
                result.longest_transition,
                (result.viewpoints[i].position - result.viewpoints[i - 1].position).norm());
    }
    return result;
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
    if (!face.valid || !face.extent_complete) {
        plan.detail = face.extent_detail.empty() ? "incomplete_face_extent" : face.extent_detail;
        return plan;
    }
    if (face.width <= 0.0 || face.height <= 0.0) {
        plan.detail = "invalid_face";
        return plan;
    }

    Eigen::Vector3d n, u, v;
    if (!buildFaceFrame(face, n, u, v)) {
        plan.detail = "invalid_face_frame";
        return plan;
    }
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
    const int S = static_cast<int>(surface.size());

    std::vector<mission::CaptureViewpoint> candidates;
    std::vector<bool> fov_reachable(static_cast<std::size_t>(S), false);
    std::vector<bool> incidence_reachable(static_cast<std::size_t>(S), false);
    uint32_t next_id = 1;
    // A single normal stand-off can be hidden by tunnel ribs or a locally
    // rough face.  Bracket it with near/far observations; every resulting
    // camera pose remains subject to the same map safety and visibility
    // checks below.
    const std::array<double, 2> standoff_distances{
            std::max(0.5, 0.75 * d), d};
    // A single lattice leaves edge samples with only one nearly frontal
    // observation.  Add a half-cell staggered lattice so the second image has
    // a meaningful photogrammetric baseline while retaining the requested
    // overlap.  Each candidate is still validated against the occupancy map.
    for (int lattice = 0; lattice < 2; ++lattice) {
        const double offset_u = lattice == 0 ? 0.0 : 0.5 * step_u;
        const double offset_v = lattice == 0 ? 0.0 : 0.5 * step_v;
        const auto u_offsets = makeLatticeAxis(half_w, step_u, offset_u);
        const auto v_offsets = makeLatticeAxis(half_h, step_v, offset_v);
        for (const double ou : u_offsets) {
            if (cfg_.viewpoint_lateral_limit > 0.0 &&
                std::abs(ou) > cfg_.viewpoint_lateral_limit) {
                continue;
            }
            for (const double ov : v_offsets) {
                const Eigen::Vector3d aim = face.center + ou * u + ov * v;
                for (const double standoff : standoff_distances) {
                // A tall end face commonly extends above the vehicle's
                // permitted flight band.  Keep the camera in that band and
                // let its pitch aim at the corresponding surface cell; do
                // not discard the cell merely because a normal-only camera
                // pose would share its altitude.
                Eigen::Vector3d pos = aim + standoff * n;
                pos.z() = std::clamp(pos.z(), cfg_.viewpoint_height_min,
                                     cfg_.viewpoint_height_max);
                if (!isKnownFreeSafe(map,
                                     pos,
                                     cfg_.safe_radius,
                                     cfg_.viewpoint_height_min,
                                     cfg_.viewpoint_height_max)) {
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
                    fov_reachable[static_cast<std::size_t>(si)] = true;
                    if (!incidenceOk(pos, sp, n, cfg_.camera.max_incidence_angle_deg)) {
                        continue;
                    }
                    incidence_reachable[static_cast<std::size_t>(si)] = true;
                    if (cfg_.use_raw_occlusion_check &&
                        !rayFreeToFaceSurface(map, pos, sp, n,
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
    }

    if (candidates.empty()) {
        plan.detail = "no_feasible_viewpoint";
        return plan;
    }

    const int K = std::max(1, cfg_.min_observation_count);
    const double min_baseline = cfg_.min_baseline_angle_deg * M_PI / 180.0;
    // Keep availability diagnostics separate from the greedy result.  This
    // makes it clear whether an incomplete plan is caused by the available
    // camera geometry or by the set-cover selection itself.
    std::vector<std::vector<Eigen::Vector3d>> available_view_dirs(
            static_cast<std::size_t>(S));
    for (const auto &candidate : candidates) {
        for (const int sid : candidate.visible_surface_ids) {
            if (sid < 0 || sid >= S) {
                continue;
            }
            available_view_dirs[static_cast<std::size_t>(sid)].push_back(
                    (candidate.position - surface[static_cast<std::size_t>(sid)]).normalized());
        }
    }
    int visible_once = 0;
    int visible_with_baseline = 0;
    for (const auto &dirs : available_view_dirs) {
        if (dirs.empty()) {
            continue;
        }
        ++visible_once;
        bool has_baseline_pair = false;
        for (std::size_t i = 0; i < dirs.size() && !has_baseline_pair; ++i) {
            for (std::size_t j = i + 1; j < dirs.size(); ++j) {
                const double angle = std::acos(std::clamp(dirs[i].dot(dirs[j]), -1.0, 1.0));
                if (angle >= min_baseline) {
                    has_baseline_pair = true;
                    break;
                }
            }
        }
        if (has_baseline_pair) {
            ++visible_with_baseline;
        }
    }
    const int fov_visible = static_cast<int>(std::count(fov_reachable.begin(),
                                                         fov_reachable.end(), true));
    const int incidence_visible = static_cast<int>(std::count(incidence_reachable.begin(),
                                                               incidence_reachable.end(), true));
    std::vector<int> cover_count(static_cast<std::size_t>(S), 0);
    std::vector<int> selected_observation_count(static_cast<std::size_t>(S), 0);
    // A count only advances when the new camera center creates the configured
    // angular baseline to an earlier observation of the same surface point.
    // This prevents two nearly coincident frames from satisfying K-coverage.
    std::vector<std::vector<Eigen::Vector3d>> observation_dirs(
            static_cast<std::size_t>(S));
    std::vector<bool> selected(candidates.size(), false);
    std::vector<mission::CaptureViewpoint> chosen;
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
            if (sid < 0 || sid >= S) {
                continue;
            }
            ++selected_observation_count[static_cast<std::size_t>(sid)];
            if (cover_count[static_cast<std::size_t>(sid)] >= K) {
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
    std::vector<mission::CaptureViewpoint> coverage_order;
    coverage_order.reserve(order.size());
    for (const int idx : order) {
        coverage_order.push_back(chosen[static_cast<std::size_t>(idx)]);
    }
    const auto bridge_result = bridgeLongCaptureTransitions(
            coverage_order, candidates, face, map, cfg_);
    plan.ordered_viewpoints = bridge_result.viewpoints;

    int covered_k = 0;
    int covered_once = 0;
    for (int c : cover_count) {
        if (c >= K) {
            ++covered_k;
        }
    }
    for (int c : selected_observation_count) {
        if (c > 0) {
            ++covered_once;
        }
    }
    const double dual_coverage =
            static_cast<double>(covered_k) / static_cast<double>(std::max(1, S));
    const double primary_coverage =
            static_cast<double>(covered_once) / static_cast<double>(std::max(1, S));
    const bool use_boundary_single_view =
            cfg_.allow_single_view_boundary_coverage && covered_once == S;
    plan.predicted_coverage = use_boundary_single_view ? primary_coverage : dual_coverage;
    const double required_coverage =
            std::clamp(cfg_.required_coverage_ratio, 0.0, 1.0);
    plan.valid = !plan.ordered_viewpoints.empty() &&
                 plan.predicted_coverage + 1e-9 >= required_coverage;
    plan.detail = (plan.valid ? "ok" : "coverage_incomplete") +
                  std::string(";candidates=") + std::to_string(candidates.size()) +
                  ";views=" + std::to_string(plan.ordered_viewpoints.size()) +
                  ";coverage_views=" + std::to_string(chosen.size()) +
                  ";candidate_bridges=" + std::to_string(bridge_result.candidate_bridges) +
                  ";synthetic_bridges=" + std::to_string(bridge_result.synthetic_bridges) +
                  ";unresolved_transitions=" +
                  std::to_string(bridge_result.unresolved_transitions) +
                  ";longest_transition=" + std::to_string(bridge_result.longest_transition) +
                  ";samples=" + std::to_string(surface.size()) +
                  ";covered=" + std::to_string(covered_k) +
                  ";single_covered=" + std::to_string(covered_once) +
                  ";dual_coverage=" + std::to_string(dual_coverage) +
                  ";boundary_single_view=" +
                  std::to_string(use_boundary_single_view ? 1 : 0) +
                  ";fov_visible=" + std::to_string(fov_visible) +
                  ";incidence_visible=" + std::to_string(incidence_visible) +
                  ";visible_once=" + std::to_string(visible_once) +
                  ";baseline_available=" + std::to_string(visible_with_baseline);
    return plan;
}

}  // namespace coverage
