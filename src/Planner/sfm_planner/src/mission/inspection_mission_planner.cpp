#include <mission/inspection_mission_planner.hpp>

#include <fmt/format.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <utility>

namespace mission {
namespace {

uint64_t nowMs() {
    using clock = std::chrono::system_clock;
    return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    clock::now().time_since_epoch())
                    .count());
}

double steadyNowSec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

}  // namespace

InspectionMissionPlanner::InspectionMissionPlanner(
        InspectionMissionConfig config,
        std::shared_ptr<MissionTargetStore> store)
    : cfg_(std::move(config)), store_(std::move(store)) {
    coverage::FaceViewpointPlanner::Config vp_cfg;
    vp_cfg.camera.hfov_deg = cfg_.camera_hfov_deg;
    vp_cfg.camera.vfov_deg = cfg_.camera_vfov_deg;
    vp_cfg.camera.min_pitch_deg = cfg_.camera_min_pitch_deg;
    vp_cfg.camera.max_pitch_deg = cfg_.camera_max_pitch_deg;
    vp_cfg.camera.capture_distance = cfg_.capture_distance;
    vp_cfg.camera.image_overlap = cfg_.image_overlap;
    vp_cfg.camera.max_incidence_angle_deg = cfg_.max_incidence_angle_deg;
    vp_cfg.min_observation_count = cfg_.min_observation_count;
    vp_cfg.min_baseline_angle_deg = cfg_.min_baseline_angle_deg;
    vp_cfg.surface_sample_resolution = cfg_.surface_sample_resolution;
    vp_cfg.required_coverage_ratio = cfg_.min_predicted_coverage;
    vp_cfg.unknown_as_occupied = cfg_.visibility_unknown_as_occupied;
    vp_cfg.use_raw_occlusion_check = cfg_.visibility_use_raw_occlusion_check;
    vp_cfg.allow_single_view_boundary_coverage =
            cfg_.allow_single_view_boundary_coverage;
    vp_cfg.safe_radius = cfg_.safe_radius;
    vp_cfg.flight_height_min = cfg_.flight_height_min;
    vp_cfg.flight_height_max = cfg_.flight_height_max;
    vp_cfg.viewpoint_height_min = cfg_.camera_viewpoint_height_min;
    vp_cfg.viewpoint_height_max = cfg_.camera_viewpoint_height_max;
    vp_cfg.viewpoint_lateral_limit = cfg_.camera_viewpoint_lateral_limit;
    vp_cfg.max_viewpoints = cfg_.max_viewpoints;
    viewpoint_planner_ = std::make_shared<coverage::FaceViewpointPlanner>(vp_cfg);
}

void InspectionMissionPlanner::setCallbacks(
        SubmitNavFn submit_nav,
        PublishFaceRequestFn publish_face_request,
        PublishCaptureRequestFn publish_capture_request,
        PublishStatusFn publish_status,
        PublishViewpointsFn publish_viewpoints,
        MapProviderFn map_provider) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    submit_nav_ = std::move(submit_nav);
    publish_face_request_ = std::move(publish_face_request);
    publish_capture_request_ = std::move(publish_capture_request);
    publish_status_ = std::move(publish_status);
    publish_viewpoints_ = std::move(publish_viewpoints);
    map_provider_ = std::move(map_provider);
}

void InspectionMissionPlanner::setViewpointPlanner(
        std::shared_ptr<coverage::FaceViewpointPlanner> planner) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (planner) {
        viewpoint_planner_ = std::move(planner);
    }
}

InspectionState InspectionMissionPlanner::state() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_;
}

bool InspectionMissionPlanner::active() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return state_ != InspectionState::IDLE &&
           state_ != InspectionState::FINISHED &&
           state_ != InspectionState::FAILED;
}

NavigationRole InspectionMissionPlanner::expectedNavigationRole() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    switch (state_) {
        case InspectionState::GO_TO_TARGET:
            return NavigationRole::APPROACH_TARGET;
        case InspectionState::GO_TO_FACE_RECENTER:
            return NavigationRole::FACE_RECENTER;
        case InspectionState::GO_TO_VIEWPOINT:
            return NavigationRole::CAPTURE_VIEWPOINT;
        case InspectionState::RETURN_HOME:
            return NavigationRole::HOME;
        default:
            return NavigationRole::EXTERNAL_CLICK;
    }
}

MissionContext InspectionMissionPlanner::context() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return ctx_;
}

void InspectionMissionPlanner::setState(InspectionState state,
                                        const std::string &detail) {
    state_ = state;
    state_enter_time_sec_ = steadyNowSec();
    publishStatusLocked(detail);
    std::cout << "[InspectionMission] state -> " << toString(state_);
    if (!detail.empty()) {
        std::cout << " (" << detail << ")";
    }
    std::cout << std::endl;
}

void InspectionMissionPlanner::publishStatusLocked(const std::string &detail) const {
    if (!publish_status_) {
        return;
    }
    MissionStatusInfo info;
    info.mission_id = ctx_.mission_id;
    info.state = state_;
    info.viewpoint_index = static_cast<uint32_t>(ctx_.viewpoint_index);
    info.viewpoint_count = static_cast<uint32_t>(ctx_.viewpoints.size());
    info.target_version = ctx_.active_target.version;
    info.has_pending_target = ctx_.has_pending_target;
    info.detail = detail;
    info.failure_reason = ctx_.failure_reason;
    publish_status_(info);
}

bool InspectionMissionPlanner::searchApproachPoint(const Eigen::Vector3d &center,
                                                   const Eigen::Vector3d &normal,
                                                   Eigen::Vector3d &nav_goal,
                                                   const double preferred_distance) const {
    Eigen::Vector3d n = normal;
    if (n.norm() < 1e-6) {
        return false;
    }
    n.normalize();
    auto map = map_provider_ ? map_provider_() : nullptr;
    const auto accept_distance = [&](const double d) {
        const Eigen::Vector3d candidate = center + d * n;
        if (candidate.z() < cfg_.flight_height_min ||
            candidate.z() > cfg_.flight_height_max) {
            return false;
        }
        if (!map || !map->ready()) {
            nav_goal = candidate;
            return true;
        }
        const rog_map::Vec3f p(candidate.x(), candidate.y(), candidate.z());
        if (!map->insideLocalMap(p)) {
            return false;
        }
        const auto gt = map->getInfGridType(p);
        if (gt == rog_map::GridType::OCCUPIED || gt == rog_map::GridType::OUT_OF_MAP) {
            return false;
        }
        if (map->hasESDF()) {
            double dist = 0.0;
            rog_map::Vec3f grad = rog_map::Vec3f::Zero();
            if (!map->evaluateESDF(p, dist, grad) || !std::isfinite(dist) ||
                dist < cfg_.safe_radius) {
                return false;
            }
        } else if (map->getGridType(p) == rog_map::GridType::OCCUPIED) {
            return false;
        }
        nav_goal = candidate;
        return true;
    };

    // A cropped observation still measures the stand-off that produced useful
    // support.  Prefer retaining it after lateral/vertical centring; moving
    // unnecessarily closer can narrow a real sensor's vertical field of view.
    if (std::isfinite(preferred_distance) &&
        preferred_distance >= cfg_.approach_distance_min &&
        preferred_distance <= cfg_.approach_distance_max &&
        accept_distance(preferred_distance)) {
        return true;
    }
    for (double d = cfg_.approach_distance_min;
         d <= cfg_.approach_distance_max + 1e-9;
         d += std::max(0.05, cfg_.approach_distance_step)) {
        if (accept_distance(d)) {
            return true;
        }
    }
    return false;
}

MissionTarget InspectionMissionPlanner::buildPendingFromFace(
        const FaceObservation &face) const {
    MissionTarget pending = ctx_.active_target;
    pending.version = ctx_.active_target.version + 1;
    pending.face_center = face.center;
    pending.face_normal = face.normal;
    if (pending.face_normal.norm() > 1e-6) {
        pending.face_normal.normalize();
    }
    pending.face_prior_valid = true;
    pending.confidence = face.confidence;
    // Persist the detected blast-face position itself as the next mission's
    // target.  After a blast this point is expected to become free space and
    // therefore represents the new face prediction, rather than a stale
    // camera/inspection stand-off point in front of the old face.  The
    // stand-off used for this mission's photographs is generated separately
    // by FaceViewpointPlanner from capture_distance.
    pending.nav_goal = face.center;
    pending.goal_yaw = std::atan2(-pending.face_normal.y(), -pending.face_normal.x());

    pending.previous_face_region.center = face.center;
    pending.previous_face_region.normal = pending.face_normal;
    pending.previous_face_region.width = std::max(0.5, face.width);
    pending.previous_face_region.height = std::max(0.5, face.height);
    pending.previous_face_region.thickness = cfg_.change_region_thickness;
    pending.previous_face_region.valid = true;
    return pending;
}

FaceObservation InspectionMissionPlanner::makePriorFaceObservation() const {
    FaceObservation face;
    const MissionTarget &target = ctx_.active_target;
    if (!target.face_center.allFinite() || !target.face_normal.allFinite() ||
        target.face_normal.norm() < 1e-6) {
        return face;
    }
    const ChangeRegion &region = target.previous_face_region;
    const double width = region.valid ? region.width : 0.0;
    const double height = region.valid ? region.height : 0.0;
    if (width <= 0.0 || height <= 0.0) {
        return face;
    }
    face.valid = true;
    face.center = target.face_center;
    face.normal = target.face_normal.normalized();
    face.width = width;
    face.height = height;
    face.area = width * height;
    face.confidence = std::max(target.confidence, cfg_.face_min_confidence);
    face.extent_complete = true;
    face.extent_detail = "stored_face_extent";
    return face;
}

FaceObservation InspectionMissionPlanner::makeMockFaceObservation(
        const MissionPose &robot,
        const FaceDetectionRequest &request) const {
    FaceObservation obs;
    obs.mission_id = request.mission_id;
    obs.target_version = request.target_version;
    obs.request_id = request.request_id;
    const MissionTarget &t = ctx_.active_target;
    Eigen::Vector3d n = t.face_normal;
    if (n.norm() < 1e-6) {
        n = (robot.position - t.face_center);
    }
    if (n.norm() < 1e-6) {
        n = -Eigen::Vector3d::UnitX();
    }
    n.normalize();
    if (n.dot(robot.position - t.face_center) < 0.0) {
        n = -n;
    }
    // Advance face slightly along tunnel to emulate a new blast face.
    const Eigen::Vector3d tunnel = -n;
    obs.valid = true;
    obs.center = t.face_center + 1.5 * tunnel;
    obs.normal = n;
    obs.width = std::max(4.0, t.previous_face_region.width);
    obs.height = std::max(3.0, t.previous_face_region.height);
    obs.area = obs.width * obs.height;
    obs.confidence = 0.9;
    obs.extent_complete = true;
    obs.extent_detail = "mock_face_extent";
    obs.surface_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    if (std::abs(n.dot(up)) > 0.95) {
        up = Eigen::Vector3d::UnitY();
    }
    const Eigen::Vector3d u = (up.cross(n)).normalized();
    const Eigen::Vector3d v = n.cross(u);
    obs.tangent_u = u;
    obs.tangent_v = v;
    for (double ou = -0.5 * obs.width; ou <= 0.5 * obs.width; ou += 0.4) {
        for (double ov = -0.5 * obs.height; ov <= 0.5 * obs.height; ov += 0.4) {
            const Eigen::Vector3d p = obs.center + ou * u + ov * v;
            obs.surface_cloud->push_back(
                    pcl::PointXYZ(static_cast<float>(p.x()),
                                  static_cast<float>(p.y()),
                                  static_cast<float>(p.z())));
        }
    }
    return obs;
}

CaptureResult InspectionMissionPlanner::makeMockCaptureResult(
        const CaptureCommand &request) const {
    CaptureResult result;
    result.mission_id = request.mission_id;
    result.target_version = request.target_version;
    result.request_id = request.request_id;
    result.viewpoint_id = request.viewpoint.id;
    result.success = true;
    result.image_id = fmt::format("mock_{}_{}", ctx_.mission_id, request.viewpoint.id);
    result.reason = "mock_ok";
    return result;
}

bool InspectionMissionPlanner::start(const MissionPose &home,
                                     const std::string &mission_id,
                                     const MissionPose *approach_override) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (state_ != InspectionState::IDLE && state_ != InspectionState::FINISHED &&
        state_ != InspectionState::FAILED) {
        return false;
    }
    if (!store_) {
        ctx_.failure_reason = "missing_target_store";
        setState(InspectionState::FAILED, ctx_.failure_reason);
        return false;
    }

    MissionTarget target;
    if (!store_->load(target)) {
        ctx_.failure_reason = "failed_to_load_target";
        setState(InspectionState::FAILED, ctx_.failure_reason);
        return false;
    }

    ctx_ = MissionContext{};
    ctx_.active_target = target;
    if (approach_override != nullptr) {
        if (!approach_override->position.allFinite() ||
            !std::isfinite(approach_override->yaw)) {
            ctx_.failure_reason = "invalid_trigger_goal";
            setState(InspectionState::FAILED, ctx_.failure_reason);
            return false;
        }
        // The input goal applies to this run only. A later successful
        // face/capture transaction is still the only path that persists a
        // replacement target.
        ctx_.active_target.nav_goal = approach_override->position;
        ctx_.active_target.goal_yaw = approach_override->yaw;
    }
    ctx_.home_pose = home;
    ctx_.last_navigation_pose = home;
    ctx_.mission_id =
            mission_id.empty() ? fmt::format("insp_{}", nowMs()) : mission_id;
    ctx_.has_pending_target = false;
    ctx_.viewpoints.clear();
    ctx_.viewpoint_index = 0;
    ctx_.mission_start_time_sec = steadyNowSec();

    MissionPose approach;
    approach.position = ctx_.active_target.nav_goal;
    approach.yaw = ctx_.active_target.goal_yaw;
    if (approach_override == nullptr && !cfg_.use_target_nav_goal_directly) {
        Eigen::Vector3d searched = approach.position;
        if (searchApproachPoint(target.face_center, target.face_normal, searched)) {
            approach.position = searched;
        }
    }

    if (!submit_nav_) {
        ctx_.failure_reason = "missing_nav_callback";
        setState(InspectionState::FAILED, ctx_.failure_reason);
        return false;
    }
    setState(InspectionState::GO_TO_TARGET, "approach_target");
    if (!submit_nav_(approach, NavigationRole::APPROACH_TARGET)) {
        ctx_.failure_reason = "submit_approach_failed";
        setState(InspectionState::FAILED, ctx_.failure_reason);
        return false;
    }
    return true;
}

void InspectionMissionPlanner::cancel(const std::string &reason) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (state_ == InspectionState::IDLE || state_ == InspectionState::FINISHED ||
        state_ == InspectionState::FAILED) {
        return;
    }
    ctx_.failure_reason = reason.empty() ? "cancelled" : reason;
    // Do not overwrite persisted target on cancel.
    ctx_.has_pending_target = false;
    ctx_.has_pending_face_observation = false;
    returnHome();
}

void InspectionMissionPlanner::tick() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const double elapsed = steadyNowSec() - state_enter_time_sec_;
    if (state_ == InspectionState::WAIT_FACE_RESULT &&
        elapsed > std::max(0.0, cfg_.face_result_timeout_sec)) {
        failAndReturnHome("face_result_timeout");
    } else if (state_ == InspectionState::WAIT_CAPTURE_RESULT) {
        const double settle = std::max(0.0, cfg_.capture_settle_time_sec);
        if (ctx_.active_capture_request_id == 0 && elapsed >= settle &&
            ctx_.viewpoint_index < ctx_.viewpoints.size()) {
            CaptureCommand request;
            request.mission_id = ctx_.mission_id;
            request.target_version = ctx_.active_target.version;
            request.request_id = ctx_.next_request_id++;
            request.viewpoint = ctx_.viewpoints[ctx_.viewpoint_index];
            ctx_.active_capture_request_id = request.request_id;
            if (ctx_.first_capture_request_time_sec < 0.0) {
                ctx_.first_capture_request_time_sec = steadyNowSec();
            }
            fmt::print(fg(fmt::color::cyan),
                       " -- [Inspection] Capture {}/{}: hover dwell {:.2f}s complete; trigger camera.\n",
                       ctx_.viewpoint_index + 1,
                       ctx_.viewpoints.size(),
                       settle);
            if (publish_capture_request_) {
                publish_capture_request_(request);
            }
            if (cfg_.mock_capture) {
                onCaptureResult(makeMockCaptureResult(request));
            }
        } else if (ctx_.active_capture_request_id != 0 &&
                   elapsed > settle + std::max(0.0, cfg_.capture_result_timeout_sec)) {
            failAndReturnHome("capture_result_timeout");
        }
    }
}

void InspectionMissionPlanner::returnHome(const std::string &detail) {
    if (!submit_nav_) {
        setState(InspectionState::FAILED, "missing_nav_callback");
        return;
    }
    setState(InspectionState::RETURN_HOME, detail);
    if (!submit_nav_(ctx_.home_pose, NavigationRole::HOME)) {
        ctx_.failure_reason = "submit_home_failed";
        setState(InspectionState::FAILED, ctx_.failure_reason);
    }
}

void InspectionMissionPlanner::failAndReturnHome(const std::string &reason) {
    ctx_.failure_reason = reason;
    ctx_.has_pending_target = false;
    ctx_.has_pending_face_observation = false;
    auto map = map_provider_ ? map_provider_() : nullptr;
    if (map) {
        const auto robot = map->getRobotState();
        if (robot.rcv && robot.p.allFinite()) {
            const Eigen::Vector3d robot_p(robot.p.x(), robot.p.y(), robot.p.z());
            if ((robot_p - ctx_.home_pose.position).norm() < 1.0) {
                setState(InspectionState::FAILED, reason);
                return;
            }
        }
    }
    returnHome();
}

bool InspectionMissionPlanner::requestFaceDetection(const std::string &detail) {
    setState(InspectionState::WAIT_FACE_RESULT, detail);
    FaceDetectionRequest request;
    request.mission_id = ctx_.mission_id;
    request.target_version = ctx_.active_target.version;
    request.request_id = ctx_.next_request_id++;
    ctx_.active_face_request_id = request.request_id;
    if (publish_face_request_) {
        publish_face_request_(request);
    }
    return true;
}

bool InspectionMissionPlanner::recenterForClippedFace(
        const FaceObservation &observation) {
    if (observation.extent_detail != "face_extent_clipped" ||
        !cfg_.face_recenter_on_clipped ||
        cfg_.face_recenter_max_attempts <= 0 ||
        ctx_.face_recenter_attempts >=
                static_cast<uint32_t>(cfg_.face_recenter_max_attempts)) {
        return false;
    }
    if (!observation.center.allFinite() || !observation.normal.allFinite() ||
        observation.normal.norm() < 1e-6 || !submit_nav_) {
        return false;
    }

    // The detector convention is that the normal points into free space /
    // toward the observing vehicle.  Enforce that convention again before
    // searching so an upstream detector cannot send the retry through the
    // blast face.
    Eigen::Vector3d normal = observation.normal.normalized();
    const Eigen::Vector3d to_robot =
            ctx_.last_navigation_pose.position - observation.center;
    if (to_robot.norm() > 1e-6 && normal.dot(to_robot) < 0.0) {
        normal = -normal;
    }

    MissionPose recenter_goal;
    const double observed_standoff = normal.dot(to_robot);
    if (!searchApproachPoint(observation.center,
                             normal,
                             recenter_goal.position,
                             observed_standoff)) {
        return false;
    }
    recenter_goal.yaw = std::atan2(-normal.y(), -normal.x());
    if (!std::isfinite(recenter_goal.yaw)) {
        return false;
    }

    ++ctx_.face_recenter_attempts;
    ctx_.active_face_request_id = 0;
    setState(InspectionState::GO_TO_FACE_RECENTER,
             fmt::format("face_extent_clipped_recenter_{}/{}",
                         ctx_.face_recenter_attempts,
                         std::max(0, cfg_.face_recenter_max_attempts)));
    fmt::print(fg(fmt::color::yellow),
               " -- [Inspection] Face support reached the detection ROI; recenter {}/{} to "
               "({:.2f}, {:.2f}, {:.2f}), yaw {:.3f} and detect again.\n",
               ctx_.face_recenter_attempts,
               std::max(0, cfg_.face_recenter_max_attempts),
               recenter_goal.position.x(),
               recenter_goal.position.y(),
               recenter_goal.position.z(),
               recenter_goal.yaw);
    if (!submit_nav_(recenter_goal, NavigationRole::FACE_RECENTER)) {
        failAndReturnHome("submit_face_recenter_failed");
    }
    return true;
}

bool InspectionMissionPlanner::dispatchCurrentViewpoint() {
    if (ctx_.viewpoint_index >= ctx_.viewpoints.size()) {
        return false;
    }
    if (!submit_nav_) {
        return false;
    }
    const auto &vp = ctx_.viewpoints[ctx_.viewpoint_index];
    MissionPose goal;
    goal.position = vp.position;
    goal.yaw = vp.body_yaw;
    setState(InspectionState::GO_TO_VIEWPOINT,
             fmt::format("viewpoint_{}/{}",
                         ctx_.viewpoint_index + 1,
                         ctx_.viewpoints.size()));
    return submit_nav_(goal, NavigationRole::CAPTURE_VIEWPOINT);
}

bool InspectionMissionPlanner::planViewsFromPending(const MissionPose &robot) {
    setState(InspectionState::PLAN_VIEWS, "planning_views");
    if (!ctx_.has_pending_face_observation) {
        return false;
    }
    auto map = map_provider_ ? map_provider_() : nullptr;
    // Preserve the detected surface instead of silently replacing it with a
    // synthetic rectangle derived from the persisted target summary.
    FaceObservation face = ctx_.pending_face_observation;

    CoveragePlan coverage;
    if (viewpoint_planner_ && map && map->ready()) {
        coverage = viewpoint_planner_->plan(face, *map, robot.position,
                                            ctx_.home_pose.position);
    }
    if ((!coverage.valid || coverage.ordered_viewpoints.empty() ||
         coverage.predicted_coverage < cfg_.min_predicted_coverage) &&
        (cfg_.mock_face_detection || cfg_.mock_capture) &&
        cfg_.allow_mock_coverage_fallback) {
        // Explicitly opt-in legacy mock fallback. This mode is never used as
        // evidence of camera coverage.
        CaptureViewpoint vp;
        vp.id = 1;
        // The persisted navigation goal is the predicted next face center,
        // not an inspection pose.  Keep the opt-in mock fallback consistent
        // with the normal capture geometry.
        Eigen::Vector3d n = ctx_.pending_face_observation.normal;
        if (n.norm() < 1e-6) {
            publishStatusLocked("plan_views_rejected;invalid_fallback_normal");
            return false;
        }
        n.normalize();
        vp.position = ctx_.pending_face_observation.center + cfg_.capture_distance * n;
        vp.body_yaw = std::atan2(-n.y(), -n.x());
        vp.camera_pitch = 0.0;
        vp.expected_coverage_gain = 1.0;
        coverage.ordered_viewpoints = {vp};
        coverage.valid = true;
        coverage.predicted_coverage = 1.0;
        coverage.detail = "fallback_single_view";
    }

    if (!coverage.valid || coverage.ordered_viewpoints.empty() ||
        coverage.predicted_coverage < cfg_.min_predicted_coverage) {
        publishStatusLocked(fmt::format("plan_views_rejected;coverage={:.2f};{}",
                                        coverage.predicted_coverage,
                                        coverage.detail));
        return false;
    }

    ctx_.viewpoints = coverage.ordered_viewpoints;
    ctx_.viewpoint_index = 0;
    ctx_.capture_workflow_start_time_sec = steadyNowSec();
    const double face_area = face.area > 0.0 ? face.area : face.width * face.height;
    fmt::print(fg(fmt::color::cyan),
               " -- [Inspection] Detected face: {:.2f}m x {:.2f}m = {:.2f}m^2; "
               "viewpoints={}, predicted_coverage={:.2f}, capture_dwell={:.2f}s.\n",
               face.width,
               face.height,
               face_area,
               ctx_.viewpoints.size(),
               coverage.predicted_coverage,
               std::max(0.0, cfg_.capture_settle_time_sec));
    if (publish_viewpoints_) {
        publish_viewpoints_(face, coverage);
    }
    publishStatusLocked(fmt::format("views={};coverage={:.2f};face={:.2f}x{:.2f}m;area={:.2f}m2;{}",
                                    ctx_.viewpoints.size(),
                                    coverage.predicted_coverage,
                                    face.width,
                                    face.height,
                                    face_area,
                                    coverage.detail));
    return dispatchCurrentViewpoint();
}

bool InspectionMissionPlanner::commitPendingTarget() {
    if (!ctx_.has_pending_target) {
        return false;
    }
    if (!cfg_.persist_target_on_success) {
        // A static replay has no blast/update between runs.  Retaining the
        // initial observation anchor keeps repeated missions observable while
        // still exercising the complete detection and coverage transaction.
        ctx_.has_pending_target = false;
        ctx_.has_pending_face_observation = false;
        return true;
    }
    if (!store_) {
        return false;
    }
    if (!store_->saveAtomic(ctx_.pending_target)) {
        return false;
    }
    ctx_.active_target = ctx_.pending_target;
    ctx_.has_pending_target = false;
    ctx_.has_pending_face_observation = false;
    return true;
}

void InspectionMissionPlanner::onNavigationSucceeded(NavigationRole role,
                                                     const MissionPose &robot) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    ctx_.last_navigation_pose = robot;
    switch (state_) {
        case InspectionState::GO_TO_TARGET: {
            if (role != NavigationRole::APPROACH_TARGET) {
                return;
            }
            if (cfg_.navigation_only) {
                // MaRSIM currently exposes a LiDAR-only vehicle model. Reaching
                // nav_goal is the outward-leg assertion; return home without
                // fabricating a successful face/capture/coverage transition.
                returnHome("navigation_target_reached_returning_home");
                return;
            }
            if (cfg_.skip_face_detection) {
                const FaceObservation prior_face = makePriorFaceObservation();
                if (!prior_face.valid) {
                    failAndReturnHome("invalid_prior_face_geometry");
                    return;
                }
                // This branch is an inspection of a known face, not a newly
                // detected face. Keep has_pending_target false so capture
                // completion cannot overwrite the persisted navigation goal.
                ctx_.pending_face_observation = prior_face;
                ctx_.has_pending_face_observation = true;
                if (!planViewsFromPending(robot)) {
                    failAndReturnHome("plan_views_from_prior_failed");
                }
                return;
            }
            requestFaceDetection("request_face");
            if (cfg_.mock_face_detection) {
                // Unlock temporarily is unsafe; synthesize under same lock.
                FaceDetectionRequest request;
                request.mission_id = ctx_.mission_id;
                request.target_version = ctx_.active_target.version;
                request.request_id = ctx_.active_face_request_id;
                const auto mock = makeMockFaceObservation(robot, request);
                // Re-enter through unlocked path by inlined handling:
                if (!mock.valid) {
                    failAndReturnHome("mock_face_invalid");
                    return;
                }
                ctx_.pending_target = buildPendingFromFace(mock);
                ctx_.has_pending_target = true;
                ctx_.pending_face_observation = mock;
                ctx_.has_pending_face_observation = true;
                if (!planViewsFromPending(robot)) {
                    failAndReturnHome("plan_views_failed");
                }
            }
            return;
        }
        case InspectionState::GO_TO_FACE_RECENTER: {
            if (role != NavigationRole::FACE_RECENTER) {
                return;
            }
            if (!requestFaceDetection("request_face_after_recenter")) {
                failAndReturnHome("request_face_after_recenter_failed");
            }
            return;
        }
        case InspectionState::GO_TO_VIEWPOINT: {
            if (role != NavigationRole::CAPTURE_VIEWPOINT) {
                return;
            }
            if (ctx_.viewpoint_index >= ctx_.viewpoints.size()) {
                failAndReturnHome("viewpoint_index_oob");
                return;
            }
            const auto &vp = ctx_.viewpoints[ctx_.viewpoint_index];
            setState(InspectionState::WAIT_CAPTURE_RESULT,
                     fmt::format("settling_capture_{}", vp.id));
            // The capture request is emitted by tick() after the configured
            // hover dwell, so real and mock cameras follow identical timing.
            ctx_.active_capture_request_id = 0;
            return;
        }
        case InspectionState::RETURN_HOME: {
            if (role != NavigationRole::HOME) {
                return;
            }
            if (!ctx_.failure_reason.empty() && !ctx_.has_pending_target &&
                state_ == InspectionState::RETURN_HOME) {
                // Distinguish cancel/fail vs success by failure_reason presence
                // after successful commit clears pending and failure_reason stays empty.
            }
            if (!ctx_.failure_reason.empty()) {
                setState(InspectionState::FAILED, ctx_.failure_reason);
            } else if (cfg_.navigation_only) {
                setState(InspectionState::FINISHED, "navigation_only_complete");
            } else {
                const double now = steadyNowSec();
                const double mission_elapsed = ctx_.mission_start_time_sec >= 0.0
                                                       ? now - ctx_.mission_start_time_sec
                                                       : 0.0;
                const double capture_elapsed =
                        ctx_.capture_workflow_start_time_sec >= 0.0 &&
                                ctx_.capture_workflow_finish_time_sec >= 0.0
                                ? ctx_.capture_workflow_finish_time_sec -
                                          ctx_.capture_workflow_start_time_sec
                                : 0.0;
                const double plan_to_first_capture =
                        ctx_.capture_workflow_start_time_sec >= 0.0 &&
                                ctx_.first_capture_request_time_sec >= 0.0
                                ? ctx_.first_capture_request_time_sec -
                                          ctx_.capture_workflow_start_time_sec
                                : 0.0;
                fmt::print(fg(fmt::color::cyan),
                           " -- [Inspection] Mission timing: total={:.2f}s, "
                           "capture_workflow={:.2f}s, plan_to_first_capture={:.2f}s.\n",
                           mission_elapsed,
                           capture_elapsed,
                           plan_to_first_capture);
                setState(InspectionState::FINISHED, "mission_complete");
            }
            return;
        }
        default:
            return;
    }
}

void InspectionMissionPlanner::onNavigationFailed(NavigationRole role,
                                                   const std::string &reason) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!active()) {
        return;
    }
    const std::string failure = reason.empty() ? "navigation_failed" : reason;
    if (role == NavigationRole::HOME || state_ == InspectionState::RETURN_HOME) {
        // Retrying a failed return-home leg by issuing the identical goal can
        // loop forever. Surface the failure instead; the vehicle remains at
        // its last safe commanded state.
        ctx_.failure_reason = failure;
        ctx_.has_pending_target = false;
        ctx_.has_pending_face_observation = false;
        setState(InspectionState::FAILED, failure);
        return;
    }
    failAndReturnHome(failure);
}

void InspectionMissionPlanner::onFaceObservation(const FaceObservation &observation) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (state_ != InspectionState::WAIT_FACE_RESULT) {
        return;
    }
    if (cfg_.mock_face_detection) {
        // Mock path already handled inside navigation success.
        return;
    }
    if (observation.mission_id != ctx_.mission_id ||
        observation.target_version != ctx_.active_target.version ||
        observation.request_id == 0 ||
        observation.request_id != ctx_.active_face_request_id) {
        return;
    }
    if ((!observation.valid || !observation.extent_complete) &&
        recenterForClippedFace(observation)) {
        return;
    }
    if (!observation.valid || !observation.extent_complete ||
        observation.confidence < cfg_.face_min_confidence ||
        observation.area < cfg_.face_min_area) {
        failAndReturnHome(observation.extent_detail.empty()
                                  ? "invalid_face_observation"
                                  : observation.extent_detail);
        return;
    }
    ctx_.active_face_request_id = 0;
    ctx_.pending_target = buildPendingFromFace(observation);
    ctx_.has_pending_target = true;
    ctx_.pending_face_observation = observation;
    ctx_.has_pending_face_observation = true;
    if (!planViewsFromPending(ctx_.last_navigation_pose)) {
        failAndReturnHome("plan_views_failed");
    }
}

void InspectionMissionPlanner::onCaptureResult(const CaptureResult &result) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (state_ != InspectionState::WAIT_CAPTURE_RESULT) {
        return;
    }
    if (result.mission_id != ctx_.mission_id ||
        result.target_version != ctx_.active_target.version ||
        result.request_id == 0 ||
        result.request_id != ctx_.active_capture_request_id ||
        ctx_.viewpoint_index >= ctx_.viewpoints.size() ||
        result.viewpoint_id != ctx_.viewpoints[ctx_.viewpoint_index].id) {
        return;
    }
    if (!result.success) {
        failAndReturnHome(result.reason.empty() ? "capture_failed" : result.reason);
        return;
    }
    ctx_.active_capture_request_id = 0;
    ++ctx_.viewpoint_index;
    if (ctx_.viewpoint_index < ctx_.viewpoints.size()) {
        if (!dispatchCurrentViewpoint()) {
            failAndReturnHome("next_viewpoint_submit_failed");
        }
        return;
    }
    if (ctx_.has_pending_target) {
        if (!commitPendingTarget()) {
            failAndReturnHome("commit_target_failed");
            return;
        }
    }
    ctx_.capture_workflow_finish_time_sec = steadyNowSec();
    const double capture_elapsed = ctx_.capture_workflow_start_time_sec >= 0.0
                                           ? ctx_.capture_workflow_finish_time_sec -
                                                     ctx_.capture_workflow_start_time_sec
                                           : 0.0;
    fmt::print(fg(fmt::color::cyan),
               " -- [Inspection] Capture workflow complete: photos={}, elapsed={:.2f}s "
               "(dwell {:.2f}s/photo).\n",
               ctx_.viewpoints.size(),
               capture_elapsed,
               std::max(0.0, cfg_.capture_settle_time_sec));
    returnHome(fmt::format("capture_complete;photos={};capture_elapsed_sec={:.2f}",
                           ctx_.viewpoints.size(), capture_elapsed));
}

}  // namespace mission
