#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mission {

enum class InspectionState {
    IDLE = 0,
    GO_TO_TARGET,
    // The first face observation reached its ROI boundary.  Navigate to a
    // face-centred stand-off pose and perform one fresh observation instead
    // of treating a cropped rectangle as coverage-ready.
    GO_TO_FACE_RECENTER,
    WAIT_FACE_RESULT,
    PLAN_VIEWS,
    GO_TO_VIEWPOINT,
    WAIT_CAPTURE_RESULT,
    RETURN_HOME,
    FINISHED,
    FAILED
};

enum class NavigationRole {
    EXTERNAL_CLICK = 0,
    APPROACH_TARGET,
    FACE_RECENTER,
    CAPTURE_VIEWPOINT,
    HOME
};

inline const char *toString(const InspectionState state) {
    switch (state) {
        case InspectionState::IDLE:
            return "IDLE";
        case InspectionState::GO_TO_TARGET:
            return "GO_TO_TARGET";
        case InspectionState::GO_TO_FACE_RECENTER:
            return "GO_TO_FACE_RECENTER";
        case InspectionState::WAIT_FACE_RESULT:
            return "WAIT_FACE_RESULT";
        case InspectionState::PLAN_VIEWS:
            return "PLAN_VIEWS";
        case InspectionState::GO_TO_VIEWPOINT:
            return "GO_TO_VIEWPOINT";
        case InspectionState::WAIT_CAPTURE_RESULT:
            return "WAIT_CAPTURE_RESULT";
        case InspectionState::RETURN_HOME:
            return "RETURN_HOME";
        case InspectionState::FINISHED:
            return "FINISHED";
        case InspectionState::FAILED:
            return "FAILED";
        default:
            return "UNKNOWN";
    }
}

inline const char *toString(const NavigationRole role) {
    switch (role) {
        case NavigationRole::EXTERNAL_CLICK:
            return "EXTERNAL_CLICK";
        case NavigationRole::APPROACH_TARGET:
            return "APPROACH_TARGET";
        case NavigationRole::FACE_RECENTER:
            return "FACE_RECENTER";
        case NavigationRole::CAPTURE_VIEWPOINT:
            return "CAPTURE_VIEWPOINT";
        case NavigationRole::HOME:
            return "HOME";
        default:
            return "UNKNOWN";
    }
}

struct ChangeRegion {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::UnitX()};
    double width{0.0};
    double height{0.0};
    double thickness{1.0};
    bool valid{false};
};

struct MissionTarget {
    uint32_t version{0};
    std::string scene_id;
    std::string map_version;

    Eigen::Vector3d face_center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d face_normal{-Eigen::Vector3d::UnitX()};
    // False for an operator-specified initial observation anchor.  It becomes
    // true only after a complete detection/capture transaction is committed.
    bool face_prior_valid{false};
    Eigen::Vector3d nav_goal{Eigen::Vector3d::Zero()};
    double goal_yaw{0.0};

    double confidence{0.0};
    ChangeRegion previous_face_region;
};

struct FaceObservation {
    std::string mission_id;
    uint32_t target_version{0};
    uint32_t request_id{0};
    bool valid{false};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{-Eigen::Vector3d::UnitX()};
    // `tangent_u` and `tangent_v` define the rectangle used by detection,
    // coverage planning and visualization.  Keeping them with the observation
    // avoids silently rotating a PCA-sized rectangle when it crosses a ROS
    // boundary or is rendered in RViz.
    Eigen::Vector3d tangent_u{Eigen::Vector3d::Zero()};
    Eigen::Vector3d tangent_v{Eigen::Vector3d::Zero()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud{
            new pcl::PointCloud<pcl::PointXYZ>()};
    double width{0.0};
    double height{0.0};
    double area{0.0};
    double confidence{0.0};
    // False means the detected support reaches the perception ROI boundary;
    // the reported extent can then be only a cropped part of the face and is
    // never eligible for a coverage mission.
    bool extent_complete{false};
    std::string extent_detail;
};

struct CaptureViewpoint {
    uint32_t id{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double body_yaw{0.0};
    double camera_pitch{0.0};
    std::vector<int> visible_surface_ids;
    double expected_coverage_gain{0.0};
};

struct FaceDetectionRequest {
    std::string mission_id;
    uint32_t target_version{0};
    uint32_t request_id{0};
};

struct CaptureCommand {
    std::string mission_id;
    uint32_t target_version{0};
    uint32_t request_id{0};
    CaptureViewpoint viewpoint;
};

struct CoveragePlan {
    bool valid{false};
    std::vector<CaptureViewpoint> ordered_viewpoints;
    double predicted_coverage{0.0};
    std::string detail;
};

struct CaptureResult {
    std::string mission_id;
    uint32_t target_version{0};
    uint32_t request_id{0};
    uint32_t viewpoint_id{0};
    bool success{false};
    std::string image_id;
    std::string reason;
};

struct MissionStatusInfo {
    std::string mission_id;
    InspectionState state{InspectionState::IDLE};
    uint32_t viewpoint_index{0};
    uint32_t viewpoint_count{0};
    uint32_t target_version{0};
    bool has_pending_target{false};
    std::string detail;
    std::string failure_reason;
};

struct MissionPose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
};

struct MissionContext {
    MissionTarget active_target;
    MissionTarget pending_target;
    bool has_pending_target{false};
    FaceObservation pending_face_observation;
    bool has_pending_face_observation{false};

    MissionPose home_pose;
    // Updated only from a navigation-success event; used to start coverage
    // ordering from the actual approach pose rather than a synthetic goal.
    MissionPose last_navigation_pose;
    std::vector<CaptureViewpoint> viewpoints;
    std::size_t viewpoint_index{0};

    uint32_t next_request_id{1};
    uint32_t active_face_request_id{0};
    uint32_t active_capture_request_id{0};
    // Number of detection-driven recenter moves in this run.  It is runtime
    // state only: an incomplete face must never overwrite MissionTarget.
    uint32_t face_recenter_attempts{0};

    // Monotonic-clock timestamps for the portion of the mission that starts
    // when the coverage plan is accepted and ends at the final successful
    // camera acknowledgement. They are runtime diagnostics only and are not
    // persisted with MissionTarget.
    double mission_start_time_sec{-1.0};
    double capture_workflow_start_time_sec{-1.0};
    double first_capture_request_time_sec{-1.0};
    double capture_workflow_finish_time_sec{-1.0};

    std::string mission_id;
    std::string failure_reason;
};

struct InspectionMissionConfig {
    bool enable{false};
    // MaRSIM/LiDAR integration profile: validate nav_goal arrival without
    // pretending that perception, capture, or target persistence succeeded.
    bool navigation_only{false};
    // Start one mission automatically after the first fresh odometry sample.
    // This is deliberately separate from fsm/auto_start, which only starts
    // the generic FSM and does not provide a mission target.
    bool auto_start{false};
    // Interpret a 2D /goal message as a mission trigger. Its XY and selected
    // flight height become this run's approach waypoint.
    bool trigger_from_2d_goal{false};
    // Use the face geometry already stored in MissionTarget after reaching the
    // approach waypoint. This is the LiDAR-only integration path when no
    // detector is available; it never creates a replacement target.
    bool skip_face_detection{false};
    // A MissionTarget's nav_goal is the operator-configured or last detected
    // blast-face target. Keep it unchanged unless a caller explicitly asks
    // for a legacy searched stand-off point.
    bool use_target_nav_goal_directly{true};
    // Only for legacy mocked demonstrations. Never use this to declare
    // coverage success in a camera-backed mission.
    bool allow_mock_coverage_fallback{false};
    // Legacy combined switch. New configurations should set the two switches
    // below independently so a real LiDAR detector can be used together with
    // a simulated camera acknowledgement.
    bool mock_external{true};
    bool mock_face_detection{true};
    bool mock_capture{true};
    bool use_internal_detector{false};
    bool apply_change_region_mask{true};
    // Real excavation advances the face, so a successful transaction normally
    // becomes the next mission's prior.  Disable this only for static replay
    // scenes, where writing the observed wall back would move the next
    // approach point into the same, unchanged wall.
    bool persist_target_on_success{true};
    std::string target_file{"config/mission_target.yaml"};

    std::string start_service{"/inspection/start"};
    std::string face_request_topic{"/inspection/face/request"};
    std::string face_result_topic{"/inspection/face/result"};
    // Successful internal detections are echoed here for evaluation.  This is
    // intentionally different from face_result_topic to avoid feeding the
    // observation back through the external-result subscriber.
    std::string face_debug_topic{"/inspection/face/debug"};
    // Latched MarkerArray containing the detected face boundary, generated
    // capture viewpoints, viewing rays, and their execution order.
    std::string viewpoint_debug_topic{"/inspection/viewpoints"};
    std::string capture_request_topic{"/inspection/capture/request"};
    std::string capture_result_topic{"/inspection/capture/result"};
    std::string status_topic{"/inspection/status"};
    std::string cloud_topic{"/cloud_registered"};

    std::string home_mode{"capture_on_trigger"};

    double approach_distance_min{2.0};
    double approach_distance_max{4.0};
    double approach_distance_step{0.2};
    double safe_radius{0.6};
    double flight_height_min{0.5};
    double flight_height_max{5.0};

    double face_forward_min{1.0};
    double face_forward_max{12.0};
    double face_min_confidence{0.75};
    double face_min_area{4.0};
    int face_min_points{300};
    double face_normal_alignment_min{0.8};
    double face_voxel_leaf{0.1};
    int face_stability_frames{3};
    double face_stability_center_tol{0.35};
    double face_stability_normal_tol{0.15};
    double face_cluster_tolerance{0.35};
    int face_cluster_min_size{200};
    double face_ransac_dist{0.08};
    // Detection ROI centred at the inspection pose.  It must contain the
    // complete end face plus this boundary margin; otherwise detection fails
    // instead of declaring coverage of a cropped patch.
    double face_lateral_half_width{7.0};
    double face_vertical_half_height{5.0};
    double face_roi_edge_margin{0.35};
    // RANSAC supplies only a local normal seed.  All cluster points within
    // this normal distance form the end-face support used for the extent.
    double face_support_plane_distance{0.45};
    // Conservative exterior margin around the measured support rectangle.
    double face_extent_padding{0.25};
    double face_prior_center_tolerance{3.0};
    double face_prior_normal_alignment_min{0.9};
    // A support cloud that touches the ROI boundary carries useful geometry
    // (centre and normal), but not a complete face extent.  Recenter once
    // from that geometry and rerun detection with a fresh ROI.  Disable this
    // explicitly for deployments that require a manually fixed observation
    // pose.
    bool face_recenter_on_clipped{true};
    int face_recenter_max_attempts{1};

    double camera_hfov_deg{70.0};
    double camera_vfov_deg{50.0};
    // Mechanical camera/gimbal pitch limits.  These are distinct from the
    // viewing FOV and let a camera in the legal flight-height band inspect a
    // taller face.
    double camera_min_pitch_deg{-35.0};
    double camera_max_pitch_deg{35.0};
    double capture_distance{4.0};
    double image_overlap{0.7};
    int min_observation_count{2};
    double min_baseline_angle_deg{8.0};
    double max_incidence_angle_deg{60.0};
    double surface_sample_resolution{0.4};
    // A visibility ray is an image-quality test, not a flight-safety test.
    // Flight poses remain checked against the inflated map; this controls
    // whether unobserved cells may block a candidate image ray.
    bool visibility_unknown_as_occupied{false};
    bool visibility_use_raw_occlusion_check{true};
    bool allow_single_view_boundary_coverage{false};
    double camera_viewpoint_height_min{0.5};
    double camera_viewpoint_height_max{5.0};
    double camera_viewpoint_lateral_limit{0.0};
    // A mission may complete only after the whole coverage rectangle satisfies
    // its multi-view requirement.  Lower values are retained as an explicit
    // escape hatch for degraded operations, not the normal inspection mode.
    double min_predicted_coverage{1.0};
    int max_viewpoints{60};
    // Hold at a capture pose before triggering the camera.  This gives the
    // vehicle estimator/camera time to settle and ensures the next leg starts
    // from a genuinely stationary state.
    double capture_settle_time_sec{0.5};

    double change_region_thickness{1.0};
    int fail_retry_count{1};
    double face_result_timeout_sec{15.0};
    double capture_result_timeout_sec{10.0};
    // Navigation is complete for an inspection leg only after both position
    // and viewing yaw have converged.  This avoids starting detection or a
    // capture while the vehicle is still turning at the target position.
    double arrival_yaw_tolerance_rad{0.15};
};

inline bool pointInChangeRegion(const Eigen::Vector3d &p, const ChangeRegion &region) {
    if (!region.valid || region.width <= 0.0 || region.height <= 0.0 ||
        region.thickness <= 0.0) {
        return false;
    }
    Eigen::Vector3d n = region.normal;
    if (n.norm() < 1e-6) {
        return false;
    }
    n.normalize();
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    if (std::abs(n.dot(up)) > 0.95) {
        up = Eigen::Vector3d::UnitY();
    }
    const Eigen::Vector3d u = (up.cross(n)).normalized();
    const Eigen::Vector3d v = n.cross(u);
    const Eigen::Vector3d d = p - region.center;
    const double du = d.dot(u);
    const double dv = d.dot(v);
    const double dn = d.dot(n);
    return std::abs(du) <= 0.5 * region.width &&
           std::abs(dv) <= 0.5 * region.height &&
           std::abs(dn) <= 0.5 * region.thickness;
}

}  // namespace mission
