#pragma once

#include <coverage/face_viewpoint_planner.hpp>
#include <mission/mission_target_store.hpp>
#include <mission/mission_types.hpp>

#include <map_manager/map_manager.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace mission {

class InspectionMissionPlanner {
public:
    using SubmitNavFn = std::function<bool(const MissionPose &goal, NavigationRole role)>;
    using PublishFaceRequestFn = std::function<void(const FaceDetectionRequest &)>;
    using PublishCaptureRequestFn = std::function<void(const CaptureCommand &)>;
    using PublishStatusFn = std::function<void(const MissionStatusInfo &)>;
    using PublishViewpointsFn = std::function<void(const FaceObservation &,
                                                   const CoveragePlan &)>;
    using MapProviderFn = std::function<general_planner::MapManager::Ptr()>;

    InspectionMissionPlanner(InspectionMissionConfig config,
                             std::shared_ptr<MissionTargetStore> store);

    void setCallbacks(SubmitNavFn submit_nav,
                      PublishFaceRequestFn publish_face_request,
                      PublishCaptureRequestFn publish_capture_request,
                      PublishStatusFn publish_status,
                      PublishViewpointsFn publish_viewpoints,
                      MapProviderFn map_provider);

    void setViewpointPlanner(std::shared_ptr<coverage::FaceViewpointPlanner> planner);

    InspectionState state() const;
    bool active() const;
    NavigationRole expectedNavigationRole() const;
    /** Thread-safe snapshot; callers must not observe mutable mission state. */
    MissionContext context() const;
    const InspectionMissionConfig &config() const {
        return cfg_;
    }

    bool start(const MissionPose &home,
               const std::string &mission_id = "",
               const MissionPose *approach_override = nullptr);
    void cancel(const std::string &reason);
    /** Stop without submitting a new RETURN_HOME navigation leg. */
    void abort(const std::string &reason);
    /** Enforce external perception/capture response deadlines. */
    void tick();
    void onNavigationSucceeded(NavigationRole role, const MissionPose &robot);
    /** Called by the low-level navigator after its bounded retry budget. */
    void onNavigationFailed(NavigationRole role, const std::string &reason);
    void onFaceObservation(const FaceObservation &observation);
    void onCaptureResult(const CaptureResult &result);

    /** Built-in mock responses for stage-A closed-loop testing. */
    FaceObservation makeMockFaceObservation(const MissionPose &robot,
                                            const FaceDetectionRequest &request) const;
    CaptureResult makeMockCaptureResult(const CaptureCommand &request) const;

    bool searchApproachPoint(const Eigen::Vector3d &center,
                             const Eigen::Vector3d &normal,
                             Eigen::Vector3d &nav_goal,
                             double preferred_distance =
                                     std::numeric_limits<double>::quiet_NaN()) const;

private:
    void setState(InspectionState state, const std::string &detail = "");
    void publishStatusLocked(const std::string &detail = "") const;
    void failAndReturnHome(const std::string &reason);
    void returnHome(const std::string &detail = "return_home");
    bool requestFaceDetection(const std::string &detail);
    // Returns true when the clipped observation was handled, including a
    // failed recenter submission that has already transitioned to return-home.
    bool recenterForClippedFace(const FaceObservation &observation);
    bool dispatchCurrentViewpoint();
    bool planViewsFromPending(const MissionPose &robot);
    bool commitPendingTarget();
    MissionTarget buildPendingFromFace(const FaceObservation &face) const;
    FaceObservation makePriorFaceObservation() const;

    InspectionMissionConfig cfg_;
    std::shared_ptr<MissionTargetStore> store_;
    std::shared_ptr<coverage::FaceViewpointPlanner> viewpoint_planner_;

    SubmitNavFn submit_nav_;
    PublishFaceRequestFn publish_face_request_;
    PublishCaptureRequestFn publish_capture_request_;
    PublishStatusFn publish_status_;
    PublishViewpointsFn publish_viewpoints_;
    MapProviderFn map_provider_;

    // Recursive: navigation callbacks may re-enter when already at a goal.
    mutable std::recursive_mutex mutex_;
    InspectionState state_{InspectionState::IDLE};
    MissionContext ctx_;
    double state_enter_time_sec_{0.0};
};

}  // namespace mission
