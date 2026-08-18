#pragma once

#include <mission/mission_types.hpp>

#include <functional>

namespace coverage {

/**
 * External face-detector contract used by InspectionMissionPlanner.
 * Detection itself may run in-process or in a separate ROS node.
 */
struct FaceDetectorContract {
    using RequestFn = std::function<void()>;
    using ResultFn = std::function<void(const mission::FaceObservation &)>;

    RequestFn publish_request;
    ResultFn on_result;
};

}  // namespace coverage
