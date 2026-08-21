#include <mission/inspection_mission_planner.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

struct NavigationRequest {
    mission::MissionPose goal;
    mission::NavigationRole role{mission::NavigationRole::EXTERNAL_CLICK};
};

bool require(const bool condition, const char *message) {
    if (!condition) {
        std::cerr << "inspection_face_recenter_self_test: " << message << std::endl;
        return false;
    }
    return true;
}

bool near(const double lhs, const double rhs, const double tolerance = 1e-6) {
    return std::abs(lhs - rhs) <= tolerance;
}

bool writeTarget(const std::string &path) {
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << "scene_id: face_recenter_self_test\n"
           "target_version: 0\n"
           "map_version: self_test\n"
           "face_center: [4.0, 1.0, 2.0]\n"
           "face_normal: [-1.0, 0.0, 0.0]\n"
           "face_prior_valid: false\n"
           "nav_goal: [0.0, 0.0, 1.5]\n"
           "goal_yaw: 0.0\n"
           "confidence: 0.0\n"
           "change_region:\n"
           "  center: [4.0, 1.0, 2.0]\n"
           "  normal: [-1.0, 0.0, 0.0]\n"
           "  width: 0.0\n"
           "  height: 0.0\n"
           "  thickness: 1.0\n";
    return out.good();
}

mission::FaceObservation clippedObservation(const mission::FaceDetectionRequest &request) {
    mission::FaceObservation observation;
    observation.mission_id = request.mission_id;
    observation.target_version = request.target_version;
    observation.request_id = request.request_id;
    observation.valid = false;
    observation.center = Eigen::Vector3d(4.0, 1.0, 2.0);
    observation.normal = -Eigen::Vector3d::UnitX();
    observation.width = 12.0;
    observation.height = 8.0;
    observation.area = 96.0;
    observation.confidence = 0.95;
    observation.extent_complete = false;
    observation.extent_detail = "face_extent_clipped";
    return observation;
}

}  // namespace

int main() {
    const std::string target_path = "/tmp/sfm_planner_face_recenter_self_test.yaml";
    std::remove(target_path.c_str());
    if (!require(writeTarget(target_path), "could not create target fixture")) {
        return 1;
    }

    mission::InspectionMissionConfig cfg;
    cfg.mock_external = false;
    cfg.mock_face_detection = false;
    cfg.mock_capture = false;
    cfg.face_recenter_on_clipped = true;
    cfg.face_recenter_max_attempts = 1;
    cfg.approach_distance_min = 2.0;
    cfg.approach_distance_max = 4.0;
    cfg.approach_distance_step = 0.2;
    cfg.flight_height_min = 0.5;
    cfg.flight_height_max = 5.0;
    auto store = std::make_shared<mission::MissionTargetStore>(target_path);
    mission::InspectionMissionPlanner planner(cfg, store);

    std::vector<NavigationRequest> navigation_requests;
    std::vector<mission::FaceDetectionRequest> face_requests;
    planner.setCallbacks(
            [&navigation_requests](const mission::MissionPose &goal,
                                   const mission::NavigationRole role) {
                navigation_requests.push_back({goal, role});
                return true;
            },
            [&face_requests](const mission::FaceDetectionRequest &request) {
                face_requests.push_back(request);
            },
            [](const mission::CaptureCommand &) {},
            [](const mission::MissionStatusInfo &) {},
            [](const mission::FaceObservation &, const mission::CoveragePlan &) {},
            []() { return general_planner::MapManager::Ptr{}; });

    mission::MissionPose home;
    home.position = Eigen::Vector3d(0.0, 0.0, 1.5);
    home.yaw = 0.0;
    if (!require(planner.start(home, "face_recenter"), "mission did not start") ||
        !require(navigation_requests.size() == 1,
                 "initial approach was not submitted") ||
        !require(navigation_requests.back().role == mission::NavigationRole::APPROACH_TARGET,
                 "initial approach has the wrong role")) {
        std::remove(target_path.c_str());
        return 1;
    }

    planner.onNavigationSucceeded(mission::NavigationRole::APPROACH_TARGET, home);
    if (!require(planner.state() == mission::InspectionState::WAIT_FACE_RESULT,
                 "approach did not request detection") ||
        !require(face_requests.size() == 1, "initial detection request missing")) {
        std::remove(target_path.c_str());
        return 1;
    }

    planner.onFaceObservation(clippedObservation(face_requests.back()));
    const auto after_clipped = planner.context();
    if (!require(planner.state() == mission::InspectionState::GO_TO_FACE_RECENTER,
                 "cropped face did not enter recenter navigation") ||
        !require(after_clipped.face_recenter_attempts == 1,
                 "recenter attempt was not recorded") ||
        !require(navigation_requests.size() == 2,
                 "recenter navigation was not submitted") ||
        !require(navigation_requests.back().role == mission::NavigationRole::FACE_RECENTER,
                 "recenter navigation has the wrong role") ||
        !require(near(navigation_requests.back().goal.position.x(), 0.0) &&
                         near(navigation_requests.back().goal.position.y(), 1.0) &&
                         near(navigation_requests.back().goal.position.z(), 2.0),
                 "recenter goal is not face-centred along the free-space normal")) {
        std::remove(target_path.c_str());
        return 1;
    }

    planner.onNavigationSucceeded(mission::NavigationRole::FACE_RECENTER,
                                  navigation_requests.back().goal);
    if (!require(planner.state() == mission::InspectionState::WAIT_FACE_RESULT,
                 "recenter arrival did not request a fresh detection") ||
        !require(face_requests.size() == 2,
                 "fresh detection request missing after recenter") ||
        !require(face_requests.front().request_id != face_requests.back().request_id,
                 "recenter reused the stale detection request")) {
        std::remove(target_path.c_str());
        return 1;
    }

    // A second crop exhausts the bounded retry budget and preserves the
    // original fail-safe return-home behaviour.
    planner.onFaceObservation(clippedObservation(face_requests.back()));
    if (!require(planner.state() == mission::InspectionState::RETURN_HOME,
                 "second cropped face did not return home") ||
        !require(navigation_requests.size() == 3,
                 "return-home navigation was not submitted") ||
        !require(navigation_requests.back().role == mission::NavigationRole::HOME,
                 "bounded recenter failure used the wrong navigation role")) {
        std::remove(target_path.c_str());
        return 1;
    }

    // A failed HOME leg is terminal. The FSM is responsible for clearing its
    // queued goal after this callback; the mission itself must neither submit
    // a duplicate HOME request nor remain active.
    const std::size_t requests_before_home_failure = navigation_requests.size();
    planner.onNavigationFailed(mission::NavigationRole::HOME,
                               "home_planning_failed");
    if (!require(planner.state() == mission::InspectionState::FAILED,
                 "failed home leg did not become terminal") ||
        !require(!planner.active(), "failed home leg left mission active") ||
        !require(navigation_requests.size() == requests_before_home_failure,
                 "failed home leg submitted a duplicate navigation request")) {
        std::remove(target_path.c_str());
        return 1;
    }

    std::remove(target_path.c_str());
    std::cout << "inspection_face_recenter_self_test: PASS" << std::endl;
    return 0;
}
