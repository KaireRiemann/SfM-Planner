#pragma once

#include <mission/mission_types.hpp>

#include <map_manager/map_manager.hpp>

namespace coverage {

struct CameraModel {
    double hfov_deg{70.0};
    double vfov_deg{50.0};
    double capture_distance{4.0};
    double image_overlap{0.7};
    double max_pitch_deg{35.0};
    double min_pitch_deg{-35.0};
    double max_incidence_angle_deg{60.0};
};

class FaceViewpointPlanner {
public:
    struct Config {
        CameraModel camera;
        int min_observation_count{2};
        double min_baseline_angle_deg{8.0};
        double surface_sample_resolution{0.4};
        double safe_radius{0.6};
        double flight_height_min{0.5};
        double flight_height_max{5.0};
        // Capture poses can use a narrower, dynamics-validated height band
        // than the global navigation workspace; camera pitch covers the
        // remaining vertical face extent.
        double viewpoint_height_min{0.5};
        double viewpoint_height_max{5.0};
        // Optional half-width of the camera flight corridor in the face's
        // lateral tangent direction.  Non-positive disables the limit.
        double viewpoint_lateral_limit{0.0};
        bool unknown_as_occupied{true};
        // The raw lidar map is useful for camera-pose collision safety, but
        // may contain the detected end-face itself as occupied.  Enable this
        // only when it is also a trustworthy optical occlusion map.
        bool use_raw_occlusion_check{true};
        // A complete primary (one-view) cover can be accepted when only
        // boundary samples lack a second safe-baseline observation.  This is
        // opt-in; normal SfM operation keeps strict K-view coverage.
        bool allow_single_view_boundary_coverage{false};
        // A complete face rectangle is accepted only if every coverage grid
        // sample reaches the requested multi-view observation count.
        double required_coverage_ratio{1.0};
        // Upper bound on capture poses selected for a single face.  It must
        // be high enough to satisfy K-coverage for large tunnel faces.
        int max_viewpoints{60};
    };

    FaceViewpointPlanner() = default;
    explicit FaceViewpointPlanner(Config config);

    void setConfig(const Config &config);
    const Config &config() const {
        return cfg_;
    }

    mission::CoveragePlan plan(const mission::FaceObservation &face,
                               const general_planner::MapManager &map,
                               const Eigen::Vector3d &current_position,
                               const Eigen::Vector3d &home) const;

private:
    Config cfg_;
};

}  // namespace coverage
