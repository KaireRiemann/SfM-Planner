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
        bool unknown_as_occupied{true};
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
