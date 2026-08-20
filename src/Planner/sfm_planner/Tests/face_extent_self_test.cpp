#include <coverage/face_detector.hpp>

#include <cmath>
#include <iostream>

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr makeFaceCloud() {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    // Face normal is -X; the robot observes it from (0, 0, 1.5).  This is a
    // 12 x 8 m end face, deliberately much larger than a local RANSAC patch.
    for (double y = -6.0; y <= 6.0 + 1e-9; y += 0.2) {
        for (double z = -2.5; z <= 5.5 + 1e-9; z += 0.2) {
            cloud->push_back(pcl::PointXYZ(3.5F,
                                            static_cast<float>(y),
                                            static_cast<float>(z)));
        }
    }
    return cloud;
}

coverage::FaceDetector::Config detectorConfig() {
    coverage::FaceDetector::Config cfg;
    cfg.forward_min = 1.0;
    cfg.forward_max = 5.0;
    cfg.min_confidence = 0.5;
    cfg.min_area = 4.0;
    cfg.min_points = 100;
    cfg.normal_alignment_min = 0.8;
    cfg.voxel_leaf = 0.05;
    cfg.cluster_tolerance = 0.35;
    cfg.cluster_min_size = 100;
    cfg.ransac_dist = 0.05;
    cfg.stability_frames = 1;
    cfg.vertical_half_height = 5.0;
    cfg.support_plane_distance = 0.2;
    cfg.extent_padding = 0.25;
    return cfg;
}

bool require(const bool condition, const char *message) {
    if (!condition) {
        std::cerr << "face_extent_self_test: " << message << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main() {
    const Eigen::Vector3d robot(0.0, 0.0, 1.5);
    const Eigen::Vector3d tunnel_dir = Eigen::Vector3d::UnitX();
    const auto cloud = makeFaceCloud();

    auto complete_cfg = detectorConfig();
    complete_cfg.lateral_half_width = 7.0;
    coverage::FaceDetector complete_detector(complete_cfg);
    const auto complete = complete_detector.detectOnce(cloud, robot, tunnel_dir);
    if (!require(complete.valid, "complete face was rejected") ||
        !require(complete.extent_complete, "complete face is not marked complete") ||
        !require(complete.width >= 12.35, "full horizontal extent was not retained") ||
        !require(complete.height >= 8.35, "full vertical extent was not retained") ||
        !require(std::abs(complete.tangent_u.dot(complete.normal)) < 1e-6,
                 "tangent_u is not in the face plane") ||
        !require(std::abs(complete.tangent_v.dot(complete.normal)) < 1e-6,
                 "tangent_v is not in the face plane")) {
        return 1;
    }

    auto cropped_cfg = detectorConfig();
    cropped_cfg.lateral_half_width = 6.0;
    coverage::FaceDetector cropped_detector(cropped_cfg);
    const auto cropped = cropped_detector.detectOnce(cloud, robot, tunnel_dir);
    if (!require(!cropped.valid, "ROI-cropped face was accepted") ||
        !require(!cropped.extent_complete, "ROI-cropped face is marked complete") ||
        !require(cropped.extent_detail == "face_extent_clipped",
                 "ROI-cropped face did not report its reason")) {
        return 1;
    }

    std::cout << "face_extent_self_test: PASS" << std::endl;
    return 0;
}
