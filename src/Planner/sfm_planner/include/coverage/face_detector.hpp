#pragma once

#include <mission/mission_types.hpp>

#include <deque>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace coverage {

class FaceDetector {
public:
    struct Config {
        double forward_min{1.0};
        double forward_max{12.0};
        double min_confidence{0.75};
        double min_area{4.0};
        int min_points{300};
        double normal_alignment_min{0.8};
        double voxel_leaf{0.1};
        double cluster_tolerance{0.35};
        int cluster_min_size{200};
        double ransac_dist{0.08};
        int stability_frames{3};
        double stability_center_tol{0.35};
        double stability_normal_tol{0.15};
        double lateral_half_width{7.0};
        double vertical_half_height{5.0};
        // A detected face that reaches this close to the ROI boundary is
        // treated as cropped.  The robot must move to a better observation
        // pose or the ROI must be enlarged before coverage can start.
        double roi_edge_margin{0.35};
        // A RANSAC plane is used only as a seed.  This admits moderately
        // curved/rough end-face points while rejecting the distant tunnel
        // walls that meet it at the perimeter.
        double support_plane_distance{0.45};
        double extent_padding{0.25};
        // The mission already has a coarse target from the prior map.  These
        // gates prevent a large but unrelated tunnel wall from replacing it.
        // A non-positive center tolerance or normal threshold disables the
        // corresponding prior check.
        double prior_center_tolerance{3.0};
        double prior_normal_alignment_min{0.9};
    };

    FaceDetector() = default;
    explicit FaceDetector(Config config);

    void setConfig(const Config &config);
    const Config &config() const {
        return cfg_;
    }

    void reset();

    /**
     * Process one frame of world-frame point cloud near the prior face.
     * Returns a valid observation only after multi-frame stability is met.
     */
    mission::FaceObservation process(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
            const Eigen::Vector3d &robot_position,
            const Eigen::Vector3d &tunnel_dir,
            const Eigen::Vector3d *prior_center = nullptr,
            const Eigen::Vector3d *prior_normal = nullptr);

    /** One-shot detection without multi-frame gating (for mock / offline). */
    mission::FaceObservation detectOnce(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
            const Eigen::Vector3d &robot_position,
            const Eigen::Vector3d &tunnel_dir,
            const Eigen::Vector3d *prior_center = nullptr,
            const Eigen::Vector3d *prior_normal = nullptr) const;

private:
    struct Candidate {
        Eigen::Vector3d center{Eigen::Vector3d::Zero()};
        Eigen::Vector3d normal{-Eigen::Vector3d::UnitX()};
        Eigen::Vector3d tangent_u{Eigen::Vector3d::Zero()};
        Eigen::Vector3d tangent_v{Eigen::Vector3d::Zero()};
        double width{0.0};
        double height{0.0};
        double area{0.0};
        double confidence{0.0};
        bool extent_complete{false};
        std::string extent_detail;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    };

    bool extractBestCandidate(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
            const Eigen::Vector3d &robot_position,
            const Eigen::Vector3d &tunnel_dir,
            const Eigen::Vector3d *prior_center,
            const Eigen::Vector3d *prior_normal,
            Candidate &best) const;

    bool isStable(const Candidate &candidate) const;

    Config cfg_;
    mutable std::mutex mutex_;
    std::deque<Candidate> history_;
};

}  // namespace coverage
