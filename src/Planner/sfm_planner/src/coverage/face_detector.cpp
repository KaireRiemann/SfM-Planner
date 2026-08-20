#include <coverage/face_detector.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace coverage {
namespace {

Eigen::Vector3d ensureNormalTowardRobot(Eigen::Vector3d normal,
                                        const Eigen::Vector3d &center,
                                        const Eigen::Vector3d &robot) {
    if (normal.norm() < 1e-6) {
        normal = (robot - center);
    }
    if (normal.norm() < 1e-6) {
        return -Eigen::Vector3d::UnitX();
    }
    normal.normalize();
    if (normal.dot(robot - center) < 0.0) {
        normal = -normal;
    }
    return normal;
}

double planeResidualScore(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                          const Eigen::Vector4f &coeffs) {
    if (!cloud || cloud->empty()) {
        return 1e3;
    }
    double sum = 0.0;
    const Eigen::Vector3d n(coeffs[0], coeffs[1], coeffs[2]);
    const double inv_n = 1.0 / std::max(1e-6, n.norm());
    for (const auto &pt : cloud->points) {
        const double d =
                std::abs(coeffs[0] * pt.x + coeffs[1] * pt.y + coeffs[2] * pt.z +
                         coeffs[3]) *
                inv_n;
        sum += d;
    }
    return sum / static_cast<double>(cloud->size());
}

void buildFaceFrame(const Eigen::Vector3d &normal_in,
                    Eigen::Vector3d &normal,
                    Eigen::Vector3d &u,
                    Eigen::Vector3d &v) {
    normal = normal_in;
    if (normal.norm() < 1e-6) {
        normal = -Eigen::Vector3d::UnitX();
    }
    normal.normalize();
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    if (std::abs(normal.dot(up)) > 0.95) {
        up = Eigen::Vector3d::UnitY();
    }
    u = (up.cross(normal)).normalized();
    v = normal.cross(u).normalized();
}

}  // namespace

FaceDetector::FaceDetector(Config config) : cfg_(std::move(config)) {}

void FaceDetector::setConfig(const Config &config) {
    std::lock_guard<std::mutex> lock(mutex_);
    cfg_ = config;
    history_.clear();
}

void FaceDetector::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    history_.clear();
}

bool FaceDetector::extractBestCandidate(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
        const Eigen::Vector3d &robot_position,
        const Eigen::Vector3d &tunnel_dir,
        const Eigen::Vector3d *prior_center,
        const Eigen::Vector3d *prior_normal,
        Candidate &best) const {
    if (!cloud || cloud->empty()) {
        return false;
    }

    Eigen::Vector3d dir = tunnel_dir;
    if (dir.norm() < 1e-6) {
        dir = Eigen::Vector3d::UnitX();
    }
    dir.normalize();

    pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
    roi->reserve(cloud->size());
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    if (std::abs(dir.dot(up)) > 0.95) {
        up = Eigen::Vector3d::UnitY();
    }
    const Eigen::Vector3d u = (up.cross(dir)).normalized();
    const Eigen::Vector3d v = dir.cross(u);

    for (const auto &pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        const Eigen::Vector3d d = p - robot_position;
        const double forward = d.dot(dir);
        if (forward < cfg_.forward_min || forward > cfg_.forward_max) {
            continue;
        }
        if (std::abs(d.dot(u)) > cfg_.lateral_half_width) {
            continue;
        }
        if (std::abs(d.dot(v)) > cfg_.vertical_half_height) {
            continue;
        }
        roi->push_back(pt);
    }
    if (static_cast<int>(roi->size()) < cfg_.min_points) {
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (cfg_.voxel_leaf > 1e-4) {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(roi);
        vg.setLeafSize(static_cast<float>(cfg_.voxel_leaf),
                       static_cast<float>(cfg_.voxel_leaf),
                       static_cast<float>(cfg_.voxel_leaf));
        vg.filter(*filtered);
    } else {
        *filtered = *roi;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cleaned);
    if (static_cast<int>(cleaned->size()) < cfg_.min_points) {
        return false;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cleaned);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cfg_.cluster_tolerance);
    ec.setMinClusterSize(std::max(50, cfg_.cluster_min_size));
    ec.setMaxClusterSize(static_cast<int>(cleaned->size()));
    ec.setSearchMethod(tree);
    ec.setInputCloud(cleaned);
    ec.extract(cluster_indices);
    if (cluster_indices.empty()) {
        return false;
    }

    double best_score = -std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto &indices : cluster_indices) {
        if (static_cast<int>(indices.indices.size()) < cfg_.min_points) {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        cluster->reserve(indices.indices.size());
        for (const int idx : indices.indices) {
            cluster->push_back(cleaned->points[static_cast<std::size_t>(idx)]);
        }

        pcl::ModelCoefficients coeffs;
        pcl::PointIndices inliers;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(cfg_.ransac_dist);
        seg.setInputCloud(cluster);
        seg.segment(inliers, coeffs);
        if (inliers.indices.size() < static_cast<std::size_t>(cfg_.min_points) ||
            coeffs.values.size() < 4) {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(inliers));
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cluster);
        extract.setIndices(inliers_ptr);
        extract.setNegative(false);
        extract.filter(*plane_cloud);
        if (static_cast<int>(plane_cloud->size()) < cfg_.min_points) {
            continue;
        }

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane_cloud, centroid);
        Eigen::Vector3d center(centroid[0], centroid[1], centroid[2]);
        Eigen::Vector3d normal =
                ensureNormalTowardRobot(Eigen::Vector3d(coeffs.values[0], coeffs.values[1],
                                                        coeffs.values[2]),
                                        center,
                                        robot_position);

        // Prefer faces whose normal aligns with -tunnel_dir (face looks back to free space).
        const double alignment = normal.dot((-dir).normalized());
        if (alignment < cfg_.normal_alignment_min) {
            continue;
        }
        if (prior_normal && prior_normal->norm() > 1e-6 &&
            cfg_.prior_normal_alignment_min > 0.0) {
            const Eigen::Vector3d expected_normal = prior_normal->normalized();
            if (normal.dot(expected_normal) < cfg_.prior_normal_alignment_min) {
                continue;
            }
        }

        // RANSAC only establishes a reliable local normal.  The old code used
        // its inliers as the entire face, which reports high coverage for a
        // small flat patch on a much larger rough/curved tunnel end.  Grow the
        // support along that normal from the pre-outlier-filtered ROI, then
        // derive the rectangle from all accepted end-face points.  In
        // particular, SOR is allowed to trim sparse boundary points for the
        // RANSAC seed but must not shrink the measured face extent.
        Eigen::Vector3d plane_normal(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
        const double plane_normal_norm = plane_normal.norm();
        if (plane_normal_norm < 1e-6) {
            continue;
        }
        plane_normal /= plane_normal_norm;
        const double plane_offset = coeffs.values[3] / plane_normal_norm;
        pcl::PointCloud<pcl::PointXYZ>::Ptr support_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
        support_cloud->reserve(filtered->size());
        const double support_distance = std::max(cfg_.ransac_dist,
                                                 cfg_.support_plane_distance);
        for (const auto &pt : filtered->points) {
            const Eigen::Vector3d p(pt.x, pt.y, pt.z);
            if (std::abs(plane_normal.dot(p) + plane_offset) <= support_distance) {
                support_cloud->push_back(pt);
            }
        }
        if (static_cast<int>(support_cloud->size()) < cfg_.min_points) {
            continue;
        }

        Eigen::Vector3d frame_normal;
        Eigen::Vector3d face_u;
        Eigen::Vector3d face_v;
        buildFaceFrame(normal, frame_normal, face_u, face_v);
        double min_u = std::numeric_limits<double>::infinity();
        double max_u = -std::numeric_limits<double>::infinity();
        double min_v = std::numeric_limits<double>::infinity();
        double max_v = -std::numeric_limits<double>::infinity();
        bool roi_edge_touched = false;
        const double lateral_edge = std::max(
                0.0, cfg_.lateral_half_width - std::max(0.0, cfg_.roi_edge_margin));
        const double vertical_edge = std::max(
                0.0, cfg_.vertical_half_height - std::max(0.0, cfg_.roi_edge_margin));
        for (const auto &pt : support_cloud->points) {
            const Eigen::Vector3d p(pt.x, pt.y, pt.z);
            const Eigen::Vector3d relative = p - center;
            const double pu = relative.dot(face_u);
            const double pv = relative.dot(face_v);
            min_u = std::min(min_u, pu);
            max_u = std::max(max_u, pu);
            min_v = std::min(min_v, pv);
            max_v = std::max(max_v, pv);
            const Eigen::Vector3d robot_relative = p - robot_position;
            if (std::abs(robot_relative.dot(u)) >= lateral_edge ||
                std::abs(robot_relative.dot(v)) >= vertical_edge) {
                roi_edge_touched = true;
            }
        }
        const double padding = std::max(0.0, cfg_.extent_padding);
        const double width = std::max(0.0, max_u - min_u) + 2.0 * padding;
        const double height = std::max(0.0, max_v - min_v) + 2.0 * padding;
        const double area = width * height;
        if (area < cfg_.min_area) {
            continue;
        }

        // Center of the conservative, world-up-aligned rectangle.  The same
        // frame is carried forward to coverage planning and RViz.
        const Eigen::Vector3d center_uv =
                center + 0.5 * (min_u + max_u) * face_u +
                0.5 * (min_v + max_v) * face_v;
        if (prior_center && cfg_.prior_center_tolerance > 0.0 &&
            (center_uv - *prior_center).norm() > cfg_.prior_center_tolerance) {
            continue;
        }

        const double residual = planeResidualScore(
                plane_cloud,
                Eigen::Vector4f(coeffs.values[0], coeffs.values[1], coeffs.values[2],
                                coeffs.values[3]));
        const double density =
                static_cast<double>(support_cloud->size()) / std::max(1e-3, area);
        const double forward = (center_uv - robot_position).dot(dir);
        const double confidence = std::clamp(
                0.35 * alignment + 0.25 * std::min(1.0, area / std::max(1.0, cfg_.min_area)) +
                        0.20 * std::min(1.0, density / 30.0) +
                        0.20 * std::max(0.0, 1.0 - residual / 0.2),
                0.0,
                1.0);
        if (confidence < cfg_.min_confidence) {
            continue;
        }

        // A complete candidate always beats an otherwise comparable ROI-cropped
        // one.  If every candidate is cropped we still return the best one so
        // the mission can emit `face_extent_clipped` rather than timing out.
        const double score = confidence * 2.0 + alignment + 0.05 * area +
                             0.01 * forward + (roi_edge_touched ? 0.0 : 10.0);
        if (score > best_score) {
            best_score = score;
            best.center = center_uv;
            best.normal = frame_normal;
            best.tangent_u = face_u;
            best.tangent_v = face_v;
            best.width = width;
            best.height = height;
            best.area = area;
            best.confidence = confidence;
            best.extent_complete = !roi_edge_touched;
            best.extent_detail = roi_edge_touched
                                         ? "face_extent_clipped"
                                         : "complete_face_extent";
            best.cloud = support_cloud;
            found = true;
        }
    }
    return found;
}

bool FaceDetector::isStable(const Candidate &candidate) const {
    if (cfg_.stability_frames <= 1) {
        return true;
    }
    if (static_cast<int>(history_.size()) + 1 < cfg_.stability_frames) {
        return false;
    }
    int stable = 1;
    for (auto it = history_.rbegin();
         it != history_.rend() && stable < cfg_.stability_frames;
         ++it) {
        if ((it->center - candidate.center).norm() > cfg_.stability_center_tol) {
            break;
        }
        if (it->normal.dot(candidate.normal) < (1.0 - cfg_.stability_normal_tol)) {
            break;
        }
        ++stable;
    }
    return stable >= cfg_.stability_frames;
}

mission::FaceObservation FaceDetector::detectOnce(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
        const Eigen::Vector3d &robot_position,
        const Eigen::Vector3d &tunnel_dir,
        const Eigen::Vector3d *prior_center,
        const Eigen::Vector3d *prior_normal) const {
    Candidate best;
    mission::FaceObservation obs;
    if (!extractBestCandidate(cloud, robot_position, tunnel_dir, prior_center, prior_normal,
                              best)) {
        return obs;
    }
    obs.center = best.center;
    obs.normal = best.normal;
    obs.tangent_u = best.tangent_u;
    obs.tangent_v = best.tangent_v;
    obs.width = best.width;
    obs.height = best.height;
    obs.area = best.area;
    obs.confidence = best.confidence;
    obs.surface_cloud = best.cloud;
    obs.extent_complete = best.extent_complete;
    obs.extent_detail = best.extent_detail;
    obs.valid = best.extent_complete;
    return obs;
}

mission::FaceObservation FaceDetector::process(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
        const Eigen::Vector3d &robot_position,
        const Eigen::Vector3d &tunnel_dir,
        const Eigen::Vector3d *prior_center,
        const Eigen::Vector3d *prior_normal) {
    Candidate best;
    mission::FaceObservation obs;
    if (!extractBestCandidate(cloud, robot_position, tunnel_dir, prior_center, prior_normal,
                              best)) {
        return obs;
    }

    if (!best.extent_complete) {
        obs.center = best.center;
        obs.normal = best.normal;
        obs.tangent_u = best.tangent_u;
        obs.tangent_v = best.tangent_v;
        obs.width = best.width;
        obs.height = best.height;
        obs.area = best.area;
        obs.confidence = best.confidence;
        obs.surface_cloud = best.cloud;
        obs.extent_complete = false;
        obs.extent_detail = best.extent_detail;
        return obs;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    // Test against *previous* observations first.  Pushing before this check
    // counted the current scan twice, so a three-frame setting could pass
    // after only two physical scans.
    if (!isStable(best)) {
        history_.push_back(best);
        while (static_cast<int>(history_.size()) > std::max(1, cfg_.stability_frames)) {
            history_.pop_front();
        }
        return obs;
    }

    history_.push_back(best);
    while (static_cast<int>(history_.size()) > std::max(1, cfg_.stability_frames)) {
        history_.pop_front();
    }

    obs.valid = true;
    obs.center = best.center;
    obs.normal = best.normal;
    obs.tangent_u = best.tangent_u;
    obs.tangent_v = best.tangent_v;
    obs.width = best.width;
    obs.height = best.height;
    obs.area = best.area;
    obs.confidence = best.confidence;
    obs.surface_cloud = best.cloud;
    obs.extent_complete = true;
    obs.extent_detail = best.extent_detail;
    return obs;
}

}  // namespace coverage
