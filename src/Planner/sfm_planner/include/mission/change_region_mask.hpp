#pragma once

#include <mission/mission_types.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map_manager/map_manager.hpp>

namespace mission {

/** Remove points that lie inside the oriented change-region slab. */
template <typename PointT>
inline std::size_t filterCloudByChangeRegion(
        const typename pcl::PointCloud<PointT>::ConstPtr &input,
        const ChangeRegion &region,
        typename pcl::PointCloud<PointT>::Ptr &output) {
    output.reset(new pcl::PointCloud<PointT>());
    if (!input) {
        return 0;
    }
    output->reserve(input->size());
    std::size_t removed = 0;
    for (const auto &pt : input->points) {
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        if (pointInChangeRegion(p, region)) {
            ++removed;
            continue;
        }
        output->push_back(pt);
    }
    return removed;
}

inline int applyChangeRegionMask(const general_planner::MapManager::Ptr &map,
                                 const ChangeRegion &region) {
    if (!map || !map->ready() || !region.valid) {
        return 0;
    }
    return map->maskChangeRegion(
            rog_map::Vec3f(region.center.x(), region.center.y(), region.center.z()),
            rog_map::Vec3f(region.normal.x(), region.normal.y(), region.normal.z()),
            region.width,
            region.height,
            region.thickness);
}

}  // namespace mission
