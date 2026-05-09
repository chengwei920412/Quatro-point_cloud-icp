#ifndef QUATRO_PCL_COMPAT_HPP
#define QUATRO_PCL_COMPAT_HPP

// PCL smart-pointer-version-agnostic utilities for Quatro.
//
// PCL 1.10 (Ubuntu 20.04) uses boost::shared_ptr; PCL 1.11+ uses
// pcl::shared_ptr (= std::shared_ptr). Function signatures in this
// header MUST take typename pcl::PointCloud<T>::Ptr / ConstPtr and
// never boost::shared_ptr<pcl::PointCloud<T>> directly, so they
// remain correct on every supported PCL version.

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

namespace quatro {

template <typename PointT>
void voxelize(const typename pcl::PointCloud<PointT>::ConstPtr& src,
              const typename pcl::PointCloud<PointT>::Ptr& dst,
              double voxel_size) {
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(src);
    filter.setLeafSize(static_cast<float>(voxel_size),
                       static_cast<float>(voxel_size),
                       static_cast<float>(voxel_size));
    filter.filter(*dst);
}

template <typename PointT>
void voxelize(const pcl::PointCloud<PointT>& src,
              const typename pcl::PointCloud<PointT>::Ptr& dst,
              double voxel_size) {
    auto src_ptr = typename pcl::PointCloud<PointT>::Ptr(
        new pcl::PointCloud<PointT>(src));
    voxelize<PointT>(src_ptr, dst, voxel_size);
}

}  // namespace quatro

#endif  // QUATRO_PCL_COMPAT_HPP
