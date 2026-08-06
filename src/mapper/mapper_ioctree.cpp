#include "mapper/mapper_ioctree.h"

#include <pcl/io/pcd_io.h>

#include <cmath>
#include <stdexcept>

namespace mapping {

IOctreeMapper::IOctreeMapper(const Options &options) : options_(options) {
  if (!std::isfinite(options_.min_extent) || options_.min_extent <= 0.0) {
    throw std::invalid_argument("i-Octree min_extent must be positive");
  }
  if (options_.bucket_size == 0) {
    throw std::invalid_argument("i-Octree bucket_size must be positive");
  }
  if (!std::isfinite(options_.max_range) || options_.max_range <= 0.0) {
    throw std::invalid_argument("i-Octree max_range must be positive");
  }

  ioctree_ = MakeTree();
}

std::unique_ptr<thuni::Octree> IOctreeMapper::MakeTree() const {
  auto tree = std::make_unique<thuni::Octree>(
      options_.bucket_size, false, static_cast<float>(options_.min_extent));
  tree->set_down_size(options_.downsample);
  return tree;
}

void IOctreeMapper::Update(const MapperInput &input) {
  if (!input.lidar_cloud || input.lidar_cloud->empty()) {
    ROS_WARN_THROTTLE(1.0, "IOctreeMapper: input has no LiDAR data.");
    return;
  }

  if (!IsValidPoint(input.lidar_origin.x(), input.lidar_origin.y(),
                    input.lidar_origin.z())) {
    ROS_WARN_THROTTLE(1.0, "IOctreeMapper: invalid LiDAR origin.");
    return;
  }

  PointCloudType accepted;
  accepted.reserve(input.lidar_cloud->size());
  const double max_range_sq = options_.max_range * options_.max_range;

  for (const auto &point : input.lidar_cloud->points) {
    if (!IsValidPoint(point.x, point.y, point.z))
      continue;

    const double dx = static_cast<double>(point.x) - input.lidar_origin.x();
    const double dy = static_cast<double>(point.y) - input.lidar_origin.y();
    const double dz = static_cast<double>(point.z) - input.lidar_origin.z();
    if (dx * dx + dy * dy + dz * dz > max_range_sq)
      continue;

    accepted.push_back(point);
  }

  if (accepted.empty()) {
    ROS_WARN_THROTTLE(1.0,
                      "IOctreeMapper: no valid LiDAR points within range.");
    return;
  }

  ioctree_->update(accepted, options_.downsample);
}

void IOctreeMapper::Reset() {
  // Upstream Octree::clear() does not reset all point counters. Reconstructing
  // the object gives Reset() the expected clean-state semantics.
  ioctree_ = MakeTree();
}

bool IOctreeMapper::GetMapCloud(CloudPtr &cloud) const {
  cloud.reset(new PointCloudType);
  if (!ioctree_)
    return false;

  cloud->points = ioctree_->get_data<PointType, PointVector>();
  for (auto &point : cloud->points) {
    // i-Octree stores XYZ only; initialize the remaining PCL fields so the
    // published PointCloud2 never contains indeterminate values.
    point.intensity = 1.0f;
    point.normal_x = 0.0f;
    point.normal_y = 0.0f;
    point.normal_z = 0.0f;
    point.curvature = 0.0f;
  }
  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
  return !cloud->empty();
}

bool IOctreeMapper::Save(const std::string &path) const {
  if (path.empty()) {
    ROS_ERROR("IOctreeMapper: save path is empty.");
    return false;
  }

  CloudPtr cloud;
  if (!GetMapCloud(cloud)) {
    ROS_WARN("IOctreeMapper: cannot save an empty map.");
    return false;
  }

  if (pcl::io::savePCDFileBinary(path, *cloud) != 0) {
    ROS_ERROR("IOctreeMapper: failed to save PCD map to %s", path.c_str());
    return false;
  }
  return true;
}

} // namespace mapping
