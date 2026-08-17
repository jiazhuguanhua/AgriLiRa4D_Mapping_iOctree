#include "mapper/mapper_ioctree.h"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <limits>
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
  if (!std::isfinite(options_.occupancy_resolution) ||
      options_.occupancy_resolution <= 0.0) {
    throw std::invalid_argument(
        "i-Octree occupancy_resolution must be positive");
  }
  if (options_.min_points_per_voxel == 0) {
    throw std::invalid_argument(
        "i-Octree min_points_per_voxel must be positive");
  }
  if (!std::isfinite(options_.occupancy_threshold) ||
      options_.occupancy_threshold < 0.5 ||
      options_.occupancy_threshold > 1.0) {
    throw std::invalid_argument(
        "i-Octree occupancy_threshold must be in [0.5, 1]");
  }
  if (!std::isfinite(options_.probability_scale) ||
      options_.probability_scale <= 0.0) {
    throw std::invalid_argument(
        "i-Octree probability_scale must be positive");
  }

  ioctree_ = MakeTree();
}

std::unique_ptr<thuni::Octree> IOctreeMapper::MakeTree() const {
  auto tree = std::make_unique<thuni::Octree>(
      options_.bucket_size, false, static_cast<float>(options_.min_extent));
  tree->set_down_size(options_.downsample);
  return tree;
}

std::size_t
IOctreeMapper::VoxelKeyHash::operator()(const VoxelKey &key) const {
  std::size_t seed = std::hash<std::int64_t>{}(key.x);
  seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) +
          (seed >> 2U);
  seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) +
          (seed >> 2U);
  return seed;
}

IOctreeMapper::VoxelKey
IOctreeMapper::PointToVoxel(const PointType &point) const {
  return {static_cast<std::int64_t>(
              std::floor(static_cast<double>(point.x) /
                         options_.occupancy_resolution)),
          static_cast<std::int64_t>(
              std::floor(static_cast<double>(point.y) /
                         options_.occupancy_resolution)),
          static_cast<std::int64_t>(
              std::floor(static_cast<double>(point.z) /
                         options_.occupancy_resolution))};
}

float IOctreeMapper::PointCountToProbability(
    const std::uint32_t point_count) const {
  return static_cast<float>(
      1.0 - std::exp(-static_cast<double>(point_count) /
                     options_.probability_scale));
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

  for (const auto &point : accepted) {
    auto &count = voxel_point_counts_[PointToVoxel(point)];
    if (count != std::numeric_limits<std::uint32_t>::max())
      ++count;
  }
}

void IOctreeMapper::Reset() {
  // Upstream Octree::clear() does not reset all point counters. Reconstructing
  // the object gives Reset() the expected clean-state semantics.
  ioctree_ = MakeTree();
  voxel_point_counts_.clear();
}

bool IOctreeMapper::GetOccupiedVoxels(OccupancyMap &map) const {
  map.resolution = options_.occupancy_resolution;
  map.voxels.clear();
  map.voxels.reserve(voxel_point_counts_.size());

  for (const auto &[key, point_count] : voxel_point_counts_) {
    if (point_count < options_.min_points_per_voxel)
      continue;

    const float probability = PointCountToProbability(point_count);
    if (probability < options_.occupancy_threshold)
      continue;

    OccupiedVoxel voxel;
    voxel.x = (static_cast<double>(key.x) + 0.5) * map.resolution;
    voxel.y = (static_cast<double>(key.y) + 0.5) * map.resolution;
    voxel.z = (static_cast<double>(key.z) + 0.5) * map.resolution;
    voxel.probability = probability;
    voxel.point_count = point_count;
    map.voxels.push_back(voxel);
  }

  std::sort(map.voxels.begin(), map.voxels.end(),
            [](const OccupiedVoxel &lhs, const OccupiedVoxel &rhs) {
              if (lhs.x != rhs.x)
                return lhs.x < rhs.x;
              if (lhs.y != rhs.y)
                return lhs.y < rhs.y;
              return lhs.z < rhs.z;
            });
  return !map.voxels.empty();
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
