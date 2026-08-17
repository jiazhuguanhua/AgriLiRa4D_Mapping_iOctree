/**
 * @file mapper_ioctree.h
 * @brief Incremental point-cloud map backed by i-Octree.
 */

#ifndef IOCTREE_MAPPER_H
#define IOCTREE_MAPPER_H

#include "mapper/mapper.h"

#include <Octree.h>

#include <cstdint>
#include <unordered_map>

namespace mapping {

class IOctreeMapper : public Mapper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options {
    Options() {}
    double min_extent = 0.25;
    std::size_t bucket_size = 32;
    bool downsample = true;
    double max_range = 50.0;
    double occupancy_resolution = 0.5;
    std::uint32_t min_points_per_voxel = 1;
    double occupancy_threshold = 0.5;
    double probability_scale = 1.0;
  };

  explicit IOctreeMapper(const Options &options = Options());

  void Update(const MapperInput &input) override;
  void Reset() override;
  bool Save(const std::string &path) const override;
  bool GetOccupiedVoxels(OccupancyMap &map) const override;

private:
  struct VoxelKey {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const VoxelKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey &key) const;
  };

  std::unique_ptr<thuni::Octree> MakeTree() const;
  VoxelKey PointToVoxel(const PointType &point) const;
  float PointCountToProbability(std::uint32_t point_count) const;
  bool GetMapCloud(CloudPtr &cloud) const;

  Options options_;
  std::unordered_map<VoxelKey, std::uint32_t, VoxelKeyHash> voxel_point_counts_;
  // The upstream export API is not const-qualified. Mapping calls Update and
  // snapshot export from the same worker thread, so mutable is safe here.
  mutable std::unique_ptr<thuni::Octree> ioctree_;
};

} // namespace mapping

#endif // IOCTREE_MAPPER_H
