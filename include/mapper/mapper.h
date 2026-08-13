/**
 * @file mapper.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief mapper for AgriLiRa4D
 * @version 0.1
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef MAPPER_H
#define MAPPER_H

#include "comm.h"

#include <cstdint>
#include <vector>

namespace mapping {

/// @brief  Check if a point is valid (finite and within a reasonable range)
bool IsValidPoint(const double &x, const double &y, const double &z);

/**
 * @brief One synchronized observation expressed in the world frame.
 *
 * Mapping owns synchronization, motion compensation and extrinsic transforms.
 * A mapper backend only consumes the resulting observation and maintains its
 * own map representation.
 */
struct MapperInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = -1.0;
  Pose6D body_pose;
  V3D lidar_origin = V3D::Zero();
  V3D radar_origin = V3D::Zero();
  CloudPtr lidar_cloud;
  RadarCloudPtr radar_cloud;
};

/**
 * @brief One occupied cell in a backend-independent map snapshot.
 *
 * Unknown and free cells are deliberately omitted. Probability is in [0, 1]
 * and point_count is zero when the backend does not maintain hit counts.
 */
struct OccupiedVoxel {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  float probability = 0.0f;
  std::uint32_t point_count = 0;
};

struct OccupancyMap {
  double resolution = 0.0;
  std::vector<OccupiedVoxel> voxels;
};

class Mapper {
public:
  virtual ~Mapper() = default;
  virtual void Update(const MapperInput &input) = 0;
  virtual void Reset() = 0;
  virtual bool Save(const std::string &path) const = 0;
  virtual bool GetOccupiedVoxels(OccupancyMap &map) const = 0;
};

} // namespace mapping

#endif // MAPPER_H
