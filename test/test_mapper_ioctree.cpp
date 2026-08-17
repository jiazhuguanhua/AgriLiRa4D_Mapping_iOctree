#include "mapper/mapper_ioctree.h"
#include "mapper/mapper_octomap.h"
#include "options.h"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <unistd.h>

namespace mapping {
namespace {

PointType MakePoint(float x, float y, float z, float intensity = 0.0f) {
  PointType point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.normal_x = 0.0f;
  point.normal_y = 0.0f;
  point.normal_z = 0.0f;
  point.curvature = 0.0f;
  return point;
}

MapperInput MakeInput(std::initializer_list<PointType> points) {
  MapperInput input;
  input.lidar_origin = V3D::Zero();
  input.lidar_cloud.reset(new PointCloudType);
  input.lidar_cloud->points.assign(points.begin(), points.end());
  input.lidar_cloud->width = input.lidar_cloud->points.size();
  input.lidar_cloud->height = 1;
  return input;
}

IOctreeMapper::Options TestOptions() {
  IOctreeMapper::Options options;
  options.min_extent = 0.05;
  options.bucket_size = 8;
  options.downsample = false;
  options.max_range = 10.0;
  options.occupancy_resolution = 0.5;
  options.min_points_per_voxel = 1;
  options.occupancy_threshold = 0.5;
  options.probability_scale = 1.0;
  return options;
}

TEST(IOctreeMapperTest, RejectsInvalidOptions) {
  auto options = TestOptions();
  options.min_extent = 0.0;
  EXPECT_THROW(IOctreeMapper mapper(options), std::invalid_argument);

  options = TestOptions();
  options.bucket_size = 0;
  EXPECT_THROW(IOctreeMapper mapper(options), std::invalid_argument);

  options = TestOptions();
  options.max_range = -1.0;
  EXPECT_THROW(IOctreeMapper mapper(options), std::invalid_argument);

  options = TestOptions();
  options.occupancy_resolution = 0.0;
  EXPECT_THROW(IOctreeMapper mapper(options), std::invalid_argument);

  options = TestOptions();
  options.min_points_per_voxel = 0;
  EXPECT_THROW(IOctreeMapper mapper(options), std::invalid_argument);

  options = TestOptions();
  options.occupancy_threshold = 0.49;
  EXPECT_THROW(IOctreeMapper mapper(options), std::invalid_argument);

  options = TestOptions();
  options.occupancy_threshold = 1.1;
  EXPECT_THROW(IOctreeMapper mapper(options), std::invalid_argument);

  options = TestOptions();
  options.probability_scale = 0.0;
  EXPECT_THROW(IOctreeMapper mapper(options), std::invalid_argument);
}

TEST(IOctreeMapperTest, EmptyInputDoesNotCreateMap) {
  IOctreeMapper mapper(TestOptions());
  MapperInput input;
  input.lidar_cloud.reset(new PointCloudType);
  mapper.Update(input);

  OccupancyMap map;
  EXPECT_FALSE(mapper.GetOccupiedVoxels(map));
  EXPECT_TRUE(map.voxels.empty());
  EXPECT_DOUBLE_EQ(map.resolution, 0.5);
}

TEST(IOctreeMapperTest, FiltersInvalidAndOutOfRangePoints) {
  IOctreeMapper mapper(TestOptions());
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto input = MakeInput(
      {MakePoint(1.0f, 0.0f, 0.0f, 12.0f),
       MakePoint(11.0f, 0.0f, 0.0f), MakePoint(nan, 0.0f, 0.0f)});
  mapper.Update(input);

  OccupancyMap map;
  ASSERT_TRUE(mapper.GetOccupiedVoxels(map));
  ASSERT_EQ(map.voxels.size(), 1U);
  EXPECT_DOUBLE_EQ(map.voxels.front().x, 1.25);
  EXPECT_EQ(map.voxels.front().point_count, 1U);
  EXPECT_NEAR(map.voxels.front().probability, 1.0 - std::exp(-1.0), 1e-6);
}

TEST(IOctreeMapperTest, AccumulatesAcrossUpdatesAndResetsCleanly) {
  IOctreeMapper mapper(TestOptions());
  mapper.Update(MakeInput({MakePoint(1.0f, 0.0f, 0.0f)}));
  mapper.Update(MakeInput({MakePoint(2.0f, 0.0f, 0.0f),
                           MakePoint(3.0f, 0.0f, 0.0f)}));

  OccupancyMap map;
  ASSERT_TRUE(mapper.GetOccupiedVoxels(map));
  EXPECT_EQ(map.voxels.size(), 3U);

  mapper.Reset();
  EXPECT_FALSE(mapper.GetOccupiedVoxels(map));
  EXPECT_TRUE(map.voxels.empty());

  mapper.Update(MakeInput({MakePoint(4.0f, 0.0f, 0.0f)}));
  ASSERT_TRUE(mapper.GetOccupiedVoxels(map));
  ASSERT_EQ(map.voxels.size(), 1U);
  EXPECT_DOUBLE_EQ(map.voxels.front().x, 4.25);
}

TEST(IOctreeMapperTest, RejectsInvalidOrigin) {
  IOctreeMapper mapper(TestOptions());
  auto input = MakeInput({MakePoint(1.0f, 0.0f, 0.0f)});
  input.lidar_origin.x() = std::numeric_limits<double>::infinity();
  mapper.Update(input);

  OccupancyMap map;
  EXPECT_FALSE(mapper.GetOccupiedVoxels(map));
}

TEST(IOctreeMapperTest, SavesBinaryPcdAndCanReadItBack) {
  IOctreeMapper mapper(TestOptions());
  mapper.Update(MakeInput({MakePoint(1.0f, 2.0f, 3.0f),
                           MakePoint(4.0f, 5.0f, 6.0f)}));

  const std::string path =
      "/tmp/mapping_ioctree_test_" + std::to_string(getpid()) + ".pcd";
  ASSERT_TRUE(mapper.Save(path));

  PointCloudType loaded;
  ASSERT_EQ(pcl::io::loadPCDFile(path, loaded), 0);
  EXPECT_EQ(loaded.size(), 2U);
  EXPECT_EQ(std::remove(path.c_str()), 0);
}

TEST(IOctreeMapperTest, HandlesLargeIncrementalMapAndRepeatedReset) {
  auto options = TestOptions();
  options.bucket_size = 4;
  options.min_extent = 0.01;
  options.max_range = 100.0;
  IOctreeMapper mapper(options);

  constexpr int kCycles = 3;
  constexpr int kBatches = 10;
  constexpr int kPointsPerBatch = 100;
  for (int cycle = 0; cycle < kCycles; ++cycle) {
    for (int batch = 0; batch < kBatches; ++batch) {
      MapperInput input;
      input.lidar_origin = V3D::Zero();
      input.lidar_cloud.reset(new PointCloudType);
      for (int i = 0; i < kPointsPerBatch; ++i) {
        const int index = batch * kPointsPerBatch + i;
        input.lidar_cloud->push_back(
            MakePoint(0.02f * static_cast<float>(index % 50),
                      0.02f * static_cast<float>((index / 50) % 20),
                      1.0f + 0.01f * static_cast<float>(index / 1000)));
      }
      mapper.Update(input);
    }

    OccupancyMap map;
    ASSERT_TRUE(mapper.GetOccupiedVoxels(map));
    EXPECT_FALSE(map.voxels.empty());
    for (const auto &voxel : map.voxels) {
      EXPECT_TRUE(std::isfinite(voxel.x));
      EXPECT_TRUE(std::isfinite(voxel.y));
      EXPECT_TRUE(std::isfinite(voxel.z));
      EXPECT_GE(voxel.probability, options.occupancy_threshold);
    }
    mapper.Reset();
    EXPECT_FALSE(mapper.GetOccupiedVoxels(map));
  }
}

TEST(IOctreeMapperTest, CountsHitsIndependentlyOfPointMapDownsampling) {
  auto options = TestOptions();
  options.bucket_size = 8;
  options.min_extent = 0.1;
  options.downsample = true;
  IOctreeMapper mapper(options);

  for (int i = 0; i < 100; ++i) {
    mapper.Update(MakeInput({MakePoint(1.0f, 1.0f, 1.0f)}));
  }

  OccupancyMap map;
  ASSERT_TRUE(mapper.GetOccupiedVoxels(map));
  ASSERT_EQ(map.voxels.size(), 1U);
  EXPECT_EQ(map.voxels.front().point_count, 100U);
  EXPECT_GT(map.voxels.front().probability, 0.99f);
}

TEST(IOctreeMapperTest, AppliesPointCountAndProbabilityThresholds) {
  auto options = TestOptions();
  options.min_points_per_voxel = 2;
  options.occupancy_threshold = 0.8;
  options.probability_scale = 2.0;
  IOctreeMapper mapper(options);

  mapper.Update(MakeInput({MakePoint(1.0f, 0.0f, 0.0f),
                           MakePoint(1.1f, 0.0f, 0.0f),
                           MakePoint(2.0f, 0.0f, 0.0f)}));
  OccupancyMap map;
  EXPECT_FALSE(mapper.GetOccupiedVoxels(map));

  mapper.Update(MakeInput({MakePoint(1.2f, 0.0f, 0.0f),
                           MakePoint(1.3f, 0.0f, 0.0f)}));
  ASSERT_TRUE(mapper.GetOccupiedVoxels(map));
  ASSERT_EQ(map.voxels.size(), 1U);
  EXPECT_EQ(map.voxels.front().point_count, 4U);
  EXPECT_NEAR(map.voxels.front().probability, 1.0 - std::exp(-2.0), 1e-6);
}

TEST(IOctreeMapperTest, LoadsIOctreeConfigurationAndRelativePosePath) {
  const std::string config =
      std::string(MAPPING_TEST_SOURCE_DIR) + "/test/data/integration.yaml";
  const Options options = LoadOptionsFromFile(config);
  EXPECT_EQ(options.mapper_type, 1);
  EXPECT_DOUBLE_EQ(options.ioctree_options.min_extent, 0.05);
  EXPECT_EQ(options.ioctree_options.bucket_size, 8U);
  EXPECT_FALSE(options.ioctree_options.downsample);
  EXPECT_DOUBLE_EQ(options.ioctree_options.max_range, 20.0);
  EXPECT_DOUBLE_EQ(options.ioctree_options.occupancy_resolution, 0.5);
  EXPECT_EQ(options.ioctree_options.min_points_per_voxel, 1U);
  EXPECT_DOUBLE_EQ(options.ioctree_options.occupancy_threshold, 0.5);
  EXPECT_DOUBLE_EQ(options.ioctree_options.probability_scale, 1.0);
  EXPECT_EQ(options.pose_gt_file,
            std::string(MAPPING_TEST_SOURCE_DIR) +
                "/test/data/integration_poses.txt");
}

TEST(MapperInterfaceTest, OctomapExportsOnlyOccupiedLeafVoxels) {
  OctoMapper::Options options;
  options.resolution = 0.5;
  options.max_range = 10.0;
  options.hit_prob = 0.7;
  options.miss_prob = 0.4;
  options.occupancy_threshold = 0.5;
  OctoMapper mapper(options);

  mapper.Update(MakeInput({MakePoint(2.0f, 0.0f, 0.0f)}));

  OccupancyMap map;
  ASSERT_TRUE(mapper.GetOccupiedVoxels(map));
  ASSERT_EQ(map.voxels.size(), 1U);
  EXPECT_DOUBLE_EQ(map.resolution, options.resolution);
  EXPECT_NEAR(map.voxels.front().probability, options.hit_prob, 1e-6);
  EXPECT_EQ(map.voxels.front().point_count, 0U);
}

} // namespace
} // namespace mapping

int main(int argc, char **argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
