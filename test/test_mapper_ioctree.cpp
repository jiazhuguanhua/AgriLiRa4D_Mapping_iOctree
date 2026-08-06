#include "mapper/mapper_ioctree.h"
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
}

TEST(IOctreeMapperTest, EmptyInputDoesNotCreateMap) {
  IOctreeMapper mapper(TestOptions());
  MapperInput input;
  input.lidar_cloud.reset(new PointCloudType);
  mapper.Update(input);

  CloudPtr map;
  EXPECT_FALSE(mapper.GetMapCloud(map));
  ASSERT_TRUE(map);
  EXPECT_TRUE(map->empty());
}

TEST(IOctreeMapperTest, FiltersInvalidAndOutOfRangePoints) {
  IOctreeMapper mapper(TestOptions());
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto input = MakeInput(
      {MakePoint(1.0f, 0.0f, 0.0f, 12.0f),
       MakePoint(11.0f, 0.0f, 0.0f), MakePoint(nan, 0.0f, 0.0f)});
  mapper.Update(input);

  CloudPtr map;
  ASSERT_TRUE(mapper.GetMapCloud(map));
  ASSERT_EQ(map->size(), 1U);
  EXPECT_FLOAT_EQ(map->front().x, 1.0f);
  EXPECT_FLOAT_EQ(map->front().intensity, 1.0f);
  EXPECT_EQ(map->width, 1U);
  EXPECT_EQ(map->height, 1U);
  EXPECT_TRUE(map->is_dense);
}

TEST(IOctreeMapperTest, AccumulatesAcrossUpdatesAndResetsCleanly) {
  IOctreeMapper mapper(TestOptions());
  mapper.Update(MakeInput({MakePoint(1.0f, 0.0f, 0.0f)}));
  mapper.Update(MakeInput({MakePoint(2.0f, 0.0f, 0.0f),
                           MakePoint(3.0f, 0.0f, 0.0f)}));

  CloudPtr map;
  ASSERT_TRUE(mapper.GetMapCloud(map));
  EXPECT_EQ(map->size(), 3U);

  mapper.Reset();
  EXPECT_FALSE(mapper.GetMapCloud(map));
  EXPECT_TRUE(map->empty());

  mapper.Update(MakeInput({MakePoint(4.0f, 0.0f, 0.0f)}));
  ASSERT_TRUE(mapper.GetMapCloud(map));
  ASSERT_EQ(map->size(), 1U);
  EXPECT_FLOAT_EQ(map->front().x, 4.0f);
}

TEST(IOctreeMapperTest, RejectsInvalidOrigin) {
  IOctreeMapper mapper(TestOptions());
  auto input = MakeInput({MakePoint(1.0f, 0.0f, 0.0f)});
  input.lidar_origin.x() = std::numeric_limits<double>::infinity();
  mapper.Update(input);

  CloudPtr map;
  EXPECT_FALSE(mapper.GetMapCloud(map));
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

    CloudPtr map;
    ASSERT_TRUE(mapper.GetMapCloud(map));
    ASSERT_EQ(map->size(),
              static_cast<std::size_t>(kBatches * kPointsPerBatch));
    for (const auto &point : *map) {
      EXPECT_TRUE(std::isfinite(point.x));
      EXPECT_TRUE(std::isfinite(point.y));
      EXPECT_TRUE(std::isfinite(point.z));
    }
    mapper.Reset();
    EXPECT_FALSE(mapper.GetMapCloud(map));
  }
}

TEST(IOctreeMapperTest, DownsamplingBoundsRepeatedPointGrowth) {
  auto options = TestOptions();
  options.bucket_size = 8;
  options.min_extent = 0.1;
  options.downsample = true;
  IOctreeMapper mapper(options);

  for (int i = 0; i < 100; ++i) {
    mapper.Update(MakeInput({MakePoint(1.0f, 1.0f, 1.0f)}));
  }

  CloudPtr map;
  ASSERT_TRUE(mapper.GetMapCloud(map));
  EXPECT_LT(map->size(), 100U);
  EXPECT_GE(map->size(), 1U);
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
  EXPECT_EQ(options.pose_gt_file,
            std::string(MAPPING_TEST_SOURCE_DIR) +
                "/test/data/integration_poses.txt");
}

} // namespace
} // namespace mapping

int main(int argc, char **argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
