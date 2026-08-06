#include "comm.h"
#include "mapper/mapper_ioctree.h"
#include "mapper/mapper_octomap.h"

#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Arguments {
  std::string backend;
  std::string scenario = "synthetic";
  std::string bag_path;
  std::string output_path;
  std::size_t frames = 100;
  std::size_t points_per_frame = 10000;
  std::size_t frame_stride = 1;
  int repetition = 0;
  double max_range = 50.0;
};

struct BenchmarkFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CloudPtr cloud{new PointCloudType};
  V3D origin = V3D::Zero();
  double timestamp = 0.0;
};

using BenchmarkFrames =
    std::vector<BenchmarkFrame, Eigen::aligned_allocator<BenchmarkFrame>>;

std::size_t ParseSize(const std::string &value, const std::string &name) {
  std::size_t consumed = 0;
  const auto parsed = std::stoull(value, &consumed);
  if (consumed != value.size() || parsed == 0)
    throw std::invalid_argument(name + " must be a positive integer");
  return static_cast<std::size_t>(parsed);
}

Arguments ParseArguments(int argc, char **argv) {
  Arguments args;
  for (int i = 1; i < argc; ++i) {
    const std::string option(argv[i]);
    if (i + 1 >= argc)
      throw std::invalid_argument("missing value for " + option);
    const std::string value(argv[++i]);
    if (option == "--backend")
      args.backend = value;
    else if (option == "--scenario")
      args.scenario = value;
    else if (option == "--bag")
      args.bag_path = value;
    else if (option == "--output")
      args.output_path = value;
    else if (option == "--frames")
      args.frames = ParseSize(value, option);
    else if (option == "--points")
      args.points_per_frame = ParseSize(value, option);
    else if (option == "--stride")
      args.frame_stride = ParseSize(value, option);
    else if (option == "--repetition")
      args.repetition = std::stoi(value);
    else if (option == "--max-range")
      args.max_range = std::stod(value);
    else
      throw std::invalid_argument("unknown option: " + option);
  }

  if (args.backend != "octomap" && args.backend != "ioctree")
    throw std::invalid_argument("--backend must be octomap or ioctree");
  if (args.scenario != "synthetic" && args.scenario != "real")
    throw std::invalid_argument("--scenario must be synthetic or real");
  if (args.output_path.empty())
    throw std::invalid_argument("--output is required");
  if (args.scenario == "real" && args.bag_path.empty())
    throw std::invalid_argument("--bag is required for the real scenario");
  if (!std::isfinite(args.max_range) || args.max_range <= 0.0)
    throw std::invalid_argument("--max-range must be positive");
  return args;
}

PointType MakePoint(float x, float y, float z, float intensity) {
  PointType point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

BenchmarkFrames MakeSyntheticFrames(const Arguments &args) {
  BenchmarkFrames frames;
  frames.reserve(args.frames);
  std::mt19937 generator(20260806);
  std::uniform_real_distribution<float> jitter(-0.015f, 0.015f);
  constexpr double kPi = 3.14159265358979323846;

  for (std::size_t frame_index = 0; frame_index < args.frames; ++frame_index) {
    BenchmarkFrame frame;
    frame.timestamp = static_cast<double>(frame_index) * 0.1;
    frame.origin = V3D(0.08 * static_cast<double>(frame_index), 0.0, 2.5);
    frame.cloud->reserve(args.points_per_frame);

    for (std::size_t point_index = 0; point_index < args.points_per_frame;
         ++point_index) {
      const double azimuth = 2.0 * kPi * static_cast<double>(point_index % 2048) /
                             2048.0;
      const double band = static_cast<double>((point_index / 2048) % 32) / 31.0;
      const double range = 4.0 + 40.0 *
                                    static_cast<double>((point_index * 37) % 1000) /
                                    999.0;
      const double elevation = -0.20 + 0.35 * band;
      const double horizontal = range * std::cos(elevation);
      const float x = static_cast<float>(frame.origin.x() +
                                         horizontal * std::cos(azimuth)) +
                      jitter(generator);
      const float y = static_cast<float>(frame.origin.y() +
                                         horizontal * std::sin(azimuth)) +
                      jitter(generator);
      const float z = static_cast<float>(frame.origin.z() +
                                         range * std::sin(elevation)) +
                      jitter(generator);
      frame.cloud->push_back(
          MakePoint(x, y, z, static_cast<float>(point_index % 255)));
    }
    frames.push_back(std::move(frame));
  }
  return frames;
}

BenchmarkFrames LoadRealFrames(const Arguments &args) {
  BenchmarkFrames frames;
  frames.reserve(args.frames);

  rosbag::Bag bag;
  bag.open(args.bag_path, rosbag::bagmode::Read);
  const std::vector<std::string> topics{"/aircraft_pose_flu", "/rslidar_points"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  geometry_msgs::PoseStamped::ConstPtr latest_pose;
  std::size_t lidar_index = 0;
  const M3D r_bl = (M3D() << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0)
                         .finished();

  for (const rosbag::MessageInstance &message : view) {
    if (message.getTopic() == "/aircraft_pose_flu") {
      latest_pose = message.instantiate<geometry_msgs::PoseStamped>();
      continue;
    }
    if (!latest_pose || lidar_index++ % args.frame_stride != 0)
      continue;

    const auto cloud_message = message.instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_message)
      continue;
    pcl::PointCloud<robosense_ros::Point> raw_cloud;
    pcl::fromROSMsg(*cloud_message, raw_cloud);

    const auto &pose = latest_pose->pose;
    const Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                       pose.orientation.y, pose.orientation.z);
    const M3D r_wl = quaternion.normalized().toRotationMatrix() * r_bl;
    const V3D t_wl(pose.position.x, pose.position.y, pose.position.z);

    BenchmarkFrame frame;
    frame.timestamp = cloud_message->header.stamp.toSec();
    frame.origin = t_wl;
    frame.cloud->reserve(raw_cloud.size());
    for (const auto &raw_point : raw_cloud) {
      if (!mapping::IsValidPoint(raw_point.x, raw_point.y, raw_point.z))
        continue;
      const V3D transformed = r_wl * V3D(raw_point.x, raw_point.y, raw_point.z) +
                              t_wl;
      frame.cloud->push_back(MakePoint(
          static_cast<float>(transformed.x()), static_cast<float>(transformed.y()),
          static_cast<float>(transformed.z()), raw_point.intensity));
    }
    frames.push_back(std::move(frame));
    if (frames.size() == args.frames)
      break;
  }
  bag.close();
  if (frames.size() != args.frames) {
    throw std::runtime_error("bag contains only " + std::to_string(frames.size()) +
                             " usable selected LiDAR frames");
  }
  return frames;
}

std::unique_ptr<mapping::Mapper> MakeMapper(const Arguments &args) {
  if (args.backend == "octomap") {
    mapping::OctoMapper::Options options;
    options.resolution = 0.5;
    options.max_range = args.max_range;
    return std::make_unique<mapping::OctoMapper>(options);
  }
  mapping::IOctreeMapper::Options options;
  options.min_extent = 0.25;
  options.bucket_size = 32;
  options.downsample = true;
  options.max_range = args.max_range;
  return std::make_unique<mapping::IOctreeMapper>(options);
}

std::size_t CountWithinRange(const BenchmarkFrame &frame, double max_range) {
  const double max_range_squared = max_range * max_range;
  return static_cast<std::size_t>(std::count_if(
      frame.cloud->begin(), frame.cloud->end(), [&](const PointType &point) {
        const double dx = point.x - frame.origin.x();
        const double dy = point.y - frame.origin.y();
        const double dz = point.z - frame.origin.z();
        return dx * dx + dy * dy + dz * dz <= max_range_squared;
      }));
}

void RunBenchmark(const Arguments &args, const BenchmarkFrames &frames) {
  auto mapper = MakeMapper(args);
  std::ofstream output(args.output_path);
  if (!output)
    throw std::runtime_error("cannot open output: " + args.output_path);
  output << "backend,scenario,point_setting,repetition,frame,input_points,within_range_points,"
            "update_wall_ms,thread_cpu_ms,thread_cpu_percent,map_elements,rss_mib\n";
  output << std::fixed << std::setprecision(6);

  for (std::size_t index = 0; index < frames.size(); ++index) {
    mapping::MapperInput input;
    input.timestamp = frames[index].timestamp;
    input.lidar_origin = frames[index].origin;
    input.lidar_cloud = frames[index].cloud;

    const auto wall_before = std::chrono::steady_clock::now();
    const double cpu_before = GetThreadCpuTimeMs();
    mapper->Update(input);
    const double cpu_after = GetThreadCpuTimeMs();
    const auto wall_after = std::chrono::steady_clock::now();

    const double wall_ms =
        std::chrono::duration<double, std::milli>(wall_after - wall_before).count();
    const double cpu_ms = cpu_after - cpu_before;
    const double cpu_percent = wall_ms > 0.0 ? 100.0 * cpu_ms / wall_ms : 0.0;
    const std::int64_t rss_bytes = GetProcessRssBytes();
    const double rss_mib = rss_bytes >= 0
                               ? static_cast<double>(rss_bytes) / (1024.0 * 1024.0)
                               : std::numeric_limits<double>::quiet_NaN();

    const std::size_t point_setting =
        args.scenario == "synthetic" ? args.points_per_frame : 0;
    output << args.backend << ',' << args.scenario << ',' << point_setting << ','
           << args.repetition << ',' << index << ',' << input.lidar_cloud->size() << ','
           << CountWithinRange(frames[index], args.max_range) << ',' << wall_ms
           << ',' << cpu_ms << ',' << cpu_percent << ','
           << mapper->GetMapElementCount() << ',' << rss_mib << '\n';
  }
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_update_benchmark",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  try {
    const Arguments args = ParseArguments(argc, argv);
    const BenchmarkFrames frames = args.scenario == "real"
                                       ? LoadRealFrames(args)
                                       : MakeSyntheticFrames(args);
    RunBenchmark(args, frames);
  } catch (const std::exception &error) {
    std::cerr << "map_update_benchmark: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
