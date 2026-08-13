#include "comm.h"
#include "mapper/mapper.h"

#include <Octree.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap/OcTree.h>
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
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

struct Arguments {
  std::string backend;
  std::string bag_path;
  std::string output_path;
  std::size_t frames = 600;
  std::size_t frame_stride = 1;
  int repetition = 0;
  double window_xy = 20.0;
  double z_below = 6.0;
  double z_above = 2.0;
  double max_range = 50.0;
  double resolution = 0.5;
  std::size_t bucket_size = 32;
};

struct Box {
  float min[3]{};
  float max[3]{};
};

struct VoxelKey {
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
    return seed;
  }
};

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
    else if (option == "--bag")
      args.bag_path = value;
    else if (option == "--output")
      args.output_path = value;
    else if (option == "--frames")
      args.frames = ParseSize(value, option);
    else if (option == "--stride")
      args.frame_stride = ParseSize(value, option);
    else if (option == "--repetition")
      args.repetition = std::stoi(value);
    else if (option == "--window-xy")
      args.window_xy = std::stod(value);
    else if (option == "--z-below")
      args.z_below = std::stod(value);
    else if (option == "--z-above")
      args.z_above = std::stod(value);
    else if (option == "--max-range")
      args.max_range = std::stod(value);
    else if (option == "--resolution")
      args.resolution = std::stod(value);
    else if (option == "--bucket-size")
      args.bucket_size = ParseSize(value, option);
    else
      throw std::invalid_argument("unknown option: " + option);
  }

  if (args.backend != "octomap" && args.backend != "ioctree")
    throw std::invalid_argument("--backend must be octomap or ioctree");
  if (args.bag_path.empty())
    throw std::invalid_argument("--bag is required");
  if (args.output_path.empty())
    throw std::invalid_argument("--output is required");
  if (!std::isfinite(args.window_xy) || args.window_xy <= 0.0 ||
      !std::isfinite(args.z_below) || args.z_below <= 0.0 ||
      !std::isfinite(args.z_above) || args.z_above <= 0.0 ||
      !std::isfinite(args.max_range) || args.max_range <= 0.0 ||
      !std::isfinite(args.resolution) || args.resolution <= 0.0) {
    throw std::invalid_argument(
        "window, height, max range, and resolution must be positive");
  }
  return args;
}

Box MakeWindow(const V3D &center, const Arguments &args) {
  const float half_xy = static_cast<float>(0.5 * args.window_xy);
  Box box;
  box.min[0] = static_cast<float>(center.x()) - half_xy;
  box.max[0] = static_cast<float>(center.x()) + half_xy;
  box.min[1] = static_cast<float>(center.y()) - half_xy;
  box.max[1] = static_cast<float>(center.y()) + half_xy;
  box.min[2] = static_cast<float>(center.z() - args.z_below);
  box.max[2] = static_cast<float>(center.z() + args.z_above);
  return box;
}

bool Contains(const Box &box, float x, float y, float z) {
  return x >= box.min[0] && x <= box.max[0] && y >= box.min[1] &&
         y <= box.max[1] && z >= box.min[2] && z <= box.max[2];
}

bool ValidBox(const Box &box) {
  return box.min[0] <= box.max[0] && box.min[1] <= box.max[1] &&
         box.min[2] <= box.max[2];
}

// Return six disjoint boxes covering the space outside the retained window.
// The bound is below OctoMap's coordinate limit at the default resolution and
// far beyond this dataset. Closed query bounds require nextafter so points on
// the retained boundary are not removed.
std::vector<Box> OutsideBoxes(const Box &window) {
  constexpr float kCoordinateBound = 10000.0F;
  const float negative_inf = -std::numeric_limits<float>::infinity();
  const float positive_inf = std::numeric_limits<float>::infinity();
  std::vector<Box> boxes;
  boxes.reserve(6);
  auto append = [&](Box box) {
    if (ValidBox(box))
      boxes.push_back(box);
  };

  Box box{{-kCoordinateBound, -kCoordinateBound, -kCoordinateBound},
          {std::nextafter(window.min[0], negative_inf), kCoordinateBound,
           kCoordinateBound}};
  append(box);
  box = {{std::nextafter(window.max[0], positive_inf), -kCoordinateBound,
          -kCoordinateBound},
         {kCoordinateBound, kCoordinateBound, kCoordinateBound}};
  append(box);

  box = {{window.min[0], -kCoordinateBound, -kCoordinateBound},
         {window.max[0], std::nextafter(window.min[1], negative_inf),
          kCoordinateBound}};
  append(box);
  box = {{window.min[0], std::nextafter(window.max[1], positive_inf),
          -kCoordinateBound},
         {window.max[0], kCoordinateBound, kCoordinateBound}};
  append(box);

  box = {{window.min[0], window.min[1], -kCoordinateBound},
         {window.max[0], window.max[1],
          std::nextafter(window.min[2], negative_inf)}};
  append(box);
  box = {{window.min[0], window.min[1],
          std::nextafter(window.max[2], positive_inf)},
         {window.max[0], window.max[1], kCoordinateBound}};
  append(box);
  return boxes;
}

class LocalMapBackend {
public:
  virtual ~LocalMapBackend() = default;
  virtual void DeleteBoxes(const std::vector<Box> &boxes) = 0;
  virtual void Insert(PointCloudType &points) = 0;
  virtual std::size_t ElementCount() const = 0;
  virtual std::size_t CountOutside(const Box &window) = 0;
  virtual std::size_t StoredCellCount() = 0;
};

class IOctreeBackend final : public LocalMapBackend {
public:
  IOctreeBackend(const Arguments &args)
      : tree_(args.bucket_size, false,
              static_cast<float>(0.5 * args.resolution)) {
    tree_.set_down_size(false);
  }

  void DeleteBoxes(const std::vector<Box> &boxes) override {
    for (const Box &box : boxes)
      tree_.boxWiseDelete(box.min, box.max, true);
  }

  void Insert(PointCloudType &points) override {
    if (!points.empty())
      tree_.update(points, false);
  }

  std::size_t ElementCount() const override {
    // Upstream size() is not const-qualified and is only a diagnostic counter.
    return const_cast<thuni::Octree &>(tree_).size();
  }

  std::size_t CountOutside(const Box &window) override {
    const PointVector points = tree_.get_data<PointType, PointVector>();
    return static_cast<std::size_t>(std::count_if(
        points.begin(), points.end(), [&](const PointType &point) {
          return !Contains(window, point.x, point.y, point.z);
        }));
  }

  std::size_t StoredCellCount() override {
    return tree_.get_data<PointType, PointVector>().size();
  }

private:
  thuni::Octree tree_;
};

class OctomapBackend final : public LocalMapBackend {
public:
  explicit OctomapBackend(const Arguments &args) : tree_(args.resolution) {}

  void DeleteBoxes(const std::vector<Box> &boxes) override {
    std::vector<octomap::OcTreeKey> keys;
    for (const Box &box : boxes) {
      const octomap::point3d min_point(box.min[0], box.min[1], box.min[2]);
      const octomap::point3d max_point(box.max[0], box.max[1], box.max[2]);
      for (auto it = tree_.begin_leafs_bbx(min_point, max_point),
                end = tree_.end_leafs_bbx();
           it != end; ++it) {
        // The floating-point BBX iterator rounds its bounds to voxel keys and
        // can return a neighboring cell. Keep deletion semantics exact.
        if (Contains(box, it.getX(), it.getY(), it.getZ()))
          keys.push_back(it.getKey());
      }
    }
    for (const auto &key : keys)
      tree_.deleteNode(key);
  }

  void Insert(PointCloudType &points) override {
    for (const PointType &point : points) {
      octomap::OcTreeKey key;
      if (tree_.coordToKeyChecked(point.x, point.y, point.z, key))
        tree_.updateNode(key, true, true);
    }
    // lazy_eval deliberately avoids inner-node occupancy propagation and
    // pruning. This benchmark needs endpoint storage, not occupancy queries.
  }

  std::size_t ElementCount() const override { return tree_.size(); }

  std::size_t CountOutside(const Box &window) override {
    std::size_t count = 0;
    for (auto it = tree_.begin_leafs(), end = tree_.end_leafs(); it != end; ++it) {
      if (!Contains(window, it.getX(), it.getY(), it.getZ()))
        ++count;
    }
    return count;
  }

  std::size_t StoredCellCount() override { return tree_.getNumLeafNodes(); }

private:
  octomap::OcTree tree_;
};

std::unique_ptr<LocalMapBackend> MakeBackend(const Arguments &args) {
  if (args.backend == "octomap")
    return std::make_unique<OctomapBackend>(args);
  return std::make_unique<IOctreeBackend>(args);
}

double MillisecondsSince(const std::chrono::steady_clock::time_point &start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

void RunBenchmark(const Arguments &args) {
  auto backend = MakeBackend(args);
  std::ofstream output(args.output_path);
  if (!output)
    throw std::runtime_error("cannot open output: " + args.output_path);
  output << "backend,repetition,frame,timestamp,input_points,window_points,"
            "endpoint_voxels,new_voxels,evicted_voxels,active_voxels,delete_boxes,"
            "crop_wall_ms,insert_wall_ms,total_wall_ms,"
            "thread_cpu_ms,thread_cpu_percent,map_elements,rss_mib,"
            "rss_delta_mib,center_x,center_y,center_z\n";
  output << std::fixed << std::setprecision(6);

  rosbag::Bag bag;
  bag.open(args.bag_path, rosbag::bagmode::Read);
  const std::vector<std::string> topics{"/aircraft_pose_flu", "/rslidar_points"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  geometry_msgs::PoseStamped::ConstPtr latest_pose;
  std::size_t lidar_index = 0;
  std::size_t frame_index = 0;
  Box final_window;
  std::unordered_set<VoxelKey, VoxelKeyHash> active_voxels;
  double baseline_rss_mib = std::numeric_limits<double>::quiet_NaN();
  const double max_range_squared = args.max_range * args.max_range;
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
    const V3D origin(pose.position.x, pose.position.y, pose.position.z);
    const Box window = MakeWindow(origin, args);

    PointCloudType frame_endpoints;
    frame_endpoints.reserve(raw_cloud.size());
    std::unordered_set<VoxelKey, VoxelKeyHash> endpoint_voxels;
    endpoint_voxels.reserve(raw_cloud.size());
    std::size_t window_points = 0;
    for (const auto &raw_point : raw_cloud) {
      if (!mapping::IsValidPoint(raw_point.x, raw_point.y, raw_point.z))
        continue;
      const V3D transformed =
          r_wl * V3D(raw_point.x, raw_point.y, raw_point.z) + origin;
      const double dx = transformed.x() - origin.x();
      const double dy = transformed.y() - origin.y();
      const double dz = transformed.z() - origin.z();
      if (dx * dx + dy * dy + dz * dz > max_range_squared ||
          !Contains(window, static_cast<float>(transformed.x()),
                    static_cast<float>(transformed.y()),
                    static_cast<float>(transformed.z()))) {
        continue;
      }
      ++window_points;
      const VoxelKey voxel{
          static_cast<std::int64_t>(std::floor(transformed.x() / args.resolution)),
          static_cast<std::int64_t>(std::floor(transformed.y() / args.resolution)),
          static_cast<std::int64_t>(std::floor(transformed.z() / args.resolution))};
      if (!endpoint_voxels.insert(voxel).second)
        continue;
      const float center_x =
          static_cast<float>((static_cast<double>(voxel.x) + 0.5) * args.resolution);
      const float center_y =
          static_cast<float>((static_cast<double>(voxel.y) + 0.5) * args.resolution);
      const float center_z =
          static_cast<float>((static_cast<double>(voxel.z) + 0.5) * args.resolution);
      if (!Contains(window, center_x, center_y, center_z))
        continue;
      PointType point{};
      point.x = center_x;
      point.y = center_y;
      point.z = center_z;
      point.intensity = raw_point.intensity;
      frame_endpoints.push_back(point);
    }

    std::size_t evicted_voxels = 0;
    for (auto it = active_voxels.begin(); it != active_voxels.end();) {
      const float center_x = static_cast<float>(
          (static_cast<double>(it->x) + 0.5) * args.resolution);
      const float center_y = static_cast<float>(
          (static_cast<double>(it->y) + 0.5) * args.resolution);
      const float center_z = static_cast<float>(
          (static_cast<double>(it->z) + 0.5) * args.resolution);
      if (Contains(window, center_x, center_y, center_z)) {
        ++it;
      } else {
        it = active_voxels.erase(it);
        ++evicted_voxels;
      }
    }
    PointCloudType additions;
    additions.reserve(frame_endpoints.size());
    for (const PointType &point : frame_endpoints) {
      const VoxelKey voxel{
          static_cast<std::int64_t>(std::floor(point.x / args.resolution)),
          static_cast<std::int64_t>(std::floor(point.y / args.resolution)),
          static_cast<std::int64_t>(std::floor(point.z / args.resolution))};
      if (active_voxels.insert(voxel).second)
        additions.push_back(point);
    }

    const std::vector<Box> delete_boxes = OutsideBoxes(window);
    if (!std::isfinite(baseline_rss_mib)) {
      const std::int64_t rss_bytes = GetProcessRssBytes();
      baseline_rss_mib = rss_bytes >= 0
                             ? static_cast<double>(rss_bytes) / (1024.0 * 1024.0)
                             : std::numeric_limits<double>::quiet_NaN();
    }

    const auto total_start = std::chrono::steady_clock::now();
    const double cpu_before = GetThreadCpuTimeMs();
    const auto crop_start = std::chrono::steady_clock::now();
    backend->DeleteBoxes(delete_boxes);
    const double crop_ms = MillisecondsSince(crop_start);
    const auto insert_start = std::chrono::steady_clock::now();
    backend->Insert(additions);
    const double insert_ms = MillisecondsSince(insert_start);
    const double cpu_ms = GetThreadCpuTimeMs() - cpu_before;
    const double total_ms = MillisecondsSince(total_start);

    const std::int64_t rss_bytes = GetProcessRssBytes();
    const double rss_mib = rss_bytes >= 0
                               ? static_cast<double>(rss_bytes) / (1024.0 * 1024.0)
                               : std::numeric_limits<double>::quiet_NaN();
    const double cpu_percent = total_ms > 0.0 ? 100.0 * cpu_ms / total_ms : 0.0;
    output << args.backend << ',' << args.repetition << ',' << frame_index << ','
           << cloud_message->header.stamp.toSec() << ',' << raw_cloud.size() << ','
           << window_points << ',' << frame_endpoints.size() << ','
           << additions.size() << ',' << evicted_voxels << ','
           << active_voxels.size() << ',' << delete_boxes.size() << ',' << crop_ms
           << ',' << insert_ms << ',' << total_ms << ',' << cpu_ms << ','
           << cpu_percent << ',' << backend->ElementCount() << ',' << rss_mib
           << ',' << (rss_mib - baseline_rss_mib) << ',' << origin.x() << ','
           << origin.y() << ',' << origin.z() << '\n';

    final_window = window;
    ++frame_index;
    if (frame_index == args.frames)
      break;
  }
  bag.close();

  if (frame_index != args.frames) {
    throw std::runtime_error("bag contains only " + std::to_string(frame_index) +
                             " usable selected LiDAR frames");
  }
  const std::size_t outside = backend->CountOutside(final_window);
  if (outside != 0) {
    throw std::runtime_error("local-map invariant failed: " +
                             std::to_string(outside) +
                             " stored elements are outside the final window");
  }
  const std::size_t stored_cells = backend->StoredCellCount();
  if (stored_cells != active_voxels.size()) {
    throw std::runtime_error(
        "local-map content invariant failed: backend stores " +
        std::to_string(stored_cells) + " cells but reference has " +
        std::to_string(active_voxels.size()));
  }
  std::cout << args.backend << ": validated " << frame_index
            << " frames; final outside-window count = 0; stored cells = "
            << stored_cells << '\n';
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_map_maintenance_benchmark",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  try {
    const Arguments args = ParseArguments(argc, argv);
    RunBenchmark(args);
  } catch (const std::exception &error) {
    std::cerr << "local_map_maintenance_benchmark: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
