# AgriLiRa4D_Mapping

***Mapping package for "AgriLiRa4D: A Multi-Sensor UAV Dataset for Robust SLAM in Challenging Agricultural Fields".***

![](imgs/pipeline.jpg)

**Supported Platforms**

- ROS-Noetic on Ubuntu20.04
<!-- - ROS-One on Ubuntu22.04 -->

## Third-party

- PCL
- Eigen3
- Octomap

## Run

```bash
# build
mkdir -p ws_agrilira4d/src
cd ws_agrilira4d/src
git clone --recurse-submodules git@github.com:zhan994/AgriLiRa4D_Mapping.git
cd ..
catkin_make

# run
source devel/setup.bash
roslaunch mapping mapping.launch
```

If the repository was cloned without `--recurse-submodules`, initialize the
pinned i-Octree dependency before building:

```bash
git submodule update --init --recursive
```

## Mapping backends

The mapper backend is selected in `config/agrilira4d.yaml`:

```yaml
mapper:
  type: 1 # 0: OctoMap occupancy map, 1: i-Octree point map
```

The i-Octree backend incrementally stores the motion-compensated LiDAR points
in the `world` frame. It publishes the complete point map as a latched
`sensor_msgs/PointCloud2` message on `/ioctree_map_points`. Unlike OctoMap, the
i-Octree backend is a point-cloud spatial index and does not represent free or
unknown space. `publish_period` limits full-map export and publication; map
insertion still runs for every synchronized measurement group.

The i-Octree options are:

```yaml
ioctree:
  min_extent: 0.25
  bucket_size: 32
  downsample: true
  max_range: 50.0
```

`IOctreeMapper::Save()` writes a binary PCD file. The existing OctoMap backend
remains available for regression testing and publishes `/octomap_binary`.


## Notes

**Ground-Truth Odometry for FRD (Body) in FLU**

![](imgs/FLU_ENU.png)

**Extrinsics for Robosense Airy LiDAR**

![](imgs/Airy_Extrinsics.png)


