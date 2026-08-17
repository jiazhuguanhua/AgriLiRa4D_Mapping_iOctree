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

Both backends expose the same occupied-voxel snapshot interface and publish a
latched full `octomap_msgs/Octomap` message on `/occupied_voxels`. Only occupied
leaf cells are exported; free and unknown cells remain backend-internal. The
provided RViz configuration renders this topic with the OctoMap OccupancyGrid
display. `publish_period` limits snapshot export and publication; map updates
still run for every synchronized measurement group.

The i-Octree backend remains a point-cloud spatial index internally. Alongside
it, the mapper maintains a fixed-resolution voxel hit counter. A cell containing
`n` accepted points receives probability `1 - exp(-n / probability_scale)` and
is exported only when it meets both configured thresholds.

The i-Octree options are:

```yaml
ioctree:
  min_extent: 0.25
  bucket_size: 32
  downsample: true
  max_range: 50.0
  occupancy_resolution: 0.5
  min_points_per_voxel: 1
  occupancy_threshold: 0.5
  probability_scale: 1.0
```

`IOctreeMapper::Save()` writes a binary PCD file. The existing OctoMap backend
remains available and exports its occupied leaves through the same interface.


## Notes

**Ground-Truth Odometry for FRD (Body) in FLU**

![](imgs/FLU_ENU.png)

**Extrinsics for Robosense Airy LiDAR**

![](imgs/Airy_Extrinsics.png)

