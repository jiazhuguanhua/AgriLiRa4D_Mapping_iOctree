# Map backend update benchmark

This benchmark compares only the wall and current-thread CPU time inside each
mapper backend's `Update()` call. Bag I/O, PointCloud2 conversion, world-frame
transformation, map export, ROS publication, and RViz are outside the measured
interval.

The real scenario takes LiDAR frames from `/rslidar_points` and the latest pose
from `/aircraft_pose_flu`. The synthetic scenario uses a deterministic seed and
tests 1k, 5k, 10k, and 50k points per frame. Both backends receive the same
world-frame cloud and origin in each matched run. Default project parameters
are used: OctoMap resolution 0.5 m, i-OctTree minimum extent 0.25 m, bucket size
32 with downsampling enabled, and maximum range 50 m.

Build and run from a catkin workspace:

```bash
catkin_make -DCATKIN_ENABLE_TESTING=ON
./src/AgriLiRa4D_Mapping_iOctree/benchmark/run_map_update_benchmark.sh \
  /path/to/dataset.bag /tmp/map-benchmark 2
```

Every repetition runs in a fresh process, and backend order alternates between
repetitions to reduce allocator-state and thermal-order bias. The third argument
pins the process to one logical CPU. Raw per-frame CSV files, environment
metadata, and a Markdown summary are written to the selected output directory.

`map_elements` is a backend diagnostic counter, not a cross-backend map-size
metric. In particular, upstream i-OctTree's `size()` counter can include points
that are later suppressed by leaf downsampling. Use RSS for memory comparison;
do not compare this counter directly with OctoMap's node count.

The comparison is intentionally an application-level backend comparison, not a
claim of equivalent map semantics: OctoMap ray-casts and updates free/occupied
probabilities, while i-OctTree stores incremental hit points.
