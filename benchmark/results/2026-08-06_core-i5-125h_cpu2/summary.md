# Map Update Benchmark Summary

| Scenario | Backend | Points/frame | Samples | Mean ms | Median ms | P95 ms | P99 ms | Stddev ms | ns/point | Thread CPU % | RSS growth MiB | Backend counter |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| real | ioctree | 34612 | 600 | 2.573 | 2.385 | 4.131 | 5.192 | 0.848 | 74.8 | 101.7 | 6.0 | 4112305 |
| real | octomap | 34612 | 600 | 29.919 | 18.864 | 83.817 | 93.516 | 29.059 | 877.7 | 101.2 | 5.3 | 111174 |
| synthetic | ioctree | 1000 | 300 | 0.239 | 0.227 | 0.387 | 0.631 | 0.132 | 238.7 | 97.9 | 1.9 | 60000 |
| synthetic | ioctree | 5000 | 300 | 1.277 | 1.191 | 2.144 | 3.037 | 0.465 | 255.5 | 99.9 | 6.3 | 300000 |
| synthetic | ioctree | 10000 | 300 | 2.644 | 2.615 | 4.037 | 4.446 | 0.731 | 264.4 | 99.9 | 10.0 | 600000 |
| synthetic | ioctree | 50000 | 300 | 18.482 | 18.639 | 28.197 | 41.124 | 6.173 | 369.6 | 100.0 | 41.6 | 3000000 |
| synthetic | octomap | 1000 | 300 | 2.842 | 2.623 | 4.162 | 4.998 | 0.610 | 2842.3 | 98.3 | 0.7 | 27751 |
| synthetic | octomap | 5000 | 300 | 14.030 | 13.471 | 18.716 | 20.392 | 2.212 | 2806.0 | 99.1 | 1.8 | 80442 |
| synthetic | octomap | 10000 | 300 | 28.033 | 26.984 | 35.980 | 41.103 | 4.099 | 2803.3 | 100.0 | 2.8 | 120315 |
| synthetic | octomap | 50000 | 300 | 185.167 | 180.175 | 218.281 | 272.309 | 22.023 | 3703.3 | 100.6 | 11.1 | 444074 |

## OctoMap / i-OctTree wall-time ratios

| Scenario | Points/frame | Mean ratio | P95 ratio |
|---|---:|---:|---:|
| real | 34612 | 11.63x | 20.29x |
| synthetic | 1000 | 11.91x | 10.76x |
| synthetic | 5000 | 10.98x | 8.73x |
| synthetic | 10000 | 10.60x | 8.91x |
| synthetic | 50000 | 10.02x | 7.74x |
