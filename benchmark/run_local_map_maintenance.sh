#!/usr/bin/env bash
# shellcheck disable=SC1091
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
set -euo pipefail

if [[ $# -lt 2 || $# -gt 5 ]]; then
  echo "Usage: $0 BAG_PATH OUTPUT_DIR [CPU] [FRAMES] [REPETITIONS]" >&2
  exit 2
fi

bag_path=$1
output_dir=$2
benchmark_cpu=${3:-2}
frames=${4:-600}
repetitions=${5:-5}
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd "${script_dir}/../.." && pwd)
benchmark_executable=${BENCHMARK_EXECUTABLE:-${workspace}/devel/lib/mapping/local_map_maintenance_benchmark}
analysis_script=${script_dir}/analyze_local_map_maintenance.py

if [[ ! -f "$bag_path" ]]; then
  echo "Bag file not found: $bag_path" >&2
  exit 2
fi
if [[ ! -x "$benchmark_executable" ]]; then
  echo "Benchmark executable not found: $benchmark_executable" >&2
  echo "Build it with: catkin_make -DCATKIN_ENABLE_TESTING=ON" >&2
  exit 2
fi

mkdir -p "$output_dir/raw"

for ((repetition = 0; repetition < repetitions; ++repetition)); do
  if (( repetition % 2 == 0 )); then
    backends=(octomap ioctree)
  else
    backends=(ioctree octomap)
  fi
  for backend in "${backends[@]}"; do
    output_file="$output_dir/raw/${backend}_r${repetition}.csv"
    echo "[20x20x8m repetition=$repetition] $backend"
    taskset -c "$benchmark_cpu" "$benchmark_executable" \
      --backend "$backend" --bag "$bag_path" --frames "$frames" \
      --window-xy 20 --z-below 6 --z-above 2 --resolution 0.5 \
      --repetition "$repetition" --output "$output_file"
  done
done

python3 "$analysis_script" "$output_dir"/raw/*.csv \
  --output "$output_dir/summary.md"

{
  echo "date: $(date --iso-8601=seconds)"
  echo "kernel: $(uname -srmo)"
  echo "cpu_affinity: $benchmark_cpu"
  echo "bag: $bag_path"
  echo "frames: $frames"
  echo "repetitions: $repetitions"
  echo "window: 20x20m, z=[origin-6m, origin+2m]"
  echo "resolution: 0.5m"
  echo "raycast: disabled"
  lscpu
} >"$output_dir/environment.txt"

echo "Summary: $output_dir/summary.md"
