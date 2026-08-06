#!/usr/bin/env bash
# shellcheck disable=SC1091
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 BAG_PATH OUTPUT_DIR [CPU]" >&2
  exit 2
fi

bag_path=$1
output_dir=$2
benchmark_cpu=${3:-2}
benchmark_executable=${BENCHMARK_EXECUTABLE:-/home/leaf/intern_ws/devel/lib/mapping/map_update_benchmark}
analysis_script=$(dirname "$0")/analyze_map_update.py

mkdir -p "$output_dir/raw"

run_one() {
  local backend=$1
  local scenario=$2
  local repetition=$3
  local points=$4
  local output_file="$output_dir/raw/${scenario}_${points}_${backend}_r${repetition}.csv"
  echo "[$scenario points=$points repetition=$repetition] $backend"
  if [[ $scenario == real ]]; then
    taskset -c "$benchmark_cpu" "$benchmark_executable" \
      --backend "$backend" --scenario real --bag "$bag_path" --frames 120 \
      --repetition "$repetition" --output "$output_file"
  else
    taskset -c "$benchmark_cpu" "$benchmark_executable" \
      --backend "$backend" --scenario synthetic --frames 60 --points "$points" \
      --repetition "$repetition" --output "$output_file"
  fi
}

for repetition in 0 1 2 3 4; do
  if (( repetition % 2 == 0 )); then
    backends=(octomap ioctree)
  else
    backends=(ioctree octomap)
  fi
  for points in 1000 5000 10000 50000; do
    for backend in "${backends[@]}"; do
      run_one "$backend" synthetic "$repetition" "$points"
    done
  done
  for backend in "${backends[@]}"; do
    run_one "$backend" real "$repetition" actual
  done
done

python3 "$analysis_script" "$output_dir"/raw/*.csv \
  --output "$output_dir/summary.md"

{
  echo "date: $(date --iso-8601=seconds)"
  echo "kernel: $(uname -srmo)"
  echo "cpu_affinity: $benchmark_cpu"
  echo "bag: $bag_path"
  echo "benchmark_executable: $benchmark_executable"
  lscpu
} >"$output_dir/environment.txt"

echo "Summary: $output_dir/summary.md"
