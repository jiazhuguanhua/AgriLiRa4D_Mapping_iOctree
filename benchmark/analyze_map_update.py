#!/usr/bin/env python3
"""Aggregate map_update_benchmark CSV files into a Markdown report."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, median, pstdev


def percentile(values, percent):
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def load_rows(paths):
    rows = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                for field in (
                    "repetition",
                    "frame",
                    "point_setting",
                    "input_points",
                    "within_range_points",
                    "map_elements",
                ):
                    row[field] = int(row[field])
                for field in (
                    "update_wall_ms",
                    "thread_cpu_ms",
                    "thread_cpu_percent",
                    "rss_mib",
                ):
                    row[field] = float(row[field])
                rows.append(row)
    return rows


def summarize(rows):
    times = [row["update_wall_ms"] for row in rows]
    cpu = [row["thread_cpu_percent"] for row in rows]
    point_times = [
        row["update_wall_ms"] * 1e6 / row["input_points"]
        for row in rows
        if row["input_points"]
    ]
    repetitions = defaultdict(list)
    for row in rows:
        repetitions[row["repetition"]].append(row)
    rss_growth = []
    final_elements = []
    for repetition_rows in repetitions.values():
        ordered = sorted(repetition_rows, key=lambda item: item["frame"])
        rss_growth.append(ordered[-1]["rss_mib"] - ordered[0]["rss_mib"])
        final_elements.append(ordered[-1]["map_elements"])
    return {
        "frames": len(rows),
        "mean": mean(times),
        "median": median(times),
        "p90": percentile(times, 90),
        "p95": percentile(times, 95),
        "p99": percentile(times, 99),
        "min": min(times),
        "max": max(times),
        "stddev": pstdev(times),
        "ns_point": mean(point_times),
        "cpu": mean(cpu),
        "rss_growth": mean(rss_growth),
        "final_elements": round(mean(final_elements)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = load_rows(args.csv)
    groups = defaultdict(list)
    for row in rows:
        groups[(row["scenario"], row["backend"], row["point_setting"])].append(row)

    summaries = {key: summarize(group) for key, group in groups.items()}
    lines = [
        "# Map Update Benchmark Summary",
        "",
        "| Scenario | Backend | Points/frame | Samples | Mean ms | Median ms | P95 ms | P99 ms | Stddev ms | ns/point | Thread CPU % | RSS growth MiB | Backend counter |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key in sorted(summaries):
        scenario, backend, point_setting = key
        item = summaries[key]
        points = round(mean(row["input_points"] for row in groups[key]))
        lines.append(
            f"| {scenario} | {backend} | {points} | {item['frames']} | "
            f"{item['mean']:.3f} | {item['median']:.3f} | {item['p95']:.3f} | "
            f"{item['p99']:.3f} | {item['stddev']:.3f} | {item['ns_point']:.1f} | "
            f"{item['cpu']:.1f} | {item['rss_growth']:.1f} | "
            f"{item['final_elements']} |"
        )

    lines.extend(["", "## OctoMap / i-OctTree wall-time ratios", ""])
    lines.append("| Scenario | Points/frame | Mean ratio | P95 ratio |")
    lines.append("|---|---:|---:|---:|")
    scenarios = sorted({(key[0], key[2]) for key in summaries})
    for scenario, point_setting in scenarios:
        octomap = summaries.get((scenario, "octomap", point_setting))
        ioctree = summaries.get((scenario, "ioctree", point_setting))
        if octomap and ioctree:
            points = round(
                mean(
                    row["input_points"]
                    for row in groups[(scenario, "octomap", point_setting)]
                )
            )
            lines.append(
                f"| {scenario} | {points} | {octomap['mean'] / ioctree['mean']:.2f}x | "
                f"{octomap['p95'] / ioctree['p95']:.2f}x |"
            )

    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
