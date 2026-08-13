#!/usr/bin/env python3
"""Aggregate local-map maintenance benchmark CSV files."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, median


def percentile(values, probability):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def summarize(rows):
    result = {}
    for field in ("crop_wall_ms", "insert_wall_ms", "total_wall_ms"):
        values = [float(row[field]) for row in rows]
        result[field] = {
            "mean": mean(values),
            "median": median(values),
            "p95": percentile(values, 0.95),
            "p99": percentile(values, 0.99),
        }
    result["window_points"] = mean(float(row["window_points"]) for row in rows)
    result["endpoint_voxels"] = mean(float(row["endpoint_voxels"]) for row in rows)
    result["new_voxels"] = mean(float(row["new_voxels"]) for row in rows)
    result["final_active_voxels"] = int(rows[-1]["active_voxels"])
    result["peak_rss_delta"] = max(float(row["rss_delta_mib"]) for row in rows)
    result["final_elements"] = int(rows[-1]["map_elements"])
    return result


def metric_cell(summary, field):
    values = summary[field]
    return f'{values["mean"]:.3f} / {values["p95"]:.3f} / {values["p99"]:.3f}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    grouped = defaultdict(list)
    for path in args.csv:
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                grouped[row["backend"]].append(row)

    lines = [
        "# 20×20×8 m local-map maintenance summary",
        "",
        "Times are `mean / p95 / p99` in milliseconds. Input filtering, bag I/O,",
        "PointCloud2 conversion, and world-frame transformation are outside the timed region.",
        "OctoMap stores occupied endpoints only; ray casting and free-space updates are disabled.",
        "",
        "| Backend | Samples | Endpoint voxels/frame | New voxels/frame | Crop | Insert | Total | Peak RSS delta | Final active voxels | Diagnostic elements |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    summaries = {}
    for backend in ("octomap", "ioctree"):
        rows = grouped.get(backend, [])
        if not rows:
            continue
        rows.sort(key=lambda row: (int(row["repetition"]), int(row["frame"])))
        summary = summarize(rows)
        summaries[backend] = summary
        lines.append(
            f"| {backend} | {len(rows)} | {summary['endpoint_voxels']:.1f} | "
            f"{summary['new_voxels']:.1f} | "
            f"{metric_cell(summary, 'crop_wall_ms')} | "
            f"{metric_cell(summary, 'insert_wall_ms')} | "
            f"{metric_cell(summary, 'total_wall_ms')} | "
            f"{summary['peak_rss_delta']:.1f} MiB | "
            f"{summary['final_active_voxels']} | {summary['final_elements']} |"
        )

    if "octomap" in summaries and "ioctree" in summaries:
        octomap = summaries["octomap"]
        ioctree = summaries["ioctree"]
        lines.extend(["", "## Ratios (OctoMap / i-OctTree)", ""])
        for field, label in (
            ("crop_wall_ms", "Crop mean"),
            ("insert_wall_ms", "Insert mean"),
            ("total_wall_ms", "Total mean"),
        ):
            ratio = octomap[field]["mean"] / ioctree[field]["mean"]
            lines.append(f"- {label}: {ratio:.2f}x")

    lines.extend(
        [
            "",
            "`map_elements` is backend-specific and must not be compared as an",
            "equivalent memory or map-size metric. Use RSS delta for memory trends.",
        ]
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
