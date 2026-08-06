#!/usr/bin/env python3
"""Render the map backend benchmark as a publication-ready static figure."""

import argparse
import csv
import glob
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean

import matplotlib.pyplot as plt


OCTOMAP = "#EA580C"
IOCTREE = "#2563EB"
INK = "#172033"
GRID = "#D9DEE7"
MUTED = "#667085"


def percentile(values, percent):
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def load_rows(results_dir):
    rows = []
    for file_name in glob.glob(str(results_dir / "raw" / "*.csv")):
        with open(file_name, newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                row["frame"] = int(row["frame"])
                row["point_setting"] = int(row["point_setting"])
                row["update_wall_ms"] = float(row["update_wall_ms"])
                rows.append(row)
    if not rows:
        raise RuntimeError(f"no benchmark CSV files found under {results_dir / 'raw'}")
    return rows


def style_axis(axis):
    axis.set_facecolor("white")
    axis.grid(True, color=GRID, linewidth=0.8, alpha=0.75)
    axis.set_axisbelow(True)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.spines["left"].set_color(GRID)
    axis.spines["bottom"].set_color(GRID)
    axis.tick_params(colors=MUTED, labelsize=9)
    axis.title.set_color(INK)


def render(rows, png_path, svg_path):
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.labelcolor": INK,
            "text.color": INK,
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
            "svg.fonttype": "none",
        }
    )
    figure = plt.figure(figsize=(15, 9), constrained_layout=False)
    grid = figure.add_gridspec(2, 2, height_ratios=[1.0, 1.35], hspace=0.42, wspace=0.28)
    scale_axis = figure.add_subplot(grid[0, 0])
    phase_axis = figure.add_subplot(grid[0, 1])
    trend_axis = figure.add_subplot(grid[1, :])
    for axis in (scale_axis, phase_axis, trend_axis):
        style_axis(axis)

    synthetic = [row for row in rows if row["scenario"] == "synthetic"]
    for backend, color, marker, display_name in (
        ("octomap", OCTOMAP, "o", "OctoMap"),
        ("ioctree", IOCTREE, "s", "i-OctTree"),
    ):
        points = sorted({row["point_setting"] for row in synthetic})
        means = []
        p95s = []
        for point_count in points:
            values = [
                row["update_wall_ms"]
                for row in synthetic
                if row["backend"] == backend and row["point_setting"] == point_count
            ]
            means.append(mean(values))
            p95s.append(percentile(values, 95))
        scale_axis.plot(
            points,
            means,
            color=color,
            marker=marker,
            linewidth=2.3,
            markersize=6,
            label=f"{display_name} mean",
        )
        scale_axis.plot(
            points,
            p95s,
            color=color,
            marker=marker,
            linewidth=1.5,
            linestyle="--",
            markersize=4,
            alpha=0.78,
            label=f"{display_name} P95",
        )
    scale_axis.axhline(100, color=INK, linewidth=1.2, linestyle=":", label="10 Hz budget")
    scale_axis.set_xscale("log")
    scale_axis.set_yscale("log")
    scale_axis.set_xticks([1000, 5000, 10000, 50000])
    scale_axis.set_xticklabels(["1k", "5k", "10k", "50k"])
    scale_axis.set_xlabel("Input points per frame")
    scale_axis.set_ylabel("Update wall time (ms, log scale)")
    scale_axis.set_title("Synthetic point-count scaling", loc="left", fontweight="bold")
    scale_axis.legend(frameon=False, fontsize=8, ncol=1, loc="upper left")

    real = [row for row in rows if row["scenario"] == "real"]
    phase_ranges = [(0, 39), (40, 79), (80, 119)]
    phase_names = ["Early\n0–39", "Middle\n40–79", "Late\n80–119"]
    ratios = []
    for start, end in phase_ranges:
        octomap_values = [
            row["update_wall_ms"]
            for row in real
            if row["backend"] == "octomap" and start <= row["frame"] <= end
        ]
        ioctree_values = [
            row["update_wall_ms"]
            for row in real
            if row["backend"] == "ioctree" and start <= row["frame"] <= end
        ]
        ratios.append(mean(octomap_values) / mean(ioctree_values))
    bars = phase_axis.bar(
        phase_names,
        ratios,
        color=IOCTREE,
        edgecolor="#174EA6",
        linewidth=0.8,
        width=0.62,
    )
    phase_axis.bar_label(bars, labels=[f"{value:.2f}x" for value in ratios], padding=4, fontsize=10)
    phase_axis.set_ylim(0, 23)
    phase_axis.set_ylabel("Mean speedup (OctoMap / i-OctTree)")
    phase_axis.set_title("Speedup as the real map grows", loc="left", fontweight="bold")
    phase_axis.grid(axis="x", visible=False)

    frame_groups = defaultdict(list)
    for row in real:
        frame_groups[(row["backend"], row["frame"])].append(row["update_wall_ms"])
    frames = sorted({row["frame"] for row in real})
    for backend, color, display_name in (
        ("octomap", OCTOMAP, "OctoMap"),
        ("ioctree", IOCTREE, "i-OctTree"),
    ):
        center = [mean(frame_groups[(backend, frame)]) for frame in frames]
        low = [percentile(frame_groups[(backend, frame)], 10) for frame in frames]
        high = [percentile(frame_groups[(backend, frame)], 90) for frame in frames]
        trend_axis.fill_between(frames, low, high, color=color, alpha=0.13, linewidth=0)
        trend_axis.plot(frames, center, color=color, linewidth=2.0, label=f"{display_name} mean")
    trend_axis.axhline(100, color=INK, linewidth=1.3, linestyle=":", label="100 ms / 10 Hz budget")
    trend_axis.text(118, 104, "real-time budget", ha="right", va="bottom", fontsize=9, color=INK)
    trend_axis.set_yscale("log")
    trend_axis.set_ylim(1, 140)
    trend_axis.set_xlim(0, 119)
    trend_axis.set_xlabel("Real LiDAR frame index")
    trend_axis.set_ylabel("Update wall time (ms, log scale)")
    trend_axis.set_title(
        "Real-data update time during incremental map growth",
        loc="left",
        fontweight="bold",
    )
    trend_axis.legend(frameon=False, fontsize=9, ncol=3, loc="upper left")

    figure.suptitle(
        "OctoMap vs i-OctTree — Mapper Update Performance",
        x=0.065,
        y=0.975,
        ha="left",
        fontsize=18,
        fontweight="bold",
        color=INK,
    )
    figure.text(
        0.065,
        0.94,
        "Release build · CPU 2 affinity · 5 independent runs · Update() only · AgriLiRa4D NJHillB01",
        ha="left",
        fontsize=10,
        color=MUTED,
    )
    figure.text(
        0.065,
        0.018,
        "Shaded bands: per-frame P10–P90 across 5 runs. Map semantics differ: OctoMap updates free/occupied probabilities; i-OctTree indexes hit points.",
        ha="left",
        fontsize=9,
        color=MUTED,
    )
    figure.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.10)
    png_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(png_path, dpi=180, bbox_inches="tight")
    figure.savefig(svg_path, bbox_inches="tight")
    plt.close(figure)
    svg_text = svg_path.read_text(encoding="utf-8")
    svg_path.write_text(
        "\n".join(line.rstrip() for line in svg_text.splitlines()) + "\n",
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--png", type=Path, required=True)
    parser.add_argument("--svg", type=Path, required=True)
    args = parser.parse_args()
    render(load_rows(args.results_dir), args.png, args.svg)


if __name__ == "__main__":
    main()
