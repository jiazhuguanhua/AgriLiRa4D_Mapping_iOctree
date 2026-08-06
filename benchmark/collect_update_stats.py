#!/usr/bin/env python3
"""Collect Mapping's OverlayText update statistics as machine-readable CSV."""

import argparse
import csv
import re
import threading

import rospy
from jsk_rviz_plugins.msg import OverlayText


PATTERNS = {
    "update_wall_ms": re.compile(r"Update wall\s*:\s*([0-9.]+)"),
    "thread_cpu_ms": re.compile(r"Update thread CPU\s*:\s*([0-9.]+)"),
    "thread_cpu_percent": re.compile(r"Update thread use\s*:\s*([0-9.]+)"),
    "process_cpu_percent": re.compile(r"Process CPU avg\s*:\s*([0-9.]+)"),
    "rss_mib": re.compile(r"RSS\s*:\s*([0-9.]+)"),
    "lidar_points": re.compile(r"Points\s*:\s*LiDAR\s+([0-9]+)"),
    "radar_points": re.compile(r"Radar\s+([0-9]+)"),
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rows = []
    complete = threading.Event()

    def callback(message):
        row = {}
        for name, pattern in PATTERNS.items():
            match = pattern.search(message.text)
            if not match:
                return
            row[name] = match.group(1)
        rows.append(row)
        if len(rows) >= args.count:
            complete.set()

    rospy.init_node("collect_update_stats", anonymous=True)
    subscriber = rospy.Subscriber(
        "/mapper_update_stats", OverlayText, callback, queue_size=1000
    )
    while not rospy.is_shutdown() and not complete.wait(0.1):
        pass
    subscriber.unregister()

    fieldnames = list(PATTERNS)
    with open(args.output, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows[: args.count])


if __name__ == "__main__":
    main()
