#!/usr/bin/env python3
import argparse
import json
import math
from collections import defaultdict

import rosbag


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    pos = q * (len(ordered) - 1)
    low = int(math.floor(pos))
    high = int(math.ceil(pos))
    if low == high:
        return ordered[low]
    frac = pos - low
    return ordered[low] * (1.0 - frac) + ordered[high] * frac


def summarize(entries):
    if not entries:
        return {
            "count": 0,
            "avg_bins_scanned": 0.0,
            "avg_candidates_before_radius": 0.0,
            "avg_candidates_after_radius": 0.0,
            "fallback_rate": 0.0,
            "overflow_rate": 0.0,
            "exact_ring_rate": 0.0,
            "avg_search_ms": 0.0,
            "p95_search_ms": 0.0,
        }

    avg_bins = [e["avg_bins_scanned"] for e in entries]
    before_radius = [e["avg_candidates_before_radius"] for e in entries]
    after_radius = [e["avg_candidates_after_radius"] for e in entries]
    fallback = [e["fallback_rate"] for e in entries]
    overflow = [e["overflow_rate"] for e in entries]
    exact_ring = [e["exact_ring_rate"] for e in entries]
    search_ms = [e["avg_search_ms"] for e in entries]

    return {
        "count": len(entries),
        "avg_bins_scanned": sum(avg_bins) / len(avg_bins),
        "avg_candidates_before_radius": sum(before_radius) / len(before_radius),
        "avg_candidates_after_radius": sum(after_radius) / len(after_radius),
        "fallback_rate": sum(fallback) / len(fallback),
        "overflow_rate": sum(overflow) / len(overflow),
        "exact_ring_rate": sum(exact_ring) / len(exact_ring),
        "avg_search_ms": sum(search_ms) / len(search_ms),
        "p95_search_ms": percentile(search_ms, 0.95),
    }


def parse_bag(bag_path, topics):
    parsed = defaultdict(list)
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            if not hasattr(msg, "data") or len(msg.data) < 9:
                continue

            branch_tag = int(msg.data[1])
            branch_name = "corner" if branch_tag == 0 else "surf"
            parsed[branch_name].append(
                {
                    "time": float(msg.data[0]),
                    "query_count": float(msg.data[2]),
                    "avg_bins_scanned": float(msg.data[3]),
                    "avg_candidates_before_radius": float(msg.data[4]),
                    "avg_candidates_after_radius": float(msg.data[5]),
                    "fallback_rate": float(msg.data[6]),
                    "exact_ring_rate": float(msg.data[7]),
                    "avg_search_ms": float(msg.data[8]),
                    "overflow_rate": float(msg.data[9]) if len(msg.data) > 9 else 0.0,
                }
            )
    return parsed


def main():
    parser = argparse.ArgumentParser(description="Export C-Index search diagnostics from rosbag")
    parser.add_argument("--bag", required=True, help="Recorded bag containing C-Index diagnostic topic")
    parser.add_argument("--output", required=True, help="Output JSON path")
    parser.add_argument(
        "--topics",
        nargs="*",
        default=["/tdar_lio_sam/cindex/search_diag"],
        help="C-Index diagnostic topics",
    )
    args = parser.parse_args()

    parsed = parse_bag(args.bag, args.topics)
    out = {
        "branches": {
            "corner": summarize(parsed.get("corner", [])),
            "surf": summarize(parsed.get("surf", [])),
        }
    }
    out["overall"] = summarize(parsed.get("corner", []) + parsed.get("surf", []))

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()
