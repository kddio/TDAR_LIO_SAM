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
            "avg_compute_us": 0.0,
            "p95_compute_us": 0.0,
            "fallback_rate": 0.0,
            "avg_acc_residual": 0.0,
            "avg_gyro_residual": 0.0,
        }

    compute_us = [e["compute_us"] for e in entries]
    fallback = [e["fallback"] for e in entries]
    acc_res = [e["acc_residual"] for e in entries]
    gyro_res = [e["gyro_residual"] for e in entries]

    return {
        "count": len(entries),
        "avg_compute_us": sum(compute_us) / len(compute_us),
        "p95_compute_us": percentile(compute_us, 0.95),
        "fallback_rate": sum(fallback) / len(fallback),
        "avg_acc_residual": sum(acc_res) / len(acc_res),
        "avg_gyro_residual": sum(gyro_res) / len(gyro_res),
    }


def parse_bag(bag_path, topics):
    parsed = defaultdict(list)
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            if not hasattr(msg, "data") or len(msg.data) < 13:
                continue
            parsed[topic].append(
                {
                    "target_time": float(msg.data[0]),
                    "source_tag": int(msg.data[1]),
                    "state_high": bool(int(msg.data[2])),
                    "used_k": int(msg.data[3]),
                    "lambda_t": float(msg.data[4]),
                    "lambda_a": float(msg.data[5]),
                    "lambda_g": float(msg.data[6]),
                    "entropy": float(msg.data[7]),
                    "sum_w": float(msg.data[8]),
                    "fallback": float(msg.data[9]),
                    "compute_us": float(msg.data[10]),
                    "acc_residual": float(msg.data[11]),
                    "gyro_residual": float(msg.data[12]),
                }
            )
    return parsed


def main():
    parser = argparse.ArgumentParser(description="Export TDAR diagnostics statistics from rosbag")
    parser.add_argument("--bag", required=True, help="Recorded bag containing TDAR diagnostic topics")
    parser.add_argument("--output", required=True, help="Output JSON path")
    parser.add_argument(
        "--topics",
        nargs="*",
        default=[
            "/tdar_lio_sam/tdar/imu_preintegration_diag",
            "/tdar_lio_sam/tdar/image_projection_diag",
        ],
        help="TDAR diagnostic topics",
    )
    args = parser.parse_args()

    parsed = parse_bag(args.bag, args.topics)
    out = {"topics": {}, "overall": {}}

    overall_entries = []
    for topic in args.topics:
        entries = parsed.get(topic, [])
        out["topics"][topic] = summarize(entries)
        overall_entries.extend(entries)

    out["overall"] = summarize(overall_entries)

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()
