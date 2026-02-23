#!/usr/bin/env python3
import argparse
import bisect
import math

import rosbag


def read_traj(bag_path, topic):
    traj = []
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            t = msg.header.stamp.to_sec()
            p = msg.pose.pose.position
            traj.append((t, (float(p.x), float(p.y), float(p.z))))
    traj.sort(key=lambda x: x[0])
    return traj


def nearest_index(times, t):
    idx = bisect.bisect_left(times, t)
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    if abs(times[idx] - t) < abs(times[idx - 1] - t):
        return idx
    return idx - 1


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def ate_rmse(ref_traj, est_traj, max_align_dt=0.05):
    ref_times = [x[0] for x in ref_traj]
    ref_pos = [x[1] for x in ref_traj]

    sq = []
    aligned = []
    for t, p_est in est_traj:
        i = nearest_index(ref_times, t)
        if abs(ref_times[i] - t) > max_align_dt:
            continue
        p_ref = ref_pos[i]
        e = vec_sub(p_est, p_ref)
        sq.append(vec_norm(e) ** 2)
        aligned.append((t, p_ref, p_est))

    if not sq:
        return float("nan"), aligned
    return math.sqrt(sum(sq) / len(sq)), aligned


def rpe_rmse(aligned, delta_t=1.0):
    if len(aligned) < 3:
        return float("nan")

    times = [x[0] for x in aligned]
    sq = []
    for i in range(len(aligned)):
        t_i = times[i]
        j = bisect.bisect_left(times, t_i + delta_t)
        if j >= len(aligned):
            continue
        p_ref_i = aligned[i][1]
        p_ref_j = aligned[j][1]
        p_est_i = aligned[i][2]
        p_est_j = aligned[j][2]
        d_ref = vec_sub(p_ref_j, p_ref_i)
        d_est = vec_sub(p_est_j, p_est_i)
        e = vec_sub(d_est, d_ref)
        sq.append(vec_norm(e) ** 2)

    if not sq:
        return float("nan")
    return math.sqrt(sum(sq) / len(sq))


def write_metric(path, name, value):
    with open(path, "w", encoding="utf-8") as f:
        if math.isnan(value):
            f.write(f"{name} rmse NA\n")
        else:
            f.write(f"{name} rmse {value:.6f}\n")


def main():
    parser = argparse.ArgumentParser(description="Evaluate est bag trajectory against baseline bag trajectory")
    parser.add_argument("--ref_bag", required=True)
    parser.add_argument("--est_bag", required=True)
    parser.add_argument("--topic", default="/tdar_lio_sam/mapping/odometry")
    parser.add_argument("--ape_out", required=True)
    parser.add_argument("--rpe_out", required=True)
    parser.add_argument("--align_dt", type=float, default=0.05)
    parser.add_argument("--delta_t", type=float, default=1.0)
    args = parser.parse_args()

    ref_traj = read_traj(args.ref_bag, args.topic)
    est_traj = read_traj(args.est_bag, args.topic)

    ape, aligned = ate_rmse(ref_traj, est_traj, max_align_dt=args.align_dt)
    rpe = rpe_rmse(aligned, delta_t=args.delta_t)

    write_metric(args.ape_out, "ape", ape)
    write_metric(args.rpe_out, "rpe", rpe)


if __name__ == "__main__":
    main()
