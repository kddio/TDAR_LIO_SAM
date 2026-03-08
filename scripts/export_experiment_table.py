#!/usr/bin/env python3
import argparse
import csv
import json
import os
import re


def parse_rmse(path):
    if not os.path.exists(path):
        return "NA"
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()
    m = re.search(r"rmse\s+([0-9.eE+-]+)", text)
    return m.group(1) if m else "NA"


def parse_diag_json(path):
    if not os.path.exists(path):
        return {"avg_compute_us": "NA", "avg_acc_residual": "NA", "avg_gyro_residual": "NA", "fallback_rate": "NA"}
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    overall = data.get("overall", {})
    return {
        "avg_compute_us": overall.get("avg_compute_us", "NA"),
        "avg_acc_residual": overall.get("avg_acc_residual", "NA"),
        "avg_gyro_residual": overall.get("avg_gyro_residual", "NA"),
        "fallback_rate": overall.get("fallback_rate", "NA"),
    }


def parse_cindex_json(path):
    if not os.path.exists(path):
        return {
            "avg_search_ms": "NA",
            "avg_bins_scanned": "NA",
            "avg_candidates_after_radius": "NA",
            "fallback_rate": "NA",
            "overflow_rate": "NA",
            "exact_ring_rate": "NA",
        }
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    overall = data.get("overall", {})
    return {
        "avg_search_ms": overall.get("avg_search_ms", "NA"),
        "avg_bins_scanned": overall.get("avg_bins_scanned", "NA"),
        "avg_candidates_after_radius": overall.get("avg_candidates_after_radius", "NA"),
        "fallback_rate": overall.get("fallback_rate", "NA"),
        "overflow_rate": overall.get("overflow_rate", "NA"),
        "exact_ring_rate": overall.get("exact_ring_rate", "NA"),
    }


def fmt(v, scale=1.0):
    if isinstance(v, str):
        return v
    return f"{v * scale:.6f}"


def main():
    parser = argparse.ArgumentParser(description="Collect ATE/RPE/TDAR cost into CSV and Markdown")
    parser.add_argument("--root", required=True, help="Experiment root directory")
    parser.add_argument("--output_csv", required=True, help="Output CSV path")
    parser.add_argument("--output_md", required=True, help="Output Markdown path")
    parser.add_argument("--cases", nargs="*", default=["baseline", "tdar_endpoint", "tdar_adaptive_gyro"])
    args = parser.parse_args()

    rows = []
    for case in args.cases:
        case_dir = os.path.join(args.root, case)
        ate_rmse = parse_rmse(os.path.join(case_dir, "ape.txt"))
        rpe_rmse = parse_rmse(os.path.join(case_dir, "rpe.txt"))
        diag = parse_diag_json(os.path.join(case_dir, "tdar_diag_stats.json"))
        cindex = parse_cindex_json(os.path.join(case_dir, "cindex_diag_stats.json"))

        row = {
            "case": case,
            "ate_rmse_m": ate_rmse,
            "rpe_rmse_m": rpe_rmse,
            "avg_compute_ms": fmt(diag["avg_compute_us"], scale=1e-3) if not isinstance(diag["avg_compute_us"], str) else diag["avg_compute_us"],
            "acc_residual": fmt(diag["avg_acc_residual"]) if not isinstance(diag["avg_acc_residual"], str) else diag["avg_acc_residual"],
            "gyro_residual": fmt(diag["avg_gyro_residual"]) if not isinstance(diag["avg_gyro_residual"], str) else diag["avg_gyro_residual"],
            "fallback_rate": fmt(diag["fallback_rate"]) if not isinstance(diag["fallback_rate"], str) else diag["fallback_rate"],
            "avg_cindex_search_ms": fmt(cindex["avg_search_ms"]) if not isinstance(cindex["avg_search_ms"], str) else cindex["avg_search_ms"],
            "avg_cindex_bins": fmt(cindex["avg_bins_scanned"]) if not isinstance(cindex["avg_bins_scanned"], str) else cindex["avg_bins_scanned"],
            "avg_cindex_candidates": fmt(cindex["avg_candidates_after_radius"]) if not isinstance(cindex["avg_candidates_after_radius"], str) else cindex["avg_candidates_after_radius"],
            "cindex_fallback_rate": fmt(cindex["fallback_rate"]) if not isinstance(cindex["fallback_rate"], str) else cindex["fallback_rate"],
            "cindex_overflow_rate": fmt(cindex["overflow_rate"]) if not isinstance(cindex["overflow_rate"], str) else cindex["overflow_rate"],
            "cindex_exact_ring_rate": fmt(cindex["exact_ring_rate"]) if not isinstance(cindex["exact_ring_rate"], str) else cindex["exact_ring_rate"],
        }
        rows.append(row)

    os.makedirs(os.path.dirname(args.output_csv), exist_ok=True)
    with open(args.output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "case",
                "ate_rmse_m",
                "rpe_rmse_m",
                "avg_compute_ms",
                "acc_residual",
                "gyro_residual",
                "fallback_rate",
                "avg_cindex_search_ms",
                "avg_cindex_bins",
                "avg_cindex_candidates",
                "cindex_fallback_rate",
                "cindex_overflow_rate",
                "cindex_exact_ring_rate",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    with open(args.output_md, "w", encoding="utf-8") as f:
        f.write("| Case | ATE RMSE (m) | RPE RMSE (m) | Avg TDAR Cost (ms) | Acc Residual | Gyro Residual | TDAR Fallback | Avg CIndex Search (ms) | Avg Bins | Avg Radius Cands | CIndex Fallback | CIndex Overflow | Exact Ring Rate |\n")
        f.write("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n")
        for r in rows:
            f.write(
                f"| {r['case']} | {r['ate_rmse_m']} | {r['rpe_rmse_m']} | {r['avg_compute_ms']} | {r['acc_residual']} | {r['gyro_residual']} | {r['fallback_rate']} | {r['avg_cindex_search_ms']} | {r['avg_cindex_bins']} | {r['avg_cindex_candidates']} | {r['cindex_fallback_rate']} | {r['cindex_overflow_rate']} | {r['cindex_exact_ring_rate']} |\n"
            )


if __name__ == "__main__":
    main()
