#!/usr/bin/env python3
"""Offline-only B2 motion-evidence diagnostics.

This program never changes SLAM inputs, B2/B3/B4 decisions, or YAML values.
It reads existing CSVs and the active YAML solely to make the motion gates
explicit and reproducible.
"""
import argparse
import csv
import json
import math
import os
import statistics
from collections import Counter, defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


METRICS = [
    "candidate_count", "matched_count", "valid_count", "match_ratio",
    "valid_match_ratio", "bg_median", "bg_sigma", "raw_r3d_median",
    "norm_r3d_median", "high_ratio", "raw_minus_bg",
]

NUMERIC_FIELDS = [
    "candidate_count", "matched_count", "valid_count", "match_ratio",
    "valid_match_ratio", "bg_count", "bg_median", "bg_sigma",
    "raw_r3d_median", "norm_r3d_median", "high_ratio",
]


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def as_float(value, default=float("nan")):
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def as_int(value, default=-1):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low, high = int(math.floor(position)), int(math.ceil(position))
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def metric_values(rows, metric):
    values = []
    for row in rows:
        if metric == "raw_minus_bg":
            raw, bg = row["raw_r3d_median"], row["bg_median"]
            value = raw - bg if raw >= 0 and bg >= 0 else float("nan")
        else:
            value = row[metric]
        # Negative residual/background values are explicit B2 sentinel values.
        if math.isfinite(value) and (metric == "raw_minus_bg" or metric not in
                                     ("bg_median", "bg_sigma", "raw_r3d_median",
                                      "norm_r3d_median", "high_ratio") or value >= 0):
            values.append(value)
    return values


def quantile_row(sequence, reliability, metric, values):
    if not values:
        return [sequence, reliability, metric, 0] + [""] * 10
    return [
        sequence, reliability, metric, len(values), min(values), percentile(values, .10),
        percentile(values, .25), percentile(values, .50), percentile(values, .75),
        percentile(values, .90), percentile(values, .95), max(values),
        statistics.fmean(values), statistics.pstdev(values) if len(values) > 1 else 0.0,
    ]


def parse_yaml_scalars(path):
    result = {}
    with open(path, encoding="utf-8") as stream:
        for source in stream:
            line = source.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            result[key.strip()] = value.strip().strip('"').strip("'")
    return result


def required_thresholds(path):
    settings = parse_yaml_scalars(path)
    # Prefer the calibrated six-zone interface.  Retain legacy moving keys
    # for old YAML files so historical diagnostics remain reproducible.
    fields = {
        "raw_threshold_m": ("B2.Geometry.RawMovingThreshold", "B2.Geometry.Decision.MinRawMovingMeters"),
        "norm_threshold": ("B2.Geometry.NormMovingThreshold", "B2.Geometry.Decision.MovingThreshold"),
        "high_ratio_threshold": ("B2.Geometry.HighRatioMovingThreshold", "B2.Geometry.Decision.MinMovingFraction"),
    }
    result = {}
    missing = []
    for name, keys in fields.items():
        key = next((key for key in keys if key in settings), None)
        if key is None:
            missing.append(" or ".join(keys))
        else:
            result[name] = float(settings[key])
    if missing:
        raise RuntimeError("missing motion decision fields in YAML: " + ", ".join(missing))
    return result


def normalize_b2(row):
    renamed = {
        "bg_count": "background_sample_count",
        "bg_median": "background_median_r3d",
        "bg_sigma": "background_sigma_r3d",
        "raw_r3d_median": "median_r3d",
        "norm_r3d_median": "median_normalized_r3d",
        "high_ratio": "high_residual_ratio",
        "geometry_reason": "decision_reason",
    }
    normalized = dict(row)
    for target, source in renamed.items():
        normalized[target] = as_float(row.get(source, "")) if target not in ("geometry_reason",) else row.get(source, "")
    for field in ("candidate_count", "matched_count", "valid_count"):
        normalized[field] = as_float(row.get(field, ""))
    for field in ("match_ratio", "valid_match_ratio"):
        normalized[field] = as_float(row.get(field, ""))
    normalized["geometry_state"] = row.get("geometry_state", "")
    normalized["geometry_reliable"] = normalized["geometry_state"] in {
        "GEOM_STATIC_CANDIDATE", "GEOM_MOVING_CANDIDATE", "GEOM_UNCERTAIN"
    }
    return normalized


def with_gate(row, thresholds):
    raw, norm, high = row["raw_r3d_median"], row["norm_r3d_median"], row["high_ratio"]
    row = dict(row)
    row["raw_minus_bg"] = raw - row["bg_median"] if raw >= 0 and row["bg_median"] >= 0 else float("nan")
    row["pass_raw"] = raw >= thresholds["raw_threshold_m"]
    row["pass_norm"] = norm >= thresholds["norm_threshold"]
    row["pass_high_ratio"] = high >= thresholds["high_ratio_threshold"]
    failed = []
    if not row["pass_raw"]:
        failed.append("RAW")
    if not row["pass_norm"]:
        failed.append("NORM")
    if not row["pass_high_ratio"]:
        failed.append("HIGH_RATIO")
    row["motion_gate_failure_code"] = "PASS_ALL" if not failed else "FAIL_" + "_AND_".join(failed)
    return row


def write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def b4_active_rows(rows):
    return [row for row in rows if row.get("b3_reason") not in
            ("YOLO_MISSED_TTL_ACTIVE", "TRACK_TTL_EXPIRED")]


def write_top(path, rows):
    header = [
        "frame_id", "timestamp", "track_id", "instance_id", "candidate_count",
        "matched_count", "valid_count", "match_ratio", "valid_match_ratio", "bg_count",
        "bg_median", "bg_sigma", "raw_r3d_median", "norm_r3d_median", "high_ratio",
        "geometry_state", "geometry_reason", "raw_minus_bg", "pass_raw", "pass_norm",
        "pass_high_ratio", "motion_gate_failure_code",
    ]
    result = []
    for row in rows[:30]:
        result.append([row.get(key, "") for key in header])
    write_csv(path, header, result)


def plot_histogram(path, walking, sitting, metric, title, xlabel):
    plt.figure(figsize=(8, 5))
    for label, rows, color in (("walking", walking, "tab:red"), ("sitting", sitting, "tab:blue")):
        values = metric_values(rows, metric)
        if values:
            plt.hist(values, bins=40, density=False, histtype="step", linewidth=1.8,
                     label=f"{label} (n={len(values)})", color=color)
    plt.xlabel(xlabel)
    plt.ylabel("instance frames")
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def plot_scatter(path, walking, sitting, x_name, y_name, xlabel, ylabel):
    plt.figure(figsize=(7, 5))
    for label, rows, color in (("walking", walking, "tab:red"), ("sitting", sitting, "tab:blue")):
        pairs = [(row[x_name], row[y_name]) for row in rows
                 if math.isfinite(row[x_name]) and math.isfinite(row[y_name]) and
                 row[x_name] >= 0 and row[y_name] >= 0]
        if pairs:
            x, y = zip(*pairs)
            plt.scatter(x, y, s=10, alpha=.45, label=f"{label} (n={len(pairs)})", color=color)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(f"{ylabel} vs {xlabel}")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--walking-csv", required=True, help="walking B2 instance CSV")
    parser.add_argument("--sitting-csv", required=True, help="sitting B2 instance CSV")
    parser.add_argument("--walking-b3-csv", required=True)
    parser.add_argument("--sitting-b3-csv", required=True)
    parser.add_argument("--walking-b4-csv", required=True)
    parser.add_argument("--sitting-b4-csv", required=True)
    parser.add_argument("--config", required=True, help="actual YAML used for the run")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=False)
    thresholds = required_thresholds(args.config)
    datasets = {}
    for name, path, b3_path, b4_path in (
        ("walking", args.walking_csv, args.walking_b3_csv, args.walking_b4_csv),
        ("sitting", args.sitting_csv, args.sitting_b3_csv, args.sitting_b4_csv),
    ):
        rows = [with_gate(normalize_b2(row), thresholds) for row in read_csv(path)]
        datasets[name] = {"rows": rows, "b3": read_csv(b3_path), "b4": read_csv(b4_path),
                          "sources": {"b2": os.path.abspath(path), "b3": os.path.abspath(b3_path),
                                      "b4": os.path.abspath(b4_path)}}

    metadata = {"thresholds": thresholds, "config": os.path.abspath(args.config),
                "inputs": {name: data["sources"] for name, data in datasets.items()},
                "notes": {"reliable_definition": "GEOM_STATIC_CANDIDATE, GEOM_MOVING_CANDIDATE, or GEOM_UNCERTAIN",
                          "b4_ttl_rows": "Rows with YOLO_MISSED_TTL_ACTIVE or TRACK_TTL_EXPIRED are excluded from active-instance B4 counts."}}
    with open(os.path.join(args.output_dir, "analysis_metadata.json"), "w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)

    basic, reason_rows, b3b4_rows = {}, [], []
    quantile_rows = [["sequence", "geometry_reliable", "metric", "count", "min", "P10", "P25", "median", "P75", "P90", "P95", "max", "mean", "std"]]
    gate_count_rows = [["sequence", "scope", "category", "count", "ratio"]]
    gate_per_frame_rows = [["sequence", "frame_id", "timestamp", "track_id", "instance_id", "candidate_count", "matched_count", "valid_count", "match_ratio", "valid_match_ratio", "bg_count", "bg_median", "bg_sigma", "raw_r3d_median", "norm_r3d_median", "high_ratio", "raw_minus_bg", "geometry_state", "geometry_reason", "pass_raw_gate", "pass_norm_gate", "pass_high_ratio_gate", "pass_all", "motion_gate_failure_code"]]

    for name, data in datasets.items():
        rows, b3, b4 = data["rows"], data["b3"], b4_active_rows(data["b4"])
        reliable = [row for row in rows if row["geometry_reliable"]]
        unreliable = [row for row in rows if not row["geometry_reliable"]]
        reasons = Counter(row["geometry_reason"] for row in rows)
        states = Counter(row["geometry_state"] for row in rows)
        b3_person = [row for row in b3 if row.get("class_name", "person") == "person"]
        b3_counts = Counter(row.get("fusion_category", "") for row in b3_person)
        b4_states = Counter(row.get("current_state", "") for row in b4)
        transition_keys = ("P_TO_S", "P_TO_D", "S_TO_P", "D_TO_P", "S_STAY", "D_STAY", "UNRELIABLE_EVIDENCE_FREEZE")
        transitions = {key: sum(key in row.get("transition_reason", "") for row in b4) for key in transition_keys}
        basic[name] = {
            "instance_frames": len(rows), "geometry_reliable_frames": len(reliable),
            "geometry_reliable_ratio": len(reliable) / len(rows) if rows else 0.0,
            "geometry_unreliable_frames": len(unreliable),
            "geometry_unreliable_ratio": len(unreliable) / len(rows) if rows else 0.0,
            "geometry_states": dict(states), "geometry_reasons": dict(reasons),
            "b3_fusion_categories": dict(b3_counts), "b4_active_states": dict(b4_states),
            "b4_transitions": transitions,
        }
        for reason, count in sorted(reasons.items()):
            reason_rows.append([name, reason, count, count / len(rows) if rows else 0.0])
        for category, count in sorted(b3_counts.items()):
            b3b4_rows.append([name, "B3_FUSION_CATEGORY", category, count, count / len(b3_person) if b3_person else 0.0])
        for state, count in sorted(b4_states.items()):
            b3b4_rows.append([name, "B4_ACTIVE_STATE", state, count, count / len(b4) if b4 else 0.0])
        for transition, count in transitions.items():
            b3b4_rows.append([name, "B4_TRANSITION", transition, count, count / len(b4) if b4 else 0.0])
        for label, group in (("1", reliable), ("0", unreliable)):
            for metric in METRICS:
                quantile_rows.append(quantile_row(name, label, metric, metric_values(group, metric)))
        gated = reliable
        combinations = Counter()
        failures = Counter()
        for row in gated:
            flags = (row["pass_raw"], row["pass_norm"], row["pass_high_ratio"])
            combinations[{(True, False, False): "RAW_ONLY", (False, True, False): "NORM_ONLY",
                          (False, False, True): "HIGH_RATIO_ONLY", (True, True, False): "RAW_AND_NORM",
                          (True, False, True): "RAW_AND_HIGH_RATIO", (False, True, True): "NORM_AND_HIGH_RATIO",
                          (True, True, True): "PASS_ALL", (False, False, False): "FAIL_ALL"}[flags]] += 1
            for gate, flag in zip(("fail_raw_count", "fail_norm_count", "fail_high_ratio_count"), flags):
                if not flag:
                    failures[gate] += 1
            gate_per_frame_rows.append([name, row.get("frame_id", ""), row.get("timestamp", ""), row.get("track_id", ""), row.get("instance_id", ""), row["candidate_count"], row["matched_count"], row["valid_count"], row["match_ratio"], row["valid_match_ratio"], row["bg_count"], row["bg_median"], row["bg_sigma"], row["raw_r3d_median"], row["norm_r3d_median"], row["high_ratio"], row["raw_minus_bg"], row["geometry_state"], row["geometry_reason"], int(row["pass_raw"]), int(row["pass_norm"]), int(row["pass_high_ratio"]), int(all(flags)), row["motion_gate_failure_code"]])
        for category, count in sorted(combinations.items()):
            gate_count_rows.append([name, "geometry_reliable", category, count, count / len(gated) if gated else 0.0])
        for category, count in sorted(failures.items()):
            gate_count_rows.append([name, "geometry_reliable", category, count, count / len(gated) if gated else 0.0])

    with open(os.path.join(args.output_dir, "b2_basic_summary.json"), "w", encoding="utf-8") as stream:
        json.dump(basic, stream, indent=2, sort_keys=True)
    write_csv(os.path.join(args.output_dir, "b2_reason_counts.csv"), ["sequence", "geometry_reason", "count", "ratio"], reason_rows)
    write_csv(os.path.join(args.output_dir, "b3_b4_summary.csv"), ["sequence", "domain", "name", "count", "ratio"], b3b4_rows)
    write_csv(os.path.join(args.output_dir, "b2_metric_quantiles.csv"), quantile_rows[0], quantile_rows[1:])
    write_csv(os.path.join(args.output_dir, "b2_motion_gate_counts.csv"), gate_count_rows[0], gate_count_rows[1:])
    write_csv(os.path.join(args.output_dir, "b2_motion_gate_per_frame.csv"), gate_per_frame_rows[0], gate_per_frame_rows[1:])

    walking_reliable = [row for row in datasets["walking"]["rows"] if row["geometry_reliable"]]
    write_top(os.path.join(args.output_dir, "walking_top_raw.csv"), sorted(walking_reliable, key=lambda row: row["raw_r3d_median"], reverse=True))
    write_top(os.path.join(args.output_dir, "walking_top_norm.csv"), sorted(walking_reliable, key=lambda row: row["norm_r3d_median"], reverse=True))
    write_top(os.path.join(args.output_dir, "walking_top_high_ratio.csv"), sorted(walking_reliable, key=lambda row: row["high_ratio"], reverse=True))
    def distance_to_gate(row):
        return sum(max(0.0, threshold - value) / threshold for value, threshold in (
            (row["raw_r3d_median"], thresholds["raw_threshold_m"]),
            (row["norm_r3d_median"], thresholds["norm_threshold"]),
            (row["high_ratio"], thresholds["high_ratio_threshold"])))
    near = sorted(walking_reliable, key=distance_to_gate)
    write_top(os.path.join(args.output_dir, "walking_near_moving_gate.csv"), near)
    raw_only_fail = [row for row in walking_reliable if row["pass_raw"] and not (row["pass_norm"] and row["pass_high_ratio"])]
    write_top(os.path.join(args.output_dir, "walking_raw_pass_other_gate_fail.csv"), sorted(raw_only_fail, key=distance_to_gate))
    unreliable = [row for row in datasets["walking"]["rows"]
                  if row["geometry_reason"] in ("INSUFFICIENT_MATCHES", "LOW_MATCH_RATIO") and
                  (row["raw_r3d_median"] >= thresholds["raw_threshold_m"] or row["norm_r3d_median"] >= thresholds["norm_threshold"])]
    write_top(os.path.join(args.output_dir, "walking_high_residual_unreliable.csv"), sorted(unreliable, key=lambda row: max(row["raw_r3d_median"], row["norm_r3d_median"]), reverse=True))

    grid_rows = [["raw_threshold_m", "norm_threshold", "high_ratio_threshold", "walking_moving_count", "walking_moving_ratio", "sitting_moving_count", "sitting_moving_ratio", "walking_minus_sitting_count"]]
    rel = {name: [row for row in data["rows"] if row["geometry_reliable"]] for name, data in datasets.items()}
    for raw_threshold in (.015, .020, .025, .030):
        for norm_threshold in (1.5, 2.0, 2.5, 3.0, 3.5):
            for high_threshold in (.20, .30, .40, .50, .60):
                counts = {}
                for name in ("walking", "sitting"):
                    counts[name] = sum(row["raw_r3d_median"] >= raw_threshold and row["norm_r3d_median"] >= norm_threshold and row["high_ratio"] >= high_threshold for row in rel[name])
                grid_rows.append([raw_threshold, norm_threshold, high_threshold, counts["walking"], counts["walking"] / len(rel["walking"]) if rel["walking"] else 0.0, counts["sitting"], counts["sitting"] / len(rel["sitting"]) if rel["sitting"] else 0.0, counts["walking"] - counts["sitting"]])
    write_csv(os.path.join(args.output_dir, "b2_threshold_grid.csv"), grid_rows[0], grid_rows[1:])

    walking_rows, sitting_rows = datasets["walking"]["rows"], datasets["sitting"]["rows"]
    plot_histogram(os.path.join(args.output_dir, "raw_r3d_distribution.png"), walking_rows, sitting_rows, "raw_r3d_median", "B2 median raw 3-D residual", "median r3d (m)")
    plot_histogram(os.path.join(args.output_dir, "norm_r3d_distribution.png"), walking_rows, sitting_rows, "norm_r3d_median", "B2 median normalized 3-D residual", "median normalized r3d")
    plot_histogram(os.path.join(args.output_dir, "high_ratio_distribution.png"), walking_rows, sitting_rows, "high_ratio", "B2 high normalized-residual ratio", "high residual ratio")
    plot_histogram(os.path.join(args.output_dir, "bg_sigma_distribution.png"), walking_rows, sitting_rows, "bg_sigma", "B2 background MAD sigma", "background sigma (m)")
    plot_scatter(os.path.join(args.output_dir, "raw_vs_norm_scatter.png"), walking_rows, sitting_rows, "raw_r3d_median", "norm_r3d_median", "median r3d (m)", "median normalized r3d")
    plot_scatter(os.path.join(args.output_dir, "norm_vs_high_ratio_scatter.png"), walking_rows, sitting_rows, "norm_r3d_median", "high_ratio", "median normalized r3d", "high residual ratio")
    plot_scatter(os.path.join(args.output_dir, "match_ratio_vs_raw_scatter.png"), walking_rows, sitting_rows, "match_ratio", "raw_r3d_median", "match ratio", "median r3d (m)")


if __name__ == "__main__":
    main()
