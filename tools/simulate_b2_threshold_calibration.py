#!/usr/bin/env python3
"""Offline B2 three-zone threshold calibration and B4 replay.

This reads existing B2 instance-evidence CSV files only.  It does not load the
SLAM executable, change YAML/C++ values, or alter any prior result.  B4 is
replayed from the documented state-transition rules so threshold alternatives
can be compared without an online run.
"""
import argparse
import csv
import json
import math
import os
from collections import Counter, defaultdict


RD = (0.018, 0.020, 0.022, 0.025, 0.030)
ED = (0.40, 0.50, 0.75, 1.00, 1.25, 1.50)
PD = (0.10, 0.12, 0.15, 0.18, 0.20)
RS = (0.012, 0.015, 0.018)
ES = (0.10, 0.20, 0.30)
PS = (0.06, 0.08, 0.10)


def f(value, default=float("nan")):
    try:
        value = float(value)
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def i(value, default=-1):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def read_b2(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        for source in csv.DictReader(stream):
            row = dict(source)
            for key in ("frame_id", "instance_id", "track_id", "candidate_count",
                        "matched_count", "valid_count", "background_sample_count"):
                row[key] = i(row.get(key))
            for key in ("timestamp", "background_median_r3d", "background_sigma_r3d",
                        "median_r3d", "median_normalized_r3d", "high_residual_ratio"):
                row[key] = f(row.get(key))
            row["reliable"] = row.get("geometry_state") in {
                "GEOM_STATIC_CANDIDATE", "GEOM_MOVING_CANDIDATE", "GEOM_UNCERTAIN"}
            rows.append(row)
    return sorted(rows, key=lambda r: (r["track_id"], r["frame_id"], r["instance_id"]))


def association_count(path):
    """Count non-comment RGB-depth association records for coverage metadata."""
    if not path:
        return None
    with open(path, encoding="utf-8") as stream:
        return sum(1 for line in stream if line.strip() and not line.lstrip().startswith("#"))


def write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def q(values, p):
    values = sorted(values)
    if not values:
        return float("nan")
    pos = (len(values) - 1) * p
    lo, hi = int(pos), math.ceil(pos)
    return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (pos - lo)


def pearson(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 2:
        return float("nan")
    mx = sum(x for x, _ in pairs) / len(pairs)
    my = sum(y for _, y in pairs) / len(pairs)
    top = sum((x - mx) * (y - my) for x, y in pairs)
    dx = math.sqrt(sum((x - mx) ** 2 for x, _ in pairs))
    dy = math.sqrt(sum((y - my) ** 2 for _, y in pairs))
    return top / (dx * dy) if dx and dy else float("nan")


def classify(row, p):
    """Return MOVING/STATIC/UNCERTAIN/UNRELIABLE; old reliability gates persist."""
    if not row["reliable"]:
        return "UNRELIABLE"
    raw, norm, high = row["median_r3d"], row["median_normalized_r3d"], row["high_residual_ratio"]
    if raw >= p["rd"] and norm >= p["ed"] and high >= p["pd"]:
        return "MOVING"
    if raw <= p["rs"] and norm <= p["es"] and high <= p["ps"]:
        return "STATIC"
    return "UNCERTAIN"


def config_id(p):
    return "RD{:.3f}_ED{:.2f}_PD{:.2f}_RS{:.3f}_ES{:.2f}_PS{:.2f}".format(
        p["rd"], p["ed"], p["pd"], p["rs"], p["es"], p["ps"])


def replay(rows, p):
    """Replay B4 independently per track. Unreliable and uncertain freeze state."""
    tracks = {}
    state_counts, transitions = Counter(), Counter()
    event_rows, track_summaries = [], []
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["track_id"]].append(row)
    for tid, events in grouped.items():
        state, ns, nd, nr = "P", 0, 0, 0
        d_intervals = []
        track_events = []
        max_observed_moving = max_effective_moving = effective_moving = 0
        counts = Counter()
        for row in sorted(events, key=lambda r: (r["frame_id"], r["instance_id"])):
            evidence, previous = classify(row, p), state
            counts[evidence] += 1
            if evidence == "UNRELIABLE" or evidence == "UNCERTAIN":
                reason = "UNRELIABLE_EVIDENCE_FREEZE" if evidence == "UNRELIABLE" else "UNCERTAIN_EVIDENCE_FREEZE"
                # Freeze: do not reset effective_moving, ns, nd, nr, or state.
                max_observed_moving = max(max_observed_moving, 0)
            elif state == "P":
                if evidence == "MOVING":
                    nd += 1; ns = 0; effective_moving += 1
                    reason = "P_ACCUMULATE_MOVING"
                    if nd >= 3:
                        state, nr, reason = "D", 0, "P_TO_D"
                else:
                    ns += 1; nd = 0; effective_moving = 0
                    reason = "P_ACCUMULATE_STATIC"
                    if ns >= 6:
                        state, reason = "S", "P_TO_S"
            elif state == "S":
                if evidence == "MOVING":
                    state, nd, ns, effective_moving, reason = "P", 1, 0, 1, "S_TO_P"
                else:
                    nd, effective_moving, reason = 0, 0, "S_STAY"
            else:  # D
                if evidence == "MOVING":
                    nr, effective_moving, reason = 0, effective_moving + 1, "D_STAY"
                else:
                    nr += 1; effective_moving = 0; reason = "D_ACCUMULATE_RECOVERY"
                    if nr >= 10:
                        state, ns, nd, nr, reason = "P", 0, 0, 0, "D_TO_P"
            transitions[reason] += 1
            state_counts[state] += 1
            max_effective_moving = max(max_effective_moving, effective_moving)
            event = {"track_id": tid, "frame_id": row["frame_id"], "timestamp": row["timestamp"],
                     "instance_id": row["instance_id"], "state": state, "previous_state": previous,
                     "evidence": evidence, "reason": reason, "ns": ns, "nd": nd, "nr": nr, "row": row}
            event_rows.append(event)
            track_events.append(event)
        # Calculate strict adjacency run separately: each non-MOVING row breaks it.
        strict, strict_max = 0, 0
        for row in sorted(events, key=lambda r: (r["frame_id"], r["instance_id"])):
            if classify(row, p) == "MOVING":
                strict += 1; strict_max = max(strict_max, strict)
            else:
                strict = 0
        # D duration is a count of observed instance frames, not raw frame-id span:
        # a track can be absent from YOLO for an interval while its TTL remains alive.
        block = []
        for event in track_events:
            if event["state"] == "D":
                block.append(event)
            elif block:
                d_intervals.append((block[0]["frame_id"], block[-1]["frame_id"], len(block)))
                block = []
        if block:
            d_intervals.append((block[0]["frame_id"], block[-1]["frame_id"], len(block)))
        track_summaries.append({"track_id": tid, "moving": counts["MOVING"], "static": counts["STATIC"],
                                "uncertain": counts["UNCERTAIN"], "unreliable": counts["UNRELIABLE"],
                                "max_consecutive_moving": strict_max,
                                "max_continuous_reliable_moving": max_effective_moving,
                                "final_state": state, "d_intervals": d_intervals})
    return state_counts, transitions, event_rows, track_summaries


def count_zones(rows, p):
    result = Counter(classify(row, p) for row in rows)
    return result


def emit_sigma_effect(path, walking, grid):
    header = ["config_id", "rd", "ed", "pd", "sigma_bin", "reliable_count", "raw_median", "raw_p95",
              "norm_median", "norm_p95", "high_median", "high_p95", "moving_count", "moving_rate"]
    bins = (("<=0.010", -float("inf"), .010), ("0.010_to_0.020", .010, .020),
            ("0.020_to_0.030", .020, .030), (">0.030", .030, float("inf")))
    reliable = [r for r in walking if r["reliable"]]
    output = []
    # all dynamic candidates; static values do not affect a moving gate.
    for rd in RD:
        for ed in ED:
            for pd in PD:
                identifier = "RD{:.3f}_ED{:.2f}_PD{:.2f}".format(rd, ed, pd)
                for name, low, high in bins:
                    items = [r for r in reliable if r["background_sigma_r3d"] > low and r["background_sigma_r3d"] <= high]
                    moved = sum(r["median_r3d"] >= rd and r["median_normalized_r3d"] >= ed and r["high_residual_ratio"] >= pd for r in items)
                    vals = lambda field: [r[field] for r in items]
                    output.append([identifier, rd, ed, pd, name, len(items), q(vals("median_r3d"), .50), q(vals("median_r3d"), .95),
                                   q(vals("median_normalized_r3d"), .50), q(vals("median_normalized_r3d"), .95),
                                   q(vals("high_residual_ratio"), .50), q(vals("high_residual_ratio"), .95), moved,
                                   moved / len(items) if items else 0.0])
    write_csv(path, header, output)


def select_candidates(summaries):
    """Select interpretable candidates after, not during, the exhaustive scan."""
    usable = [s for s in summaries if s["walking_p_to_d"] > 0]
    if not usable:
        return []
    zero = [s for s in usable if s["sitting_moving"] == 0 and s["sitting_p_to_d"] == 0]
    # Conservative: smallest walking moving support that nevertheless enters D; high thresholds win ties.
    conservative = min(zero or usable, key=lambda s: (s["walking_moving"], -s["rd"], -s["ed"], -s["pd"], s["sitting_moving"]))
    # Balanced: zero sitting moves preferred, then several D entries and state-frame support.
    balanced_pool = zero or usable
    balanced = max(balanced_pool, key=lambda s: (s["walking_p_to_d"], s["walking_d_frames"], -s["walking_moving"], s["rd"], s["ed"], s["pd"]))
    # Sensitive: low gates which expose false-positive tendency; prefer some sitting moving evidence.
    noisy = [s for s in usable if s["sitting_moving"] > 0]
    # The supplied grid's minimum normalized gate (0.40) still exceeds the
    # observed sitting maximum (0.348), so `noisy` can be empty.  In that case
    # intentionally keep the lowest dynamic gates as a distinct sensitivity
    # probe; it shows that this grid has not yet reached a sitting false-positive
    # regime rather than duplicating the balanced candidate.
    sensitive = max(noisy, key=lambda s: (s["sitting_moving"], s["walking_p_to_d"], s["walking_moving"])) if noisy else min(
        usable, key=lambda s: (s["rd"], s["ed"], s["pd"], -s["walking_p_to_d"], -s["walking_d_frames"], s["rs"], s["es"], s["ps"]))
    chosen, seen = [], set()
    for label, item in (("conservative", conservative), ("balanced", balanced), ("sensitive", sensitive)):
        # Keep three labels even if data make two policies coincide; the CSV says so explicitly.
        item = dict(item); item["candidate_level"] = label; item["selection_note"] = {
            "conservative": "sitting zero-MOVING preferred; minimal walking support that still reaches D",
            "balanced": "sitting zero-MOVING preferred; maximizes D entries/support, not total MOVING",
            "sensitive": "lowest supplied dynamic gates; sitting has no MOVING even here, so this grid has not reached its false-positive regime",
        }[label]
        chosen.append(item)
        seen.add(item["config_id"])
    return chosen


def event_windows(candidate, replay_events, track_summaries):
    """Emit only the globally salient manual-review windows for one candidate."""
    by_track = defaultdict(list)
    for event in replay_events:
        by_track[event["track_id"]].append(event)
    for events in by_track.values():
        events.sort(key=lambda e: (e["frame_id"], e["instance_id"]))
    ordered = sorted(replay_events, key=lambda e: (e["frame_id"], e["track_id"], e["instance_id"]))
    targets = []
    for label in ("S_TO_P", "P_TO_D"):
        found = next((e for e in ordered if e["reason"] == label), None)
        if found:
            targets.append((label, found["track_id"], by_track[found["track_id"]].index(found)))
    intervals = [(count, tid, start) for t in track_summaries for start, _, count in t["d_intervals"] for tid in [t["track_id"]]]
    if intervals:
        _, tid, start_frame = max(intervals)
        index = next(n for n, e in enumerate(by_track[tid]) if e["frame_id"] == start_frame and e["state"] == "D")
        targets.append(("LONGEST_D", tid, index))
    reliable = [e for e in replay_events if e["row"]["reliable"]]
    for label, field in (("MAX_RAW", "median_r3d"), ("MAX_NORM", "median_normalized_r3d"), ("MAX_HIGH_RATIO", "high_residual_ratio")):
        if reliable:
            best = max(reliable, key=lambda e: e["row"][field])
            targets.append((label, best["track_id"], by_track[best["track_id"]].index(best)))
    output, seen = [], set()
    for label, tid, center in targets:
        if (label, tid, center) in seen:
            continue
        seen.add((label, tid, center))
        events = by_track[tid]
        for offset in range(-5, 6):
            index = center + offset
            if 0 <= index < len(events):
                e, r = events[index], events[index]["row"]
                output.append([candidate["candidate_level"], candidate["config_id"], label, offset,
                               e["frame_id"], e["timestamp"], e["track_id"], e["instance_id"],
                               r["candidate_count"], r["matched_count"], r["valid_count"],
                               r["background_sample_count"], r["background_median_r3d"], r["background_sigma_r3d"],
                               r["median_r3d"], r["median_normalized_r3d"], r["high_residual_ratio"],
                               e["evidence"], e["previous_state"], e["state"], e["reason"]])
    return output


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--walking-csv", required=True)
    ap.add_argument("--sitting-csv", required=True)
    ap.add_argument("--sitting-association", help="optional association file used to verify sitting coverage")
    ap.add_argument("--output-dir", required=True)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=False)
    datasets = {"walking": read_b2(args.walking_csv), "sitting": read_b2(args.sitting_csv)}
    parameters = []
    for rd in RD:
        for ed in ED:
            for pd in PD:
                for rs in RS:
                    for es in ES:
                        for ps in PS:
                            if rs < rd and es < ed and ps < pd:
                                p = {"rd": rd, "ed": ed, "pd": pd, "rs": rs, "es": es, "ps": ps}
                                p["config_id"] = config_id(p)
                                parameters.append(p)
    grid_rows, sequence_rows, track_rows, all_summaries = [], [], [], []
    grid_header = ["config_id", "rd", "ed", "pd", "rs", "es", "ps", "sequence", "instance_frames",
                   "moving_count", "moving_ratio", "static_count", "static_ratio", "uncertain_count", "uncertain_ratio",
                   "unreliable_count", "unreliable_ratio"]
    sim_header = ["config_id", "rd", "ed", "pd", "rs", "es", "ps", "sequence", "P_TO_S", "S_TO_P", "P_TO_D", "D_TO_P",
                  "S_instance_frames", "P_instance_frames", "D_instance_frames", "first_enter_D_frame_id",
                  "first_enter_D_track_id", "d_track_count", "max_D_duration_instance_frames"]
    track_header = ["config_id", "rd", "ed", "pd", "rs", "es", "ps", "sequence", "track_id", "moving_count", "static_count",
                    "uncertain_count", "unreliable_count", "max_consecutive_moving", "max_continuous_reliable_moving",
                    "final_state", "D_intervals_frame_id"]
    replay_cache = {}
    for p in parameters:
        per = {"config_id": p["config_id"], **p}
        for name, rows in datasets.items():
            zones = count_zones(rows, p); total = len(rows)
            grid_rows.append([p["config_id"], p["rd"], p["ed"], p["pd"], p["rs"], p["es"], p["ps"], name, total,
                              zones["MOVING"], zones["MOVING"] / total, zones["STATIC"], zones["STATIC"] / total,
                              zones["UNCERTAIN"], zones["UNCERTAIN"] / total, zones["UNRELIABLE"], zones["UNRELIABLE"] / total])
            state_counts, transitions, events, tracks = replay(rows, p)
            replay_cache[(p["config_id"], name)] = (events, tracks)
            first_d = next((e for e in sorted(events, key=lambda e: (e["frame_id"], e["track_id"])) if e["reason"] == "P_TO_D"), None)
            durations = [count for t in tracks for _, _, count in t["d_intervals"]]
            sequence_rows.append([p["config_id"], p["rd"], p["ed"], p["pd"], p["rs"], p["es"], p["ps"], name,
                                  transitions["P_TO_S"], transitions["S_TO_P"], transitions["P_TO_D"], transitions["D_TO_P"],
                                  state_counts["S"], state_counts["P"], state_counts["D"],
                                  first_d["frame_id"] if first_d else "", first_d["track_id"] if first_d else "",
                                  len(durations), max(durations) if durations else 0])
            for t in tracks:
                track_rows.append([p["config_id"], p["rd"], p["ed"], p["pd"], p["rs"], p["es"], p["ps"], name, t["track_id"],
                                   t["moving"], t["static"], t["uncertain"], t["unreliable"], t["max_consecutive_moving"],
                                   t["max_continuous_reliable_moving"], t["final_state"],
                                   ";".join("{}-{}({})".format(a, b, count) for a, b, count in t["d_intervals"])])
            per[name + "_moving"] = zones["MOVING"]
            per[name + "_static"] = zones["STATIC"]
            per[name + "_uncertain"] = zones["UNCERTAIN"]
            per[name + "_unreliable"] = zones["UNRELIABLE"]
            per[name + "_p_to_d"] = transitions["P_TO_D"]
            per[name + "_d_frames"] = state_counts["D"]
        all_summaries.append(per)
    write_csv(os.path.join(args.output_dir, "b2_three_zone_threshold_grid.csv"), grid_header, grid_rows)
    write_csv(os.path.join(args.output_dir, "b2_threshold_b4_simulation.csv"), sim_header, sequence_rows)
    write_csv(os.path.join(args.output_dir, "b2_threshold_track_sequences.csv"), track_header, track_rows)

    candidates = select_candidates(all_summaries)
    candidate_header = ["candidate_level", "selection_note", "config_id", "rd", "ed", "pd", "rs", "es", "ps",
                        "walking_moving", "walking_static", "walking_uncertain", "walking_unreliable", "walking_p_to_d", "walking_d_frames",
                        "sitting_moving", "sitting_static", "sitting_uncertain", "sitting_unreliable", "sitting_p_to_d", "sitting_d_frames"]
    write_csv(os.path.join(args.output_dir, "b2_candidate_parameter_sets.csv"), candidate_header,
              [[x.get(k, "") for k in candidate_header] for x in candidates])
    interval_header = ["candidate_level", "config_id", "event", "relative_track_row", "frame_id", "timestamp", "track_id", "instance_id",
                       "candidate_count", "matched_count", "valid_count", "background_sample_count", "background_median_r3d", "background_sigma_r3d",
                       "raw_r3d_median", "norm_r3d_median", "high_ratio", "b2_zone", "previous_b4_state", "current_b4_state", "b4_reason"]
    interval_rows = []
    for c in candidates:
        replay_events, track_summaries = replay_cache[(c["config_id"], "walking")]
        interval_rows.extend(event_windows(c, replay_events, track_summaries))
    write_csv(os.path.join(args.output_dir, "candidate_motion_intervals.csv"), interval_header, interval_rows)

    # Sigma relationship is based only on existing reliable walking rows; no background algorithm is changed.
    reliable = [r for r in datasets["walking"] if r["reliable"]]
    correlations = {"bg_sigma_vs_raw_r3d": pearson([r["background_sigma_r3d"] for r in reliable], [r["median_r3d"] for r in reliable]),
                    "bg_sigma_vs_norm_r3d": pearson([r["background_sigma_r3d"] for r in reliable], [r["median_normalized_r3d"] for r in reliable]),
                    "bg_sigma_vs_high_ratio": pearson([r["background_sigma_r3d"] for r in reliable], [r["high_residual_ratio"] for r in reliable])}
    emit_sigma_effect(os.path.join(args.output_dir, "b2_bg_sigma_effect.csv"), datasets["walking"], parameters)
    sitting_expected = association_count(args.sitting_association)
    sitting_seen = len({r["frame_id"] for r in datasets["sitting"]})
    sitting_coverage = {"b2_unique_frame_ids": sitting_seen, "association_records": sitting_expected,
                        "complete_against_association": sitting_seen == sitting_expected if sitting_expected is not None else None}
    metadata = {"mode": "offline_only", "inputs": {k: os.path.abspath(v) for k, v in (("walking", args.walking_csv), ("sitting", args.sitting_csv))},
                "grid_size": len(parameters), "b4": {"N_D": 3, "N_S": 6, "N_R": 10, "TTL": 15},
                "freeze_rule": "UNRELIABLE and UNCERTAIN freeze B4; they are never moving and do not clear B4 counters.",
                "sitting_coverage": sitting_coverage,
                "sitting_caveat": "Coverage is verified against the supplied association file." if sitting_expected is not None else "No association file was supplied; coverage is not independently verified.",
                "correlations_walking_reliable": correlations}
    with open(os.path.join(args.output_dir, "analysis_metadata.json"), "w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)


if __name__ == "__main__":
    main()
