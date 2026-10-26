#!/usr/bin/env python3
"""Summarize B3 reliability coverage and potential B2 track fragmentation."""
import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


def integer(row, key, fallback=-1):
    try:
        return int(row.get(key, fallback))
    except (TypeError, ValueError):
        return fallback


def boolean(row, key):
    return row.get(key, "0").strip().lower() in ("1", "true")


def iou(a, b):
    left, top = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[0] + a[2], b[0] + b[2]), min(a[1] + a[3], b[1] + b[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    union = a[2] * a[3] + b[2] * b[3] - intersection
    return intersection / union if union > 0 else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="b3_fusion_summary_*.csv")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--short-track-frames", type=int, default=5)
    parser.add_argument("--switch-iou", type=float, default=0.30)
    args = parser.parse_args()
    output = args.output_dir or args.csv.parent / "b3_track_analysis"
    output.mkdir(parents=True, exist_ok=False)

    with args.csv.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = {"frame_id", "track_id", "geometry_reliable", "geometry_reason"}
    if not required.issubset(rows[0] if rows else {}):
        raise SystemExit("CSV is missing required B3 columns: " + ", ".join(sorted(required)))

    tracks = defaultdict(list)
    reason_counts = Counter()
    frames = defaultdict(list)
    has_boxes = all(key in (rows[0] if rows else {}) for key in
                    ("bbox_x", "bbox_y", "bbox_width", "bbox_height"))
    for row in rows:
        track_id = integer(row, "track_id")
        reason_counts[row.get("geometry_reason", "UNKNOWN")] += 1
        if track_id >= 0:
            tracks[track_id].append(row)
        if has_boxes:
            box = tuple(integer(row, key, 0) for key in
                        ("bbox_x", "bbox_y", "bbox_width", "bbox_height"))
            if box[2] > 0 and box[3] > 0:
                frames[integer(row, "frame_id")].append((track_id, box))

    summary_rows = []
    for track_id, entries in sorted(tracks.items()):
        reliable = sum(boolean(row, "geometry_reliable") for row in entries)
        reasons = Counter(row.get("geometry_reason", "UNKNOWN") for row in entries)
        summary_rows.append({
            "track_id": track_id,
            "visible_frames": len(entries),
            "reliable_geometry_frames": reliable,
            "unreliable_geometry_frames": len(entries) - reliable,
            "reliable_coverage": reliable / len(entries) if entries else 0.0,
            "insufficient_matches": reasons["INSUFFICIENT_MATCHES"],
            "insufficient_valid_depth": reasons["INSUFFICIENT_VALID_DEPTH"],
            "short_track": int(len(entries) < args.short_track_frames),
        })

    switches = []
    previous = []
    previous_frame = None
    for frame_id in sorted(frames):
        current = frames[frame_id]
        if previous_frame is not None and frame_id == previous_frame + 1:
            for current_track, current_box in current:
                candidates = [(iou(current_box, old_box), old_track)
                              for old_track, old_box in previous]
                if candidates:
                    score, old_track = max(candidates)
                    if score >= args.switch_iou and old_track >= 0 and current_track >= 0 and old_track != current_track:
                        switches.append({"frame_id": frame_id, "previous_track_id": old_track,
                                         "current_track_id": current_track, "iou": score})
        previous, previous_frame = current, frame_id

    with (output / "b3_track_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary_rows[0]) if summary_rows else ["track_id"])
        writer.writeheader()
        writer.writerows(summary_rows)
    with (output / "b3_id_switches.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["frame_id", "previous_track_id", "current_track_id", "iou"])
        writer.writeheader()
        writer.writerows(switches)
    report = {
        "input_csv": str(args.csv), "person_visible_instance_frames": len(rows),
        "track_count": len(summary_rows), "short_track_count": sum(row["short_track"] for row in summary_rows),
        "suspected_id_switch_count": len(switches), "geometry_reason_counts": dict(reason_counts),
        "track_summary_csv": str(output / "b3_track_summary.csv"),
        "id_switches_csv": str(output / "b3_id_switches.csv"),
        "note": "ID switches are an adjacent-frame bbox-IoU heuristic, not ground-truth identity labels.",
    }
    (output / "b3_track_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
