#!/usr/bin/env python3
"""Export existing online B2/B3/B4 records and deterministic RGB annotations."""
import argparse
import csv
import json
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def read(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def save(path, fields, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def only(paths):
    paths = list(paths)
    if len(paths) != 1:
        raise RuntimeError("Expected one CSV: " + repr(paths))
    return paths[0]


def key(row):
    return tuple(int(row[k]) for k in ("frame_id", "instance_id", "track_id"))


def active(row):
    return int(row["instance_id"]) >= 0 and row["b3_reason"] not in {
        "YOLO_MISSED_TTL_ACTIVE", "TRACK_TTL_EXPIRED"}


def motion_runs(rows):
    intervals, block = [], []
    for row in sorted(rows, key=lambda r: int(r["frame_id"])):
        if row["current_state"] == "D":
            block.append(row)
        elif block:
            intervals.append((int(block[0]["frame_id"]), int(block[-1]["frame_id"]), len(block)))
            block = []
    if block:
        intervals.append((int(block[0]["frame_id"]), int(block[-1]["frame_id"]), len(block)))
    return intervals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--offline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / "walking_frames").mkdir()
    data, count_tables, track_table, inputs = {}, {d: [] for d in ("b2", "b3", "b4", "transition")}, [], {}
    for sequence in ("sitting_static", "walking_xyz"):
        directory = args.batch / ("fr3_" + sequence)
        paths = {"b2": only(directory.glob("b2_evidence_*/b2_instance_evidence.csv")),
                 "b3": only(directory.glob("b3_fusion_summary_*.csv")),
                 "b4": only(directory.glob("b4_state_summary_*.csv"))}
        inputs[sequence] = {k: str(v.resolve()) for k, v in paths.items()}
        values = {k: read(p) for k, p in paths.items()}
        values["b4_all"] = values["b4"]
        values["b4"] = [r for r in values["b4_all"] if active(r)]
        indices = {k: {key(r): r for r in values[k]} for k in ("b2", "b3", "b4")}
        assert all(len(indices[k]) == len(values[k]) for k in indices), "Duplicate instance key"
        assert set(indices["b2"]) == set(indices["b3"]) == set(indices["b4"]), "B2/B3/B4 key mismatch"
        values["indices"] = indices
        categories = {"b2": Counter(r["geometry_state"] for r in values["b2"]),
                      "b3": Counter(r["fusion_category"] for r in values["b3"]),
                      "b4": Counter(r["current_state"] for r in values["b4"]),
                      "transition": Counter(t for r in values["b4"] for t in r["transition_reason"].split(";"))}
        defaults = {"b2": ["GEOM_NOT_EVALUATED", "GEOM_STATIC_CANDIDATE", "GEOM_MOVING_CANDIDATE", "GEOM_UNCERTAIN", "GEOM_UNRELIABLE"],
                    "b3": ["SEM_DYNAMIC_GEOM_STATIC", "SEM_DYNAMIC_GEOM_MOVING", "FUSION_UNRELIABLE"],
                    "b4": ["S", "P", "D"], "transition": ["P_TO_S", "P_TO_D", "S_TO_P", "D_TO_P", "S_STAY", "D_STAY", "UNRELIABLE_EVIDENCE_FREEZE"]}
        for domain, counts in categories.items():
            for category in sorted(set(counts) | set(defaults[domain])):
                count_tables[domain].append(dict(sequence=sequence, category=category, count=counts[category],
                    ratio=counts[category] / len(values["b4"]), denominator_instance_frames=len(values["b4"])))
        by_track = defaultdict(list)
        for r in values["b4"]:
            by_track[int(r["track_id"])].append(r)
        for tid, rows in sorted(by_track.items()):
            rows.sort(key=lambda r: int(r["frame_id"]))
            counts = Counter(r["current_state"] for r in rows)
            evidences = Counter(r["b3_category"] for r in rows)
            strict = accumulated = strict_max = accumulated_max = total_moving = 0
            previous_frame = None
            for r in rows:
                frame = int(r["frame_id"])
                moving = r["b3_reliable"] == "1" and r["b3_category"] == "SEM_DYNAMIC_GEOM_MOVING"
                stationary = r["b3_reliable"] == "1" and r["b3_category"] == "SEM_DYNAMIC_GEOM_STATIC"
                if moving:
                    strict = strict + 1 if previous_frame is not None and frame == previous_frame + 1 else 1
                    accumulated += 1
                    total_moving += 1
                else:
                    strict = 0
                    if stationary:
                        accumulated = 0
                strict_max = max(strict_max, strict)
                accumulated_max = max(accumulated_max, accumulated)
                previous_frame = frame
            intervals = motion_runs(rows)
            first_d = next((r for r in rows if "P_TO_D" in r["transition_reason"].split(";")), None)
            summary = dict(sequence=sequence, track_id=tid, first_frame_id=rows[0]["frame_id"], last_frame_id=rows[-1]["frame_id"],
                visible_instance_frames=len(rows), frame_span=int(rows[-1]["frame_id"]) - int(rows[0]["frame_id"]) + 1,
                static_evidence=evidences["SEM_DYNAMIC_GEOM_STATIC"], moving_evidence=total_moving,
                unreliable_evidence=len(rows)-total_moving-evidences["SEM_DYNAMIC_GEOM_STATIC"],
                max_consecutive_moving=strict_max, max_moving_with_unreliable_frozen=accumulated_max,
                S_instance_frames=counts["S"], P_instance_frames=counts["P"], D_instance_frames=counts["D"],
                final_state=rows[-1]["current_state"], first_enter_D_frame_id=first_d["frame_id"] if first_d else "",
                D_intervals=";".join(f"{a}-{b}({n})" for a,b,n in intervals),
                longest_D_observed_instance_frames=max((n for _,_,n in intervals), default=0))
            track_table.append(summary)
        values["categories"] = categories
        data[sequence] = values
    names = {"b2": "online_b2_category_counts.csv", "b3": "online_b3_category_counts.csv", "b4": "online_b4_state_counts.csv",
             "transition": "online_b4_transition_counts.csv"}
    for domain, filename in names.items():
        save(out / filename, ["sequence", "category", "count", "ratio", "denominator_instance_frames"], count_tables[domain])
    save(out / "online_b4_track_summary.csv", list(track_table[0]), track_table)

    config_id = "RD0.020_ED0.40_PD0.10_RS0.018_ES0.10_PS0.06"
    offline_b2 = read(args.offline / "b2_three_zone_threshold_grid.csv")
    offline_b4 = read(args.offline / "b2_threshold_b4_simulation.csv")
    comparison = []
    for short, sequence in (("walking", "walking_xyz"), ("sitting", "sitting_static")):
        z = next(r for r in offline_b2 if r["config_id"] == config_id and r["sequence"] == short)
        s = next(r for r in offline_b4 if r["config_id"] == config_id and r["sequence"] == short)
        domains = data[sequence]["categories"]
        for metric, before, after, note in [
            ("B2_MOVING", int(z["moving_count"]), domains["b2"]["GEOM_MOVING_CANDIDATE"], ""),
            ("B2_STATIC", int(z["static_count"]), domains["b2"]["GEOM_STATIC_CANDIDATE"], ""),
            ("B2_UNCERTAIN", int(z["uncertain_count"]), domains["b2"]["GEOM_UNCERTAIN"], ""),
            ("B2_UNRELIABLE_INCLUDING_NOT_EVALUATED", int(z["unreliable_count"]), domains["b2"]["GEOM_UNRELIABLE"]+domains["b2"]["GEOM_NOT_EVALUATED"], "Offline script folds NOT_EVALUATED into UNRELIABLE"),
            ("SEM_DYNAMIC_GEOM_MOVING", int(z["moving_count"]), domains["b3"]["SEM_DYNAMIC_GEOM_MOVING"], ""),
            ("SEM_DYNAMIC_GEOM_STATIC", int(z["static_count"]), domains["b3"]["SEM_DYNAMIC_GEOM_STATIC"], ""),
            ("FUSION_UNRELIABLE", int(z["uncertain_count"])+int(z["unreliable_count"]), domains["b3"]["FUSION_UNRELIABLE"], "Includes uncertain B2 evidence"),
        ] + [(f"B4_{state}_instance_frames", int(s[f"{state}_instance_frames"]), domains["b4"][state], "Current detected instances; excludes TTL rows") for state in ("S", "P", "D")] + [
            (t, int(s[t]), domains["transition"][t], "") for t in ("P_TO_S", "S_TO_P", "P_TO_D", "D_TO_P")]:
            comparison.append(dict(sequence=sequence, metric=metric, offline_count=before, online_count=after, delta=after-before, note=note))
    save(out / "offline_vs_online_comparison.csv", list(comparison[0]), comparison)

    metadata = json.loads((args.batch / "fr3_walking_xyz/run_metadata.json").read_text())
    pairs = [line.split() for line in Path(metadata["association_file"]).read_text().splitlines() if line.strip() and not line.startswith("#")]
    walking = data["walking_xyz"]
    by_frame = defaultdict(list)
    for r in walking["b3"]:
        by_frame[int(r["frame_id"])].append(r)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18)
    title_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 23)
    colors = {"S": (50,220,100), "P": (255,210,40), "D": (255,70,70)}
    manifest, annotation_rows = [], []
    requested = sorted(set(list(range(407,418)) + list(range(628,751)) + [445,526,607]))
    for frame in requested:
        association = pairs[frame]
        rgb = Path(metadata["dataset"]) / association[1]
        rows = sorted(by_frame[frame], key=lambda r: int(r["track_id"]))
        with Image.open(rgb) as source:
            source = source.convert("RGB")
            canvas = Image.new("RGB", (1500, max(720,100+len(rows)*265)), (22,24,30))
            canvas.paste(source, (0,72))
        draw = ImageDraw.Draw(canvas)
        draw.text((12,10), f"walking_xyz  frame={frame}  timestamp={association[0]}   EVIDENCE ONLY", font=title_font, fill="white")
        draw.text((12,558), "Boxes: recorded YOLO detections; colors: B4 S=green P=yellow D=red", font=font, fill="white")
        draw.text((12,590), "No segmentation-mask replay or motion ground-truth annotation.", font=font, fill=(185,185,195))
        if not rows:
            draw.text((670,90), "NO CURRENT PERSON DETECTION", font=title_font, fill="white")
        for idx, b3 in enumerate(rows):
            k = key(b3)
            b2, b4 = walking["indices"]["b2"][k], walking["indices"]["b4"][k]
            assert abs(float(b3["timestamp"])-float(association[0])) < 1e-5, "Image/CSV timestamp mismatch"
            box = [int(b3[n]) for n in ("bbox_x", "bbox_y", "bbox_width", "bbox_height")]
            x,y,w,h = box
            color = colors[b4["current_state"]]
            draw.rectangle((x,y+72,x+w,y+h+72), outline=color, width=3)
            draw.text((max(0,x),max(72,y+72-23)), f"ID{b3['track_id']} {b4['previous_state']}->{b4['current_state']}", font=font, fill=color, stroke_width=1, stroke_fill="black")
            lines = [f"track_id={b3['track_id']} instance_id={b3['instance_id']}",
                "B2="+b2["geometry_state"], "B3="+b3["fusion_category"],
                f"B4 previous={b4['previous_state']}  current={b4['current_state']}", "reason="+b4["transition_reason"],
                f"Ns/Nd/Nr={b4['static_count']}/{b4['dynamic_count']}/{b4['recovery_count']}",
                f"R={float(b2['median_r3d']):.4f} E={float(b2['median_normalized_r3d']):.4f} P={float(b2['high_residual_ratio']):.4f}",
                f"candidates/matched/valid={b2['candidate_count']}/{b2['matched_count']}/{b2['valid_count']}",
                "B2 reason="+b2["decision_reason"]]
            for n,line in enumerate(lines):
                draw.text((670,80+idx*265+n*26), line, font=font, fill=color if n in (0,3,4) else "white")
            annotation_rows.append(dict(frame_id=frame, timestamp=association[0], track_id=b3["track_id"], instance_id=b3["instance_id"],
                b2_geometry_state=b2["geometry_state"], b3_fusion_category=b3["fusion_category"],
                b4_previous_state=b4["previous_state"], b4_current_state=b4["current_state"], transition_reason=b4["transition_reason"],
                raw_r3d=b2["median_r3d"], norm_r3d=b2["median_normalized_r3d"], high_ratio=b2["high_residual_ratio"]))
        output = out / "walking_frames" / f"frame_{frame:06d}.png"
        canvas.save(output)
        manifest.append(dict(frame_id=frame, timestamp=association[0], rgb_path=str(rgb), visualization_path=str(output),
                             current_person_count=len(rows), width=canvas.width, height=canvas.height))
    save(out / "visualization_manifest.csv", list(manifest[0]), manifest)
    save(out / "walking_frame_annotations.csv", list(annotation_rows[0]), annotation_rows)
    sheet = Image.new("RGB", (1500,720), "black")
    for index, frame in enumerate((407,412,417,633,680,728,729,745,750)):
        with Image.open(out / "walking_frames" / f"frame_{frame:06d}.png") as im:
            sheet.paste(im.resize((500,240)), ((index%3)*500, (index//3)*240))
    sheet.save(out / "walking_contact_sheet.png")
    for m in manifest:
        with Image.open(m["visualization_path"]) as im:
            im.verify()
    summary = {"inputs": inputs, "offline_config_id": config_id, "requested_frames": requested, "png_count":len(manifest),
               "indexing":"zero-based frame_id, RGB-depth association order", "annotation_method":"Recorded YOLO bounding boxes and CSV states; no model rerun",
               "B4_counts":"Current detected instances only; TTL-only rows excluded", "created_at":datetime.now().astimezone().isoformat()}
    (out / "export_metadata.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(str(out))
    print("Verified PNG frames:",len(manifest))
    for t in track_table:
        if t["sequence"] == "walking_xyz" and t["D_instance_frames"]:
            print("track",t["track_id"],"D intervals",t["D_intervals"])


if __name__ == "__main__":
    main()
