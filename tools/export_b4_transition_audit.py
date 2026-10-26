#!/usr/bin/env python3
"""Make transition windows and reproducible visual samples from online CSVs."""
import argparse
import csv
import json
import math
import random
from collections import defaultdict
from functools import lru_cache
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps
from export_b4_online_review import active, key, motion_runs, only, read, save


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--batch", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / "transitions").mkdir()
    (out / "samples").mkdir()
    directory = args.batch / "fr3_walking_xyz"
    paths = {"b2":only(directory.glob("b2_evidence_*/b2_instance_evidence.csv")),
             "b3":only(directory.glob("b3_fusion_summary_*.csv")),
             "b4":only(directory.glob("b4_state_summary_*.csv"))}
    data = {k:read(v) for k,v in paths.items()}
    data["b4"] = [r for r in data["b4"] if active(r)]
    joined = {}
    b2 = {key(r):r for r in data["b2"]}
    b3 = {key(r):r for r in data["b3"]}
    by_track = defaultdict(list)
    for r in data["b4"]:
        frame,instance,track = key(r)
        r = dict(r, b2_state=b2[key(r)]["geometry_state"], b3_category=b3[key(r)]["fusion_category"],
                 raw_r3d=b2[key(r)]["median_r3d"], norm_r3d=b2[key(r)]["median_normalized_r3d"],
                 high_ratio=b2[key(r)]["high_residual_ratio"], bbox=tuple(int(b3[key(r)][k]) for k in ("bbox_x","bbox_y","bbox_width","bbox_height")))
        joined[frame,track] = r
        by_track[track].append(r)
    for rows in by_track.values():
        rows.sort(key=lambda r:int(r["frame_id"]))
    metadata = json.loads((directory / "run_metadata.json").read_text())
    pairs = [line.split() for line in Path(metadata["association_file"]).read_text().splitlines() if line.strip() and not line.startswith("#")]
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 13)
    color = {"S":(65,230,100), "P":(255,220,50), "D":(255,90,90)}

    @lru_cache(maxsize=120)
    def original(frame):
        with Image.open(Path(metadata["dataset"]) / pairs[frame][1]) as im:
            return im.convert("RGB")

    def tile(frame,track,center=None):
        im = original(frame).copy()
        row = joined.get((frame,track))
        caption = row if row else min(by_track[track],key=lambda r:abs(int(r["frame_id"])-frame))
        box = caption["bbox"]
        x,y,w,h = box
        pad = 30
        crop_box = (max(0,x-pad),max(0,y-pad),min(im.width,x+w+pad),min(im.height,y+h+pad))
        context = im.copy()
        drawer = ImageDraw.Draw(context)
        outline = color.get(row["current_state"],(170,170,170)) if row else (170,170,170)
        drawer.rectangle((x,y,x+w,y+h),outline=outline,width=4)
        canvas = Image.new("RGB",(400,405),(24,26,32))
        crop = ImageOps.contain(im.crop(crop_box),(300,255))
        canvas.paste(crop,((300-crop.width)//2,30+(255-crop.height)//2))
        canvas.paste(context.resize((100,75)),(300,30))
        d=ImageDraw.Draw(canvas)
        d.text((5,5),f"f={frame} track={track} {'CENTER' if frame==center else ''}",font=font,fill="white")
        if row:
            lines=["B2="+row["b2_state"],"B3="+row["b3_category"],
                   "B4="+row["previous_state"]+"->"+row["current_state"],
                   "R/E/P={:.4f}/{:.3f}/{:.3f}".format(float(row["raw_r3d"]),float(row["norm_r3d"]),float(row["high_ratio"])),
                   row["transition_reason"],"Ns/Nd/Nr="+"/".join(row[k] for k in ("static_count","dynamic_count","recovery_count"))]
        else:
            lines=["TARGET TRACK NOT DETECTED","B2/B3/B4=N/A","Gray context box is nearest saved detection;","not a current-frame observation."]
        for n,line in enumerate(lines):
            d.text((5,292+n*18),line,font=font,fill=outline if n==2 else "white")
        if center==frame:
            d.rectangle((0,0,399,404),outline=(80,200,255),width=3)
        return canvas

    transitions = sorted([r for r in joined.values() if r["transition_reason"] in ("S_TO_P","P_TO_D","D_TO_P")],key=lambda r:(int(r["frame_id"]),int(r["track_id"])))
    fields=["frame_id","timestamp","track_id","previous_state","current_state","b2_state","b3_category","raw_r3d","norm_r3d","high_ratio","transition_reason"]
    save(out/"all_state_transitions.csv",fields,[{k:r[k] for k in fields} for r in transitions])
    window_rows=[]
    for r in transitions:
        frame,track=int(r["frame_id"]),int(r["track_id"])
        indices=list(range(max(0,frame-15),min(len(pairs),frame+16)))
        sheet=Image.new("RGB",(400*5,405*math.ceil(len(indices)/5)),"black")
        for n,f in enumerate(indices):
            sheet.paste(tile(f,track,frame),((n%5)*400,(n//5)*405))
            row=joined.get((f,track))
            window_rows.append(dict(transition=r["transition_reason"],event_frame_id=frame,track_id=track,frame_id=f,
                timestamp=pairs[f][0],offset=f-frame,track_detected=int(row is not None),b4_state=row["current_state"] if row else "MISSING"))
        filename=f"transition_{r['transition_reason']}_track_{track}_frame_{frame}_contact_sheet.png"
        sheet.save(out/"transitions"/filename)
    save(out/"transition_window_frames.csv",list(window_rows[0]),window_rows)

    selections=defaultdict(set)
    intervals=[]
    for track,rows in by_track.items():
        for start,end,count in motion_runs(rows):
            intervals.append(dict(track_id=track,start_frame_id=start,end_frame_id=end,observed_D_frames=count))
            for event,label in ((start,"D_INTERVAL_START"),((start+end)//2,"D_INTERVAL_MIDDLE"),(end,"D_INTERVAL_END")):
                # Sample a real D instance observation nearest the requested position.
                candidates=[r for r in rows if start<=int(r["frame_id"])<=end and r["current_state"]=="D"]
                chosen=min(candidates,key=lambda r:abs(int(r["frame_id"])-event))
                selections[int(chosen["frame_id"]),track].add(label)
    longest=max(intervals,key=lambda r:r["observed_D_frames"])
    track=longest["track_id"]
    for f in range(longest["start_frame_id"],longest["end_frame_id"]+1,10):
        if (f,track) in joined and joined[f,track]["current_state"]=="D":
            selections[f,track].add("LONGEST_D_EVERY_10_FRAMES")
        else:
            # Keep scheduled missing detections in a separate manifest, not as D samples.
            pass
    rng=random.Random(20261022)
    for state in ("D","S"):
        pool=sorted([k for k,r in joined.items() if r["current_state"]==state])
        for k in rng.sample(pool,20):
            selections[k].add("RANDOM_20_"+state)
    samples=[]
    for n,((frame,track),reasons) in enumerate(sorted(selections.items())):
        row=joined[frame,track]
        samples.append(dict(sample_id=n,frame_id=frame,timestamp=pairs[frame][0],track_id=track,b4_state=row["current_state"],
            transition=row["transition_reason"],selection=";".join(sorted(reasons)),visual_label="",track_correct="",motion_interval_correct="",notes=""))
    for page in range(math.ceil(len(samples)/5)):
        group=samples[page*5:(page+1)*5]
        sheet=Image.new("RGB",(1200,405*len(group)),"black")
        for n,s in enumerate(group):
            frame,track=s["frame_id"],s["track_id"]
            for col,offset in enumerate((-5,0,5)):
                f=min(len(pairs)-1,max(0,frame+offset))
                sheet.paste(tile(f,track,frame),(col*400,n*405))
        sheet.save(out/"samples"/f"sample_page_{page:02d}.png")
    save(out/"visual_sample_selection.csv",list(samples[0]),samples)
    save(out/"D_intervals.csv",list(intervals[0]),intervals)
    transition_review=[dict(frame_id=r["frame_id"],track_id=r["track_id"],b4_state=r["current_state"],transition=r["transition_reason"],
                            visual_label="",track_correct="",motion_interval_correct="",notes="") for r in transitions]
    save(out/"transition_review_template.csv",list(transition_review[0]),transition_review)
    # Verify all PNG files were encoded successfully.
    for path in out.rglob("*.png"):
        with Image.open(path) as im:
            im.verify()
    info=dict(inputs={k:str(v.resolve()) for k,v in paths.items()},random_seed=20261022,
              transitions=len(transitions),transition_window_rows=len(window_rows),sample_count=len(samples),
              longest_D=longest,notes="CSV labels are intentionally blank until multi-frame visual inspection. Missing detections are explicitly marked, not filled with stale state.")
    (out/"audit_metadata.json").write_text(json.dumps(info,indent=2)+"\n")
    print(json.dumps(info,indent=2))


if __name__=="__main__":
    main()
