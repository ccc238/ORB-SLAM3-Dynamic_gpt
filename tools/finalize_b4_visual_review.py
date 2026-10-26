#!/usr/bin/env python3
"""Tabulate recorded Codex multi-frame visual review, keeping human verification explicit."""
import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

from export_b4_online_review import active, only, read, save


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch",type=Path,required=True)
    parser.add_argument("--audit",type=Path,required=True)
    args=parser.parse_args()
    review=json.loads((args.audit/"codex_visual_labels.json").read_text())
    samples,transitions=review["samples"],review["transitions"]
    for name,rows in (("sample_visual_review.csv",samples),("transition_visual_review.csv",transitions)):
        path=args.audit/name
        if path.exists():raise RuntimeError("Refusing to overwrite "+str(path))
        save(path,list(rows[0]),rows)
    fields=["frame_id","track_id","b4_state","transition","visual_label","track_correct","motion_interval_correct","notes","review_kind","reviewer","human_verified"]
    combined=[]
    for kind,rows in (("transition",transitions),("sample",samples)):
        for r in rows:
            combined.append({k:(kind if k=="review_kind" else r[k]) for k in fields})
    save(args.audit/"visual_review.csv",fields,combined)
    summary={"reviewer":"CODEX_VISUAL_INITIAL_REVIEW","human_verified":False,"review_protocol":review["review_protocol"],
             "transition_count":len(transitions),"sample_count":len(samples)}
    for tag in ("RANDOM_20_D","RANDOM_20_S"):
        group=[r for r in samples if tag in r["selection"].split(";")]
        counts=Counter(r["visual_label"] for r in group)
        summary[tag]={"n":len(group),"counts":dict(counts),"ratios_all_samples":{k:v/len(group) for k,v in counts.items()}}
    p_to_d=[r for r in transitions if r["transition"]=="P_TO_D"]
    d_to_p=[r for r in transitions if r["transition"]=="D_TO_P"]
    summary["P_TO_D"]={"total":len(p_to_d),"visually_moving":sum(r["visual_label"]=="VISIBLE_MOVING" for r in p_to_d),
                       "visually_static":sum(r["visual_label"]=="VISIBLE_STATIC" for r in p_to_d),
                       "other_labels":dict(Counter(r["visual_label"] for r in p_to_d if r["visual_label"] not in ("VISIBLE_MOVING","VISIBLE_STATIC")))}
    summary["D_TO_P"]={"total":len(d_to_p),"visually_static":sum(r["visual_label"]=="VISIBLE_STATIC" for r in d_to_p),
                       "visually_moving":sum(r["visual_label"]=="VISIBLE_MOVING" for r in d_to_p),
                       "ambiguous":sum(r["visual_label"]=="AMBIGUOUS" for r in d_to_p)}
    summary["confirmed_wrong_track_review_rows"]=sum(r["track_correct"]=="NO" or r["visual_label"]=="WRONG_TRACK" for r in combined)
    summary["unknown_track_review_rows"]=sum(r["track_correct"]=="UNKNOWN" for r in combined)
    summary["track_identity_scope"]="Only inspected local windows; zero confirmed errors does not establish zero global ID switches. The previously observed duplicate head detection (track 12 at frame 661) is not classified as a proven cross-person ID swap."
    invariant_rows=[]
    walking=None
    for seq in ("sitting_static","walking_xyz"):
        directory=args.batch/("fr3_"+seq)
        allrows=read(only(directory.glob("b4_state_summary_*.csv")))
        rows=[r for r in allrows if active(r)]
        by_track=defaultdict(list)
        for r in rows:by_track[r["track_id"]].append(r)
        state_fail=counter_fail=unreliable_transitions=freeze_count=0
        for rr in by_track.values():
            previous=None
            for r in sorted(rr,key=lambda x:int(x["frame_id"])):
                if "UNRELIABLE_EVIDENCE_FREEZE" in r["transition_reason"]:
                    freeze_count+=1
                    state_fail+=r["previous_state"]!=r["current_state"]
                    counter_fail+=bool(previous and any(r[k]!=previous[k] for k in ("static_count","dynamic_count","recovery_count")))
                if r["previous_state"]!=r["current_state"] and r["b3_reliable"]!="1":unreliable_transitions+=1
                previous=r
        invariant_rows.append(dict(sequence=seq,freeze_rows=freeze_count,state_freeze_violations=state_fail,counter_freeze_violations=counter_fail,
                                   unreliable_triggered_transitions=unreliable_transitions))
        assert state_fail==counter_fail==unreliable_transitions==0
        if seq=="walking_xyz":walking=by_track
        else:
            b3=read(only(directory.glob("b3_fusion_summary_*.csv")))
            summary["sitting_control"]={"moving_fusion":sum(r["fusion_category"]=="SEM_DYNAMIC_GEOM_MOVING" for r in b3),
                                       "D_instance_frames":sum(r["current_state"]=="D" for r in rows)}
    save(args.audit/"evidence_freeze_audit.csv",list(invariant_rows[0]),invariant_rows)
    summary["freeze_audit"]=invariant_rows
    anchors={(608,9):585,(719,11):685,(729,9):680}
    exit_rows=[]
    for event in d_to_p:
        frame,tid=int(event["frame_id"]),int(event["track_id"])
        rows=sorted(walking[str(tid)],key=lambda r:int(r["frame_id"]))
        upto=[r for r in rows if int(r["frame_id"])<=frame]
        current=upto[-1]
        last_moving=next(r for r in reversed(upto) if r["b3_reliable"]=="1" and r["b3_category"]=="SEM_DYNAMIC_GEOM_MOVING")
        recovery=[r for r in upto if int(r["frame_id"])>int(last_moving["frame_id"]) and r["b3_reliable"]=="1" and r["b3_category"]=="SEM_DYNAMIC_GEOM_STATIC"]
        anchor=anchors.get((frame,tid))
        anchor_row=next((r for r in rows if int(r["frame_id"])==anchor),None)
        exit_rows.append(dict(frame_id=frame,timestamp=current["timestamp"],track_id=tid,visual_label=event["visual_label"],
                             track_correct=event["track_correct"],last_reliable_moving_frame=last_moving["frame_id"],
                             static_recovery_count=len(recovery),static_recovery_frames=";".join(r["frame_id"] for r in recovery),
                             inspected_stationary_anchor_frame=anchor if anchor is not None else "",
                             lag_since_inspected_stationary_anchor_seconds=float(current["timestamp"])-float(anchor_row["timestamp"]) if anchor_row else "",
                             notes="Anchor is an inspected relatively stationary sample, not a ground-truth stop-onset. Same track persists through the exit."))
    save(args.audit/"D_TO_P_visual_exit_review.csv",list(exit_rows[0]),exit_rows)
    summary["overall_visual_acceptance"]="PARTIAL: entry evidence is visually supported, but D-static retention and S-state missed motion are observed. Do not enable feature filtering based on these checks."
    (args.audit/"visual_review_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2))


if __name__=="__main__":main()
