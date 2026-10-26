#!/usr/bin/env python3
"""Serial stage checks: all 100-frame cases pass before any full-sequence case."""
import argparse
from collections import Counter
import csv
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stage",required=True)
    p.add_argument("--mode-config",action="append",required=True,help="name=path")
    p.add_argument("--require-main-person",action="store_true")
    p.add_argument("--short-reference",type=Path,help="Reuse verified short runs from an interrupted batch; full runs use a new directory")
    p.add_argument("--short-only",action="store_true",help="Stop after all short runs for explicit acceptance audits")
    args=p.parse_args()
    configs={name:Path(path).resolve() for name,path in (item.split("=",1) for item in args.mode_config)}
    parent=ROOT/"results/frontend_stage_validation"/(args.stage+"_"+datetime.now().strftime("%Y%m%d_%H%M%S"))
    parent.mkdir(parents=True,exist_ok=False)
    print("BATCH "+str(parent),flush=True)
    env=os.environ.copy()
    env["LD_LIBRARY_PATH"]=":".join(str(ROOT/x) for x in ("build_yolo11/lib","Thirdparty/DBoW2/lib","Thirdparty/g2o/lib","third_party_ort/onnxruntime-linux-x64-1.17.3/lib"))+":"+env.get("LD_LIBRARY_PATH","")
    env["QT_QPA_PLATFORM"]="offscreen"
    env["LIBGL_ALWAYS_SOFTWARE"]="1"
    summary=[]
    phases=("short100","full")
    if args.short_only:phases=("short100",)
    if args.short_only and args.short_reference:
        raise ValueError("--short-only and --short-reference are mutually exclusive")
    if args.short_reference:
        reference=json.loads((args.short_reference/"summary.json").read_text())
        for mode,config in configs.items():
            for seq in ("sitting_static","walking_xyz"):
                matches=[r for r in reference if r["phase"]=="short100" and r["mode"]==mode and r["sequence"]==seq]
                if len(matches)!=1 or not matches[0]["passed"] or matches[0]["config_sha256"]!=sha(config) or matches[0]["runtime_sha256"]!=sha(ROOT/"build_yolo11/lib/libORB_SLAM3.so"):
                    raise RuntimeError("Short-run reference is missing, failed or differs from current config/runtime")
                summary.append(matches[0])
        (parent/"short_validation_reference.json").write_text(json.dumps(summary,indent=2)+"\n")
        phases=("full",)
    for phase in phases:
        for mode,config in configs.items():
            for seq in ("sitting_static","walking_xyz"):
                directory=parent/(phase+"_mode_"+mode+"_"+seq)
                directory.mkdir()
                snapshot=directory/"effective_config.yaml"
                shutil.copy2(config,snapshot)
                associations=Path("/home/aaa/dynamic_slam_experiments/associations")/("fr3_"+seq+"_associations.txt")
                pairs=[l.split() for l in associations.read_text().splitlines() if l.strip() and not l.startswith("#")]
                expected=min(100,len(pairs)) if phase=="short100" else len(pairs)
                command=[str(ROOT/"build_yolo11/bin/ablation_sequence"),"rgbd",str(ROOT/"Vocabulary/ORBvoc.txt"),
                         str(snapshot),str(Path("/home/aaa/Datasets/TUM")/("rgbd_dataset_freiburg3_"+seq)),str(associations),str(expected if phase=="short100" else 0),"0","0"]
                (directory/"command.sh").write_text("#!/usr/bin/env bash\ncd "+shlex.quote(str(directory))+"\n"+
                    "\n".join("export "+k+"="+shlex.quote(env[k]) for k in ("LD_LIBRARY_PATH","QT_QPA_PLATFORM","LIBGL_ALWAYS_SOFTWARE"))+"\n"+shlex.join(command)+"\n")
                (directory/"commit.txt").write_bytes(subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT))
                print("START "+directory.name,flush=True)
                start=time.monotonic()
                with (directory/"run.log").open("wb") as log:
                    process=subprocess.Popen(command,cwd=directory,env=env,stdout=log,stderr=subprocess.STDOUT)
                    try:code=process.wait(timeout=900)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        try:process.wait(timeout=10)
                        except subprocess.TimeoutExpired:process.kill();process.wait()
                        code=124
                log=(directory/"run.log").read_text()
                final=re.search(r"\[RUN\] frames=(\d+) OK=(\d+) RECENTLY_LOST=(\d+) LOST=(\d+)",log)
                front=[dict(re.findall(r"(\w+)=([^ ]+)",line)) for line in log.splitlines() if line.startswith("[FRONTEND]")]
                dyn=[dict(re.findall(r"(\w+)=([^ ]+)",line)) for line in log.splitlines() if line.startswith("[DYNAMIC FRONTEND]")]
                if (directory/'dynamic_frontend.csv').exists():
                    # Structured snapshots are not vulnerable to interleaved
                    # frontend/backend terminal fragments. Legacy path unchanged.
                    with (directory/'dynamic_frontend.csv').open() as data:
                        structured=[r for r in csv.DictReader(data) if r['row_type']=='FRAME']
                    front=[dict(frame_id=r['frame_id'],mask_rejected=r['b1_mask_rejected'],
                                pose_inliers=r['pose_inliers'],keyframe=r['keyframe_created']) for r in structured]
                    dyn=[dict(frame_id=r['frame_id'],person_main_orb=r['person_main_orb']) for r in structured]
                counts={name:sum(1 for l in (directory/name).read_text().splitlines() if l.strip() and not l.startswith("#"))
                        if (directory/name).exists() else 0 for name in ("CameraTrajectory.txt","KeyFrameTrajectory.txt")}
                mask_rejected=sum(int(r["mask_rejected"]) for r in front)
                passed=code==0 and bool(final) and int(final[1])==expected and len(front)==expected and all(counts.values())
                if mode in ("0","2","3","fine3","map3","kf3"):
                    passed=passed and mask_rejected==0
                if mode=="1":passed=passed and mask_rejected>0
                if args.require_main_person and mode in ("0","1","2","3","fine3","map3","kf3"):
                    passed=passed and len(dyn)==expected
                    if mode in ("2","3","fine3","map3","kf3"):passed=passed and sum(int(r["person_main_orb"]) for r in dyn)>0
                if mode in ("3","fine3","map3","kf3"):
                    passed=passed and int(final[2])==expected and int(final[3])==int(final[4])==0
                    filter_paths=list(directory.glob('coarse_feature_filter_*.csv'))
                    filters=list(csv.DictReader(filter_paths[0].open())) if len(filter_paths)==1 else []
                    passed=passed and bool(filters)
                if mode in ("fine3","map3","kf3"):
                    fine_paths=list(directory.glob('fine_feature_policy_*.csv'))
                    fine_rows=list(csv.DictReader(fine_paths[0].open())) if len(fine_paths)==1 else []
                    final_fine=[r for r in fine_rows if r['phase']=='result']
                    inputs=[r for r in fine_rows if r['phase']=='pre_pose_optimization']
                    passed=passed and len(inputs)==len(final_fine)==expected-1 and all(
                        r['P_at_optimizer']==r['D_at_optimizer']==r['forbidden_at_optimizer']=='0' for r in inputs)
                    passed=passed and all(r['fine_fallback']=='0' and r['optimizer_executed']=='1' for r in final_fine)
                categories={}
                for kind,pattern,column in (("b2","b2_evidence_*/b2_instance_evidence.csv","geometry_state"),
                                            ("b3","b3_fusion_summary_*.csv","fusion_category"),
                                            ("b4","b4_state_summary_*.csv","current_state")):
                    paths=list(directory.glob(pattern))
                    records=list(csv.DictReader(paths[0].open())) if len(paths)==1 else []
                    if kind=="b4":records=[r for r in records if int(r["instance_id"])>=0 and r["b3_reason"] not in ("YOLO_MISSED_TTL_ACTIVE","TRACK_TTL_EXPIRED")]
                    categories[kind]=dict(Counter(r[column] for r in records))
                    if mode in ("0","1"):passed=passed and not paths
                    if mode in ("legacy","2","3","fine3","map3","kf3"):passed=passed and bool(records)
                if mode in ("map3","kf3"):
                    audit_path=directory/"map_admission_final_audit.csv"
                    audit=list(csv.DictReader(audit_path.open())) if audit_path.exists() else []
                    map_paths=list(directory.glob("map_admission_[0-9]*.csv"))
                    map_rows=list(csv.DictReader(map_paths[0].open())) if len(map_paths)==1 else []
                    passed=passed and len(audit)==1 and bool(map_rows) and int(audit[0]['active_keyframes'])>0 and (
                        audit[0]['forbidden_observations']==audit[0]['unaligned_keyframes']=='0')
                    passed=passed and sum(int(r['map_points_created']) for r in map_rows)>0
                if mode=="kf3":
                    from audit_keyframe_order import audit_run
                    keyframe_audit=audit_run(directory,expected)
                    passed=passed and keyframe_audit['passed']
                    (directory/'keyframe_order_audit.json').write_text(json.dumps(keyframe_audit,indent=2)+'\n')
                if mode=="legacy" and phase=="full":
                    if seq=="sitting_static":passed=passed and categories["b3"].get("SEM_DYNAMIC_GEOM_MOVING",0)==0 and categories["b4"].get("D",0)==0
                    else:passed=passed and categories["b3"].get("SEM_DYNAMIC_GEOM_MOVING",0)>0 and categories["b4"].get("D",0)>0
                stats={"phase":phase,"mode":mode,"sequence":seq,"directory":str(directory),"exit_code":code,
                       "expected_frames":expected,"duration_seconds":time.monotonic()-start,"config_sha256":sha(snapshot),
                       "runtime_sha256":sha(ROOT/"build_yolo11/lib/libORB_SLAM3.so"),"trajectory_rows":counts,
                       "mask_rejected_sum":mask_rejected,"person_main_orb_sum":sum(int(r["person_main_orb"]) for r in dyn),
                       "tracking_counts":list(map(int,final.groups())) if final else None,"categories":categories,"passed":passed}
                (directory/"validation.json").write_text(json.dumps(stats,indent=2)+"\n")
                (directory/"exit_code.txt").write_text(str(code)+"\n")
                summary.append(stats)
                (parent/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
                print("END "+directory.name+" passed="+str(passed)+" exit="+str(code),flush=True)
                if not passed:raise RuntimeError("Stage validation failed; STOP: "+str(directory))
    print("STAGE PASSED "+str(parent),flush=True)


if __name__=="__main__":main()
