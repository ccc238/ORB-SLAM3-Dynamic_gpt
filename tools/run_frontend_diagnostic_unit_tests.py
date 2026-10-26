#!/usr/bin/env python3
"""Run old safety tests and the new output-only tests in fresh directories."""
import csv
import json
import os
from pathlib import Path
import subprocess
import argparse
import re

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--name',required=True,help='Fresh unit result directory name')
args=p.parse_args()
assert Path(args.name).name==args.name and args.name not in ('.','..')
OUT=ROOT/'results/frontend_stage_validation/batch13_diagnostics_build_01'/args.name
OUT.mkdir(exist_ok=False)
env=os.environ.copy()
env['LD_LIBRARY_PATH']=':'.join(str(ROOT/p) for p in ['build_yolo11/lib','Thirdparty/DBoW2/lib','Thirdparty/g2o/lib','third_party_ort/onnxruntime-linux-x64-1.17.3/lib'])+':'+env.get('LD_LIBRARY_PATH','')
env['QT_QPA_PLATFORM']='offscreen'
tests=['frontend_diagnostics_test','keyframe_decision_guard_test','map_admission_test',
       'fine_feature_policy_test','coarse_feature_filter_test','coarse_pose_quality_test',
       'feature_policy_test','feature_semantic_test','frontend_mask_test','b4_state_machine_test',
       'instance_track_lifecycle_test','b2_candidate_test']
summary=[]
for test in tests:
    folder=OUT/test;folder.mkdir();(folder/'build_yolo11/validation').mkdir(parents=True)
    with (folder/'test.log').open('wb') as log:
        result=subprocess.run([str(ROOT/'build_yolo11/bin'/test)],cwd=folder,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120)
    summary.append({'test':test,'exit_code':result.returncode})
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(test,result.returncode,flush=True)
    assert result.returncode==0,(test,(folder/'test.log').read_text())
folder=OUT/'frontend_diagnostics_test'
with (folder/'dynamic_frontend.csv').open() as file:rows=list(csv.DictReader(file))
assert len(rows)==2
assert rows[0]['map_points_created']=='7' and rows[0]['map_reject_P']=='3' and rows[0]['map_reject_D']=='3'
assert rows[0]['map_candidates_total']=='17' and rows[0]['map_counts_pending']=='0'
assert rows[1]['b3_reason']=='quoted,"reason"' and not rows[1]['map_points_created']
base=(ROOT/'configs/frontend_modes/rgbd_tum3/mode0_original.yaml').read_text()
base=re.sub(r'^Yolo.ModelPath:.*$', 'Yolo.ModelPath: "/nonexistent/diagnostic_model.onnx"', base,flags=re.M)
for key in ['SaveCsv','ShowOverlay','SaveOverlay']:
    for value in ['0','1','0.5']:
        path=OUT/f'config_{key}_{value}.yaml'
        path.write_text(base+f'\nDynamicFrontend.Debug.{key}: {value}\n')
        result=subprocess.run([str(ROOT/'build_yolo11/bin/yolo_seg_smoke'),'--config-only',str(path)],cwd=OUT,env=env,capture_output=True,text=True)
        (OUT/f'config_{key}_{value}.log').write_text(result.stdout+result.stderr)
        assert result.returncode==(2 if value=='0.5' else 0),(key,value,result.stderr)
        if value=='0.5':assert f'DynamicFrontend.Debug.{key}' in result.stderr
print('PASS 12 unit tests + source-frame aggregation, CSV round-trip and 9 diagnostic config checks',flush=True)
