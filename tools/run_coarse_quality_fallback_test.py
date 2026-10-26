#!/usr/bin/env python3
"""Force quality fallback only, without corrupting images or modifying SLAM."""
from datetime import datetime
import csv
import json
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def main():
    directory=ROOT/'results/frontend_stage_validation'/('batch7_forced_fallback_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    directory.mkdir(parents=True,exist_ok=False)
    config=(ROOT/'configs/frontend_modes/rgbd_tum3/mode2_coarse_quality_evidence.yaml').read_text()
    config,count=re.subn(r'^TwoStage.MinCoarseInliers:.*$','TwoStage.MinCoarseInliers: 100000',config,flags=re.M)
    assert count==1
    snapshot=directory/'effective_config.yaml'
    snapshot.write_text(config)
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=':'.join(str(ROOT/x) for x in ('build_yolo11/lib','Thirdparty/DBoW2/lib','Thirdparty/g2o/lib','third_party_ort/onnxruntime-linux-x64-1.17.3/lib'))
    env['QT_QPA_PLATFORM']='offscreen'
    command=[str(ROOT/'build_yolo11/bin/ablation_sequence'),'rgbd',str(ROOT/'Vocabulary/ORBvoc.txt'),
             str(snapshot),'/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_sitting_static',
             '/home/aaa/dynamic_slam_experiments/associations/fr3_sitting_static_associations.txt','20','0','0']
    (directory/'command.sh').write_text('cd '+shlex.quote(str(directory))+'\nexport LD_LIBRARY_PATH='+shlex.quote(env['LD_LIBRARY_PATH'])+'\n'+shlex.join(command)+'\n')
    (directory/'commit.txt').write_bytes(subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT))
    with (directory/'run.log').open('w') as stream:
        code=subprocess.run(command,cwd=directory,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=180).returncode
    (directory/'exit_code.txt').write_text(str(code)+'\n')
    assert code==0
    log=(directory/'run.log').read_text()
    assert '[RUN] frames=20 OK=20 RECENTLY_LOST=0 LOST=0' in log
    def rows(pattern):
        paths=list(directory.glob(pattern));assert len(paths)==1
        with paths[0].open() as stream:return list(csv.DictReader(stream))
    quality=rows('coarse_pose_quality_*.csv')
    assert len(quality)==20 and all(r['fallback']=='1' and r['reliable']=='0' for r in quality)
    assert quality[0]['fallback_reason']=='NOT_INITIALIZED'
    assert all(r['fallback_reason']=='INSUFFICIENT_COARSE_FEATURES' for r in quality[1:])
    assert all(r['pose_valid']=='0' for r in rows('b2_evidence_*/b2_instance_evidence.csv'))
    assert all(r['fusion_reliable']=='0' for r in rows('b3_fusion_summary_*.csv'))
    b4=rows('b4_state_summary_*.csv')
    assert b4 and all(r['current_state']==r['previous_state']=='P' and
        all(r[k]=='0' for k in ('static_count','dynamic_count','recovery_count')) and
        r['applied_policy']=='NONE_EVIDENCE_ONLY' for r in b4)
    policy=rows('feature_policy_summary_*.csv')
    assert len(policy)==20 and all(r['actual_excluded_count']==r['policy_applied']=='0' for r in policy)
    fine=[line for line in log.splitlines() if 'event=fine_complete' in line]
    assert len(fine)==19 and all('attempted=1 success=1' in line for line in fine)
    counts={name:len((directory/name).read_text().splitlines()) for name in ('CameraTrajectory.txt','KeyFrameTrajectory.txt')}
    assert counts['CameraTrajectory.txt']==20
    result=dict(passed=True,directory=str(directory),exit_code=code,frames=20,fallback_frames=20,
                original_local_map_tracking_completed=19,b4_effective_counters_all_zero=True,trajectory_rows=counts)
    (directory/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
