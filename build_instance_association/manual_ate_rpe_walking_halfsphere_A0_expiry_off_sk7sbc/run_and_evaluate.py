#!/usr/bin/env python3
"""One fresh complete A0 run, then SE(3) ATE and adjacent-pose RPE."""
import csv
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time
import zipfile
from evo.core import sync
from evo.tools import file_interface

ROOT=Path('/home/aaa/ORB-SLAM3_original0')
OUT=Path(__file__).resolve().parent
BUILD=ROOT/'build_instance_association'
DATA=Path('/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_halfsphere')
ASSOC=Path('/home/aaa/dynamic_slam_experiments/associations/fr3_walking_halfsphere_associations.txt')
CONFIG=OUT/'experiment_config.yaml'
MODEL=Path('/home/aaa/models/yolo/yolo11s-seg.onnx')
EVO=Path('/home/aaa/dynamic_slam_experiments/venv/bin')

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
    return h.hexdigest()

def main():
    for name in ['run.log','CameraTrajectory.txt','KeyFrameTrajectory.txt','ate.txt','ate.zip','effective.yaml']:
        if (OUT/name).exists():raise RuntimeError('Refuse to overwrite '+str(OUT/name))
    for path in [DATA/'groundtruth.txt',ASSOC,CONFIG,MODEL,BUILD/'bin/ablation_sequence',ROOT/'Vocabulary/ORBvoc.txt',EVO/'evo_ape',EVO/'evo_rpe']:
        if not path.is_file():raise RuntimeError('Missing '+str(path))
    (OUT/'effective.yaml').write_bytes(CONFIG.read_bytes())
    for name,cmd in [('commit.txt',['git','rev-parse','HEAD']),('git_status.txt',['git','status','--short']),
                     ('worktree.patch',['git','diff','--binary'])]:
        (OUT/name).write_bytes(subprocess.check_output(cmd,cwd=ROOT))
    expected=sum(bool(l.strip()) and not l.lstrip().startswith('#') for l in ASSOC.read_text().splitlines())
    metadata=dict(dataset=str(DATA),associations=str(ASSOC),association_rows=expected,
        source_config=str(CONFIG),config_sha256=digest(CONFIG),model=str(MODEL),model_sha256=digest(MODEL),
        executable=str(BUILD/'bin/ablation_sequence'),executable_sha256=digest(BUILD/'bin/ablation_sequence'),
        shared_library_sha256=digest(BUILD/'lib/libORB_SLAM3.so'),evo_version=version('evo'),
        alignment='SE(3), no scale correction',t_max_diff=.02,sync_method='nearest_time',
        rpe_delta=1,rpe_delta_unit='f',all_pairs=True,viewer=False,
        association_mode=0,b4_evidence_expiry=False,max_gap_seconds=.20,max_static_age_seconds=.30)
    (OUT/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    (OUT/'build_yolo11/validation').mkdir(parents=True)
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=':'.join(map(str,[BUILD/'lib',ROOT/'Thirdparty/DBoW2/lib',ROOT/'Thirdparty/g2o/lib',
        ROOT/'third_party_ort/onnxruntime-linux-x64-1.17.3/lib']))+':'+env.get('LD_LIBRARY_PATH','')
    env.update(QT_QPA_PLATFORM='offscreen',LIBGL_ALWAYS_SOFTWARE='1',MPLBACKEND='Agg')
    cmd=[BUILD/'bin/ablation_sequence','rgbd',ROOT/'Vocabulary/ORBvoc.txt',OUT/'effective.yaml',DATA,ASSOC,'0','0','1']
    (OUT/'run_command.txt').write_text(shlex.join(map(str,cmd))+'\n')
    start=time.monotonic();began=datetime.now(timezone.utc).isoformat()
    print('START full walking_halfsphere A0, evidence expiry OFF, viewer disabled:',OUT,flush=True)
    with (OUT/'run.log').open('w') as log:
        try:
            code=subprocess.run(list(map(str,cmd)),cwd=OUT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900).returncode
        except subprocess.TimeoutExpired:code=124
    (OUT/'exit_code.txt').write_text(str(code)+'\n')
    (OUT/'duration.json').write_text(json.dumps(dict(start_utc=began,end_utc=datetime.now(timezone.utc).isoformat(),
        elapsed_seconds=time.monotonic()-start),indent=2)+'\n')
    if code:raise RuntimeError('SLAM failed with exit '+str(code)+'; see run.log')
    camera=OUT/'CameraTrajectory.txt';keyframe=OUT/'KeyFrameTrajectory.txt'
    for path in [camera,keyframe]:
        if not path.is_file() or not path.stat().st_size:raise RuntimeError('Empty/missing trajectory '+str(path))
    counts={p.name:sum(bool(l.strip()) and not l.lstrip().startswith('#') for l in p.read_text().splitlines()) for p in [camera,keyframe]}
    (OUT/'trajectory_line_count.txt').write_text('\n'.join(f'{k}: {v}' for k,v in counts.items())+'\n')
    text=(OUT/'run.log').read_text(errors='replace')
    state=re.search(r'\[RUN\] frames=(\d+) OK=(\d+) RECENTLY_LOST=(\d+) LOST=(\d+)',text)
    if not state or int(state[1])!=expected:raise RuntimeError('Sequence not fully processed')
    print('SLAM finished:',state[0],counts,flush=True)
    result=dict(sequence='fr3_walking_halfsphere',input_frames=expected,camera_poses=counts[camera.name],
        keyframe_poses=counts[keyframe.name],exit_code=code,OK=int(state[2]),RECENTLY_LOST=int(state[3]),LOST=int(state[4]))
    gt=file_interface.read_tum_trajectory_file(str(DATA/'groundtruth.txt'))
    est=file_interface.read_tum_trajectory_file(str(camera))
    _,associated=sync.associate_trajectories(gt,est,max_diff=.02)
    result['matched_poses']=associated.num_poses
    common=['tum',DATA/'groundtruth.txt',camera,'--align','--t_max_diff','0.02','--t_offset','0',
            '--sync_method','nearest_time','--verbose']
    commands=[]
    for name,tool,relation,extra in [('ate','evo_ape','trans_part',[]),
        ('rpe_trans','evo_rpe','trans_part',['--delta','1','--delta_unit','f','--all_pairs']),
        ('rpe_rot','evo_rpe','angle_deg',['--delta','1','--delta_unit','f','--all_pairs'])]:
        command=[EVO/tool]+common+['--pose_relation',relation]+extra+['--save_results',OUT/(name+'.zip')]
        commands.append(shlex.join(map(str,command)))
        (OUT/'evaluation_commands.txt').write_text('\n'.join(commands)+'\n')
        with (OUT/(name+'.txt')).open('w') as log:
            completed=subprocess.run(list(map(str,command)),cwd=OUT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120)
        if completed.returncode:raise RuntimeError(name+' failed; see '+name+'.txt')
        with zipfile.ZipFile(OUT/(name+'.zip')) as z:
            stats=json.loads(z.read('stats.json'));info=json.loads(z.read('info.json'))
        result[name]=dict(stats=stats,info=info)
    (OUT/'metrics_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    compact={k:v for k,v in result.items() if not isinstance(v,dict)}
    compact.update(ate_rmse_m=result['ate']['stats']['rmse'],ate_median_m=result['ate']['stats']['median'],
        rpe_trans_rmse_m=result['rpe_trans']['stats']['rmse'],rpe_rot_rmse_deg=result['rpe_rot']['stats']['rmse'])
    with (OUT/'metrics_summary.csv').open('w') as f:
        writer=csv.DictWriter(f,list(compact));writer.writeheader();writer.writerow(compact)
    print(json.dumps(compact,indent=2),flush=True)

if __name__=='__main__':main()




