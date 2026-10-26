#!/usr/bin/env python3
"""Controlled coarse-filter fallback probes, always in fresh result directories."""
from datetime import datetime
import csv
import json
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def rows(directory,pattern):
    paths=list(directory.glob(pattern));assert len(paths)==1,(directory,pattern)
    with paths[0].open() as stream:return list(csv.DictReader(stream))

def main():
    parent=ROOT/'results/frontend_stage_validation'/('batch9_forced_filter_fallback_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    parent.mkdir(parents=True,exist_ok=False)
    print('BATCH '+str(parent),flush=True)
    base=(ROOT/'configs/frontend_modes/rgbd_tum3/mode3_coarse_filter_validation.yaml').read_text()
    results=[]
    for name,key,value in [('insufficient_matches','TwoStage.MinCoarseInliers','100000'),
                           ('low_inlier_ratio','TwoStage.MinCoarseInlierRatio','1.0')]:
        directory=parent/name;directory.mkdir()
        config,count=re.subn(r'^'+re.escape(key)+r':.*$',key+': '+value,base,flags=re.M);assert count==1
        snapshot=directory/'effective_config.yaml';snapshot.write_text(config)
        env=os.environ.copy()
        env['LD_LIBRARY_PATH']=':'.join(str(ROOT/x) for x in ('build_yolo11/lib','Thirdparty/DBoW2/lib',
            'Thirdparty/g2o/lib','third_party_ort/onnxruntime-linux-x64-1.17.3/lib'))
        env['QT_QPA_PLATFORM']='offscreen'
        command=[str(ROOT/'build_yolo11/bin/ablation_sequence'),'rgbd',str(ROOT/'Vocabulary/ORBvoc.txt'),
            str(snapshot),'/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_sitting_static',
            '/home/aaa/dynamic_slam_experiments/associations/fr3_sitting_static_associations.txt','30','0','0']
        (directory/'command.sh').write_text('cd '+shlex.quote(str(directory))+'\nexport LD_LIBRARY_PATH='+shlex.quote(env['LD_LIBRARY_PATH'])+'\n'+shlex.join(command)+'\n')
        (directory/'commit.txt').write_bytes(subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT))
        with (directory/'run.log').open('w') as log:
            code=subprocess.run(command,cwd=directory,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
        (directory/'exit_code.txt').write_text(str(code)+'\n');assert code==0
        log=(directory/'run.log').read_text()
        assert '[RUN] frames=30 OK=30 RECENTLY_LOST=0 LOST=0' in log
        filters=rows(directory,'coarse_feature_filter_*.csv')
        failed=[r for r in filters if r['coarse_fallback']=='1'];assert failed
        quality={int(r['frame_id']):r for r in rows(directory,'coarse_pose_quality_*.csv')}
        fallback_frames={int(r['frame_id']) for r in failed}
        for r in failed:
            assert r['coarse_policy_applied']=='0' and int(r['restored_inliers'])>0
            assert quality[int(r['frame_id'])]['reliable']=='0'
        for r in rows(directory,'feature_policy_summary_*.csv'):
            if int(r['frame_id']) in fallback_frames:
                assert r['actual_excluded_count']==r['policy_applied']=='0'
        for r in rows(directory,'b3_fusion_summary_*.csv'):
            if int(r['frame_id']) in fallback_frames:assert r['fusion_reliable']=='0'
        for r in rows(directory,'b4_state_summary_*.csv'):
            if int(r['frame_id']) in fallback_frames and int(r['instance_id'])>=0:
                assert r['previous_state']==r['current_state']
                assert 'UNRELIABLE_EVIDENCE_FREEZE' in r['transition_reason']
        if name=='insufficient_matches':
            assert len(fallback_frames)==29
            assert all(r['coarse_fallback_reason']=='COARSE_FILTER_INSUFFICIENT_MATCHES' for r in failed)
            assert all(r['current_state']==r['previous_state']=='P' and
                all(r[k]=='0' for k in ('static_count','dynamic_count','recovery_count'))
                for r in rows(directory,'b4_state_summary_*.csv'))
        else:
            assert any(r['coarse_fallback_reason']=='COARSE_FILTER_POSE_NOT_RELIABLE' for r in failed)
        result=dict(case=name,passed=True,directory=str(directory),exit_code=code,frames=30,
            fallback_frames=len(fallback_frames),reasons=sorted({r['coarse_fallback_reason'] for r in failed}))
        results.append(result)
        (directory/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
        (parent/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(result),flush=True)
    print('FALLBACK PROBES PASSED '+str(parent),flush=True)

if __name__=='__main__':main()
