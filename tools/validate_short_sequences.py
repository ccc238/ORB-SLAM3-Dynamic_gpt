#!/usr/bin/env python3
"""Real-data process checks only; these are not accuracy/performance benchmarks."""
import datetime
import json
from pathlib import Path
import re
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    out = root/'results/yolo_validation'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=False)
    summary = []
    for mode, camera, sequence in [
        ('rgbd','rgbd_tum3','/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz'),
        ('stereo','stereo_euroc','/home/aaa/Datasets/EuRoc/MH_01_easy')]:
        for group in ['baseline','yolo_only','yolo_dilate']:
            run = out/(camera+'_'+group)
            command = [sys.executable,str(root/'tools/run_ablation.py'),'--mode',mode,
                       '--config',str(root/f'configs/ablation/{camera}/{group}.yaml'),
                       '--sequence',sequence,'--output',str(run),'--max-frames','60','--timeout','180']
            if mode=='rgbd':
                command += ['--associations','/home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt']
            print('Running '+run.name,flush=True)
            p = subprocess.run(command,cwd=root,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            print(p.stdout,flush=True)
            if p.returncode: raise RuntimeError(f'Short sequence failed: {run}; {p.stdout}')
            text = (run/'semantic_timing.yaml').read_text()
            def number(key):
                return float(re.search(r'^'+key+r':\s*(\S+)',text,re.M).group(1))
            views = 120 if mode=='stereo' else 60
            expected_infer = 0 if group=='baseline' else views
            expected_post = views if group=='yolo_dilate' else 0
            if number('inference_calls')!=expected_infer or number('postprocess_calls')!=expected_post:
                raise RuntimeError('Wrong module invocation counts: '+str(run))
            summary.append(dict(run=str(run),frames=60,exit_code=0,
                                inference_calls=number('inference_calls'),postprocess_calls=number('postprocess_calls'),
                                trajectories=(run/'trajectory_line_count.txt').read_text()))
    # Full SLAM startup must work without a model path when all modules are off.
    config = out/'baseline_missing_model.yaml'
    config.write_text((root/'configs/ablation/rgbd_tum3/baseline.yaml').read_text().replace(
        '/home/aaa/models/yolo/yolo11s-seg.onnx','/definitely_missing/yolo.onnx'))
    command = [sys.executable,str(root/'tools/run_ablation.py'),'--mode','rgbd','--config',str(config),
               '--sequence','/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz',
               '--associations','/home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt',
               '--output',str(out/'baseline_missing_model'),'--max-frames','10','--timeout','180']
    p = subprocess.run(command,cwd=root,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    print(p.stdout,flush=True)
    if p.returncode: raise RuntimeError('Disabled full startup without model failed')
    again = subprocess.run(command,cwd=root,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if again.returncode==0 or 'FileExistsError' not in again.stdout:
        raise RuntimeError('Existing run directory not protected')
    summary.append(dict(name='disabled_full_startup_without_model',passed=True))
    summary.append(dict(name='existing_output_refused',passed=True))
    (out/'validation_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('Report: '+str(out),flush=True)


if __name__=='__main__': main()
