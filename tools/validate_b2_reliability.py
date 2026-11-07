#!/usr/bin/env python3
"""Isolated regression, short/full RGB-D comparison and fixed evo evaluation."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
EVO = Path('/home/aaa/dynamic_slam_experiments/venv/bin')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--build-dir', type=Path, default=ROOT/'build_b2_reliability')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--full-sequences', action='store_true', help='short tests always precede full runs')
    args = parser.parse_args()
    build=args.build_dir.resolve(); output=args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env=os.environ.copy()
    env.update(QT_QPA_PLATFORM='offscreen', LIBGL_ALWAYS_SOFTWARE='1', MPLBACKEND='Agg',
               MPLCONFIGDIR=str(output/'matplotlib'))
    env['LD_LIBRARY_PATH']=':'.join(str(p) for p in (build/'lib',
        ROOT/'third_party_ort/onnxruntime-linux-x64-1.17.3/lib',ROOT/'Thirdparty/DBoW2/lib',
        ROOT/'Thirdparty/g2o/lib'))+':'+env.get('LD_LIBRARY_PATH','')
    results=[]; comparison=[]
    (output/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary'],cwd=ROOT))

    def run(name, command, expected=0, contains='', config=None, timeout=240):
        folder=output/name;folder.mkdir()
        (folder/'command.txt').write_text(shlex.join(map(str,command))+'\n')
        (folder/'commit.txt').write_text(subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True))
        if config:
            (folder/'effective.yaml').write_bytes(config.read_bytes())
            (folder/'config.sha256').write_text(hashlib.sha256(config.read_bytes()).hexdigest()+'\n')
        start=time.monotonic()
        with (folder/'run.log').open('w') as log:
            try:
                code=subprocess.run(list(map(str,command)),cwd=folder,env=env,stdout=log,
                                    stderr=subprocess.STDOUT,timeout=timeout).returncode
            except subprocess.TimeoutExpired:
                code=124
        text=(folder/'run.log').read_text(errors='replace')
        passed=code==expected and contains in text
        results.append(dict(name=name,exit_code=code,passed=passed,seconds=time.monotonic()-start))
        (folder/'exit_code.txt').write_text(str(code)+'\n')
        (output/'validation_summary.json').write_text(json.dumps(results,indent=2)+'\n')
        print(name,code,'PASS' if passed else 'FAIL',flush=True)
        if not passed: raise RuntimeError(text[-6000:])
        return folder

    run('targeted_test',[build/'bin/b2_reliability_test'],contains='PASS')
    run('multiclass_regression',[build/'bin/multiclass_frontend_test'],contains='PASS')
    run('existing_regressions',['python3',ROOT/'tools/validate_instance_association.py',
        '--build-dir',build,'--output',output/'legacy_regressions','--compatibility'],timeout=300)
    configs=ROOT/'configs/b2_reliability/rgbd_tum3'
    off=(configs/'enhancement_off.yaml').read_text(); on=(configs/'enhancement_on.yaml').read_text()
    assert off.replace('B2.Reliability.Enable: 0','B2.Reliability.Enable: 1')==on
    for group in ['off','on']:
        config=configs/('enhancement_'+group+'.yaml')
        run('config_'+group,[build/'bin/yolo_seg_smoke','--config-only',config],config=config)
    for label,key,value in [('radius','Depth.Radius','0'),('samples','Depth.MinValidSamples','10'),
                            ('grid','Coarse.GridCols','0'),('error','Coarse.MaxMedianReprojectionPixels','.Nan'),
                            ('coverage','Coarse.MinGridCoverage','1.1')]:
        config=output/('invalid_'+label+'.yaml')
        config.write_text(re.sub(r'^B2\.Reliability\.'+re.escape(key)+r':.*$',
            'B2.Reliability.'+key+': '+value,on,flags=re.M))
        run('invalid_'+label,[build/'bin/yolo_seg_smoke','--config-only',config],expected=2,contains='B2.Reliability',config=config)

    for stage,limit in [('short',100)]+([('full',0)] if args.full_sequences else []):
        for seq in ['sitting_static','walking_xyz']:
            dataset=Path('/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_'+seq)
            association=Path('/home/aaa/dynamic_slam_experiments/associations/fr3_'+seq+'_associations.txt')
            if not dataset.is_dir() or not association.is_file():
                results.append(dict(name=seq,skipped=True,reason='missing dataset/association'))
                (output/'validation_summary.json').write_text(json.dumps(results,indent=2)+'\n');continue
            for group in ['off','on']:
                config=configs/('enhancement_'+group+'.yaml')
                folder=run(stage+'_'+seq+'_'+group,[build/'bin/ablation_sequence','rgbd',ROOT/'Vocabulary/ORBvoc.txt',
                    config,dataset,association,limit,0,0],contains='[RUN] frames=',config=config,timeout=900)
                stats=dict(stage=stage,sequence=seq,enhancement=group)
                timing=(folder/'tracking_timing.yaml').read_text()
                for key in ['frames','track_mean_ms','track_median_ms','lost_frames','recently_lost_frames']:
                    stats[key]=float(re.search(r'^'+key+r':\s*(.*)$',timing,re.M).group(1))
                if limit: assert stats['frames']==limit
                instance_rows=list(csv.DictReader(next(folder.glob('b2_evidence_*/b2_instance_evidence.csv')).open()))
                stats['instance_frames']=len(instance_rows)
                stats['reliable_instance_frames']=sum(r['evidence_update_blocked']=='0' for r in instance_rows)
                stats['reliable_instance_ratio']=stats['reliable_instance_frames']/max(1,len(instance_rows))
                matched=sum(int(r['matched_count']) for r in instance_rows)
                stats['valid_3d_over_matched']=sum(int(r['valid_count']) for r in instance_rows)/max(1,matched)
                for key in ['depth_invalid_count','depth_edge_count','match_invalid_count','duplicate_match_count','association_invalid_count','numeric_invalid_count']:
                    stats[key]=sum(int(r[key]) for r in instance_rows)
                first_per_frame={r['frame_id']:r for r in instance_rows}
                stats['quality_sampling_mean_ms']=sum(float(r['quality_sampling_ms']) for r in first_per_frame.values())/max(1,len(first_per_frame))
                stats['b2_processing_mean_ms']=sum(float(r['processing_ms']) for r in first_per_frame.values())/max(1,len(first_per_frame))
                fine=list(csv.DictReader(next(folder.glob('fine_feature_policy_*.csv')).open()))
                assert all(int(r[k])==0 for r in fine for k in ['P_at_optimizer','D_at_optimizer','forbidden_at_optimizer'])
                audit=list(csv.DictReader((folder/'map_admission_final_audit.csv').open()))
                assert audit and all(int(r['forbidden_observations'])==int(r['unaligned_keyframes'])==0 for r in audit)
                stats['camera_poses']=len((folder/'CameraTrajectory.txt').read_text().splitlines())
                if stage=='full' and (dataset/'groundtruth.txt').is_file() and (EVO/'evo_ape').is_file():
                    for metric,relation in [('ate','trans_part'),('rpe_trans','trans_part'),('rpe_rot','angle_deg')]:
                        command=[EVO/('evo_ape' if metric=='ate' else 'evo_rpe'),'tum',dataset/'groundtruth.txt',
                            folder/'CameraTrajectory.txt','--align','--pose_relation',relation,'--t_max_diff','0.02',
                            '--save_results',folder/(metric+'.zip')]
                        if metric!='ate': command+=['--delta','1','--delta_unit','f','--all_pairs']
                        with (folder/(metric+'.txt')).open('w') as log:
                            subprocess.run(list(map(str,command)),cwd=folder,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
                        with zipfile.ZipFile(folder/(metric+'.zip')) as result:
                            value=json.loads(result.read('stats.json'))
                        stats[metric+'_rmse']=value['rmse']
                        if metric=='ate':stats['ate_median']=value['median']
                    match_code='from evo.tools import file_interface as f;from evo.core import sync;import sys;print(sync.associate_trajectories(f.read_tum_trajectory_file(sys.argv[1]),f.read_tum_trajectory_file(sys.argv[2]),max_diff=0.02)[1].num_poses)'
                    stats['matched_poses']=int(subprocess.check_output([str(EVO/'python'),'-c',match_code,
                        str(dataset/'groundtruth.txt'),str(folder/'CameraTrajectory.txt')],env=env,text=True).strip())
                elif stage=='full':stats['metrics_skipped']='missing evo or groundtruth'
                comparison.append(stats)
                columns=sorted(set().union(*(s.keys() for s in comparison)))
                with (output/'comparison.csv').open('w',newline='') as out:
                    writer=csv.DictWriter(out,fieldnames=columns);writer.writeheader();writer.writerows(comparison)
                print(json.dumps(stats),flush=True)


if __name__=='__main__': main()
