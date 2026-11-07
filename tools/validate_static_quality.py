#!/usr/bin/env python3
"""Fresh-output tests and paired RGB-D experiments; never overwrite old results."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import statistics
import subprocess
import tarfile
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build-dir',type=Path,default=ROOT/'build_static_quality')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--data-root',type=Path,default=Path.home()/'Datasets/TUM')
    p.add_argument('--association-root',type=Path,default=Path.home()/'dynamic_slam_experiments/associations')
    p.add_argument('--evo-bin',type=Path,default=Path.home()/'dynamic_slam_experiments/venv/bin')
    p.add_argument('--full',action='store_true')
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--sequences',nargs='+',default=['sitting_static','walking_xyz'])
    p.add_argument('--config-root',type=Path,default=ROOT/'configs/static_quality/rgbd_tum3')
    p.add_argument('--require-b2-reliability',action='store_true')
    a=p.parse_args()
    if a.repeats<3: p.error('use at least three full repeats')
    if len(set(a.sequences))!=len(a.sequences) or any(not re.fullmatch('[a-z0-9_]+',s) for s in a.sequences):
        p.error('sequence names must be unique lowercase identifiers')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    build=a.build_dir.resolve();env=os.environ.copy()
    env.update(QT_QPA_PLATFORM='offscreen',LIBGL_ALWAYS_SOFTWARE='1',MPLBACKEND='Agg',MPLCONFIGDIR=str(out/'matplotlib'))
    env['LD_LIBRARY_PATH']=':'.join(map(str,[build/'lib',ROOT/'third_party_ort/onnxruntime-linux-x64-1.17.3/lib',
        ROOT/'Thirdparty/DBoW2/lib',ROOT/'Thirdparty/g2o/lib']))+':'+env.get('LD_LIBRARY_PATH','')
    reports=[];metrics=[]
    def execute(name,command,expected=0,contains='',timeout=900):
        folder=out/name;folder.mkdir()
        (folder/'build_yolo11/validation').mkdir(parents=True)
        (folder/'command.txt').write_text(shlex.join(map(str,command))+'\n')
        (folder/'commit.txt').write_bytes(subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT))
        t=time.monotonic()
        with (folder/'run.log').open('w') as log:
            try: code=subprocess.run(list(map(str,command)),cwd=folder,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=timeout).returncode
            except subprocess.TimeoutExpired: code=124
        (folder/'exit_code.txt').write_text(str(code)+'\n')
        text=(folder/'run.log').read_text(errors='replace')
        passed=code==expected and contains in text
        reports.append(dict(name=name,exit_code=code,passed=passed,seconds=time.monotonic()-t))
        (out/'checks.json').write_text(json.dumps(reports,indent=2)+'\n')
        print(name,code,'PASS' if passed else 'FAIL',flush=True)
        if not passed: raise RuntimeError(text[-6000:])
        return folder
    (out/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary'],cwd=ROOT))
    # Include untracked new files as well as dirty tracked sources.
    files=sorted({*ROOT.glob('include/**/*.h'),*ROOT.glob('src/**/*.cc'),*ROOT.glob('src/**/*.cpp'),
                  *ROOT.glob('tools/*.cc'),*ROOT.glob('tools/*.py'),
                  ROOT/'CMakeLists.txt',ROOT/'docs/STATIC_QUALITY_FINE_POSE.md',
                  *ROOT.glob('configs/static_quality/**/*.yaml')})
    files=sorted(set(files)|set(a.config_root.resolve().glob('*.yaml')))
    (out/'source_hashes.json').write_text(json.dumps({str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2))
    with tarfile.open(out/'implemented_sources.tar.gz','w:gz') as archive:
        for f in files:archive.add(f,arcname=str(f.relative_to(ROOT)))
    (out/'branch.txt').write_bytes(subprocess.check_output(['git','branch','--show-current'],cwd=ROOT))
    (out/'git_status.txt').write_bytes(subprocess.check_output(['git','status','--short'],cwd=ROOT))
    (out/'runtime_hashes.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest()
        for f in [build/'bin/ablation_sequence',build/'lib/libORB_SLAM3.so',ROOT/'Vocabulary/ORBvoc.txt']},indent=2))
    execute('synthetic',[build/'bin/static_quality_test'],contains='PASS')
    for name in ['fine_feature_policy_test','map_admission_test','b4_evidence_expiry_test',
                 'identity_state_integration_test','coarse_feature_filter_test','b2_reliability_test']:
        execute(name,[build/'bin'/name],contains='PASS')
    execute('legacy_suite',['python3',ROOT/'tools/validate_instance_association.py',
        '--build-dir',build,'--output',out/'legacy_checks','--compatibility'],timeout=600)
    config_root=a.config_root.resolve()
    off=(config_root/'quality_off.yaml').read_text();on=(config_root/'quality_on.yaml').read_text()
    assert off.replace('StaticQuality.Enable: 0','StaticQuality.Enable: 1')==on
    if a.require_b2_reliability:
        assert re.search(r'^B2\.Reliability\.Enable:\s*1\s*$',off,re.M)
        assert re.search(r'^B2\.Reliability\.Enable:\s*1\s*$',on,re.M)
    for group in ['off','on']:
        execute('config_'+group,[build/'bin/yolo_seg_smoke','--config-only',config_root/('quality_'+group+'.yaml')])
    for key,value in [('WeightMin','0'),('WeightMin','1.1'),('DepthSigmaA','0'),('DepthSigmaB','-0.1'),
                      ('WeightMin','"bad"'),('DepthSigmaA','.Nan'),('Enable','2'),('SaveSummaryCsv','"yes"')]:
        config=out/('invalid_'+key+'_'+str(len(reports))+'.yaml')
        config.write_text(re.sub(r'^StaticQuality\.'+key+r':.*$',f'StaticQuality.{key}: {value}',on,flags=re.M))
        execute('invalid_'+str(len(reports)),[build/'bin/yolo_seg_smoke','--config-only',config],expected=2,contains='StaticQuality')
    config=out/'invalid_mode.yaml';config.write_text(on.replace('TwoStage.MapAdmission.Enable: 1','TwoStage.MapAdmission.Enable: 0'))
    execute('invalid_mode',[build/'bin/yolo_seg_smoke','--config-only',config],expected=2,contains='StaticQuality')
    # Verify legacy disabled startup without any usable model path.
    legacy=out/'legacy_no_model.yaml'
    legacy.write_text(re.sub(r'^Yolo.ModelPath:.*$', 'Yolo.ModelPath: "/nonexistent/static_quality_test.onnx"',
        (ROOT/'configs/frontend_modes/rgbd_tum3/mode0_original.yaml').read_text(),flags=re.M))
    execute('disabled_no_model',[build/'bin/yolo_seg_smoke','--config-only',legacy])
    # Actual sensor rejection is exercised with existing camera/vocabulary inputs.
    sitting=a.data_root/'rgbd_dataset_freiburg3_sitting_static'
    assoc=a.association_root/'fr3_sitting_static_associations.txt'
    if sitting.is_dir() and assoc.is_file():
        # Use valid stereo calibration: an RGB-D YAML would abort inside
        # Settings before Tracking can report the unsupported quality mode.
        stereo=out/'unsupported_stereo.yaml'
        stereo.write_text((ROOT/'configs/ablation/stereo_euroc/baseline.yaml').read_text()+
            '\nStaticQuality.Enable: 1\n')
        execute('unsupported_stereo',[build/'bin/ablation_sequence','stereo',ROOT/'Vocabulary/ORBvoc.txt',
            stereo,sitting,assoc,1,0,0],expected=2,contains='StaticQuality supports pure visual RGB-D only')
        execute('legacy_rgbd_startup',[build/'bin/ablation_sequence','rgbd',ROOT/'Vocabulary/ORBvoc.txt',
            legacy,sitting,assoc,3,0,0],contains='[RUN] frames=3')
        reset=execute('quality_system_reset',[build/'bin/identity_system_reset_test',ROOT/'Vocabulary/ORBvoc.txt',
            config_root/'quality_on.yaml',sitting,assoc],contains='PASS',timeout=180)
        reset_rows=list(csv.DictReader(next(reset.glob('static_quality_*.csv')).open()))
        assert len(reset_rows)==9
        for index in [0,3,6]:
            assert reset_rows[index]['quality_applied']=='0' and reset_rows[index]['depth_quality_valid']=='0'
    stages=[('short',100,0)]+([('full',0,n) for n in range(1,a.repeats+1)] if a.full else [])
    for stage,limit,repeat in stages:
        for seq in a.sequences:
            dataset=a.data_root/('rgbd_dataset_freiburg3_'+seq)
            association=a.association_root/('fr3_'+seq+'_associations.txt')
            if not dataset.is_dir() or not association.is_file():
                reports.append(dict(name=seq,skipped=True,reason='missing dataset/association'))
                (out/'checks.json').write_text(json.dumps(reports,indent=2));continue
            expected=sum(bool(l.strip()) and not l.startswith('#') for l in association.read_text().splitlines())
            # Alternate order to reduce systematic thermal/scheduling bias.
            for group in (['off','on'] if repeat%2 else ['on','off']):
                name=f'{stage}_{seq}_{group}_r{repeat}'
                config=config_root/('quality_'+group+'.yaml')
                command=[build/'bin/ablation_sequence','rgbd',ROOT/'Vocabulary/ORBvoc.txt',config,dataset,association,limit,0,0]
                folder=execute(name,command,contains='[RUN] frames=')
                if a.require_b2_reliability:
                    assert '[B2 RELIABILITY CONFIG] Enable=1' in (folder/'run.log').read_text()
                (folder/'effective.yaml').write_bytes(config.read_bytes())
                (folder/'config.sha256').write_text(hashlib.sha256(config.read_bytes()).hexdigest())
                model=re.search(r'^Yolo.ModelPath:\s*"([^"]+)"',config.read_text(),re.M).group(1)
                (folder/'model.sha256').write_text(hashlib.sha256(Path(model).read_bytes()).hexdigest())
                timing=(folder/'tracking_timing.yaml').read_text()
                row=dict(sequence=seq,stage=stage,repeat=repeat,quality=group)
                for key in ['frames','track_mean_ms','track_median_ms','lost_frames','recently_lost_frames']:
                    row[key]=float(re.search(r'^'+key+r':\s*(.*)$',timing,re.M).group(1))
                assert row['frames']==(limit or expected)
                row['poses']=len((folder/'CameraTrajectory.txt').read_text().splitlines())
                fine=list(csv.DictReader(next(folder.glob('fine_feature_policy_*.csv')).open()))
                assert all(int(r[k])==0 for r in fine for k in ['P_at_optimizer','D_at_optimizer','forbidden_at_optimizer'])
                audit=list(csv.DictReader((folder/'map_admission_final_audit.csv').open()))
                assert audit and all(int(r['forbidden_observations'])==int(r['unaligned_keyframes'])==0 for r in audit)
                if group=='on':
                    quality=list(csv.DictReader(next(folder.glob('static_quality_*.csv')).open()))
                    assert len(quality)==row['frames'] and len({r['frame_id'] for r in quality})==len(quality)
                    for q in quality:
                        assert int(q['allowed_matches'])==int(q['depth_quality_valid'])+int(q['neutral_weight_count'])
                    observations=sum(int(q['allowed_matches']) for q in quality)
                    row['quality_valid_ratio']=sum(int(q['depth_quality_valid']) for q in quality)/max(1,observations)
                    row['weight_mean']=sum(float(q['weight_sum']) for q in quality)/max(1,observations)
                    row['quality_compute_mean_ms']=statistics.mean(float(q['quality_compute_ms']) for q in quality)
                    row['quality_applied_frames']=sum(q['quality_applied']=='1' for q in quality)
                else: assert not list(folder.glob('static_quality_*.csv'))
                if stage=='full' and (dataset/'groundtruth.txt').is_file() and (a.evo_bin/'evo_ape').is_file():
                    for metric,relation in [('ate','trans_part'),('rpe_trans','trans_part'),('rpe_rot','angle_deg')]:
                        cmd=[a.evo_bin/('evo_ape' if metric=='ate' else 'evo_rpe'),'tum',dataset/'groundtruth.txt',
                             folder/'CameraTrajectory.txt','--align','--pose_relation',relation,'--t_max_diff','0.02',
                             '--save_results',folder/(metric+'.zip')]
                        if metric!='ate':cmd+=['--delta','1','--delta_unit','f','--all_pairs']
                        (folder/(metric+'_command.txt')).write_text(shlex.join(map(str,cmd)))
                        with (folder/(metric+'.txt')).open('w') as log:
                            subprocess.run(list(map(str,cmd)),cwd=folder,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                        with zipfile.ZipFile(folder/(metric+'.zip')) as z: stats=json.loads(z.read('stats.json'))
                        row[metric+'_rmse']=stats['rmse']
                        if metric=='ate':row['ate_median']=stats['median']
                    code='from evo.tools import file_interface as f;from evo.core import sync;import sys;print(sync.associate_trajectories(f.read_tum_trajectory_file(sys.argv[1]),f.read_tum_trajectory_file(sys.argv[2]),max_diff=0.02)[1].num_poses)'
                    row['matched_poses']=int(subprocess.check_output([str(a.evo_bin/'python'),'-c',code,
                        str(dataset/'groundtruth.txt'),str(folder/'CameraTrajectory.txt')],env=env,text=True).strip())
                elif stage=='full':row['metrics_skipped']='missing groundtruth or evo'
                metrics.append(row)
                fields=sorted(set().union(*(r.keys() for r in metrics)))
                with (out/'comparison.csv').open('w',newline='') as f:
                    w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(metrics)
                print(json.dumps(row),flush=True)
    summary=[]
    for seq in a.sequences:
        for group in ['off','on']:
            rows=[r for r in metrics if r['stage']=='full' and r['sequence']==seq and r['quality']==group]
            if not rows:continue
            s=dict(sequence=seq,quality=group,repeats=len(rows))
            for key in ['ate_rmse','rpe_trans_rmse','rpe_rot_rmse','track_mean_ms','weight_mean']:
                vals=[r[key] for r in rows if key in r]
                if vals:s[key+'_median']=statistics.median(vals);s[key+'_mean']=statistics.mean(vals);s[key+'_std']=statistics.pstdev(vals)
            summary.append(s)
    (out/'aggregate.json').write_text(json.dumps(summary,indent=2)+'\n')

if __name__=='__main__':main()
