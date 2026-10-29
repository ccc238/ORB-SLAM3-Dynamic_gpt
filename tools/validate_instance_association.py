#!/usr/bin/env python3
"""Fresh-output association regressions and optional real RGB-D sequences."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import os
import re
from pathlib import Path
import shlex
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT/'configs/instance_association/rgbd_tum3'
NAMES = ['A0_legacy', 'A1_global', 'A2_predict_ambiguity', 'A3_controlled']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build-dir', type=Path, default=ROOT/'build_instance_association')
    p.add_argument('--output', type=Path, required=True, help='new directory only')
    p.add_argument('--short-sequences', action='store_true', help='100 frames, all four ablations, both sequences')
    p.add_argument('--full-sequences', action='store_true', help='complete A3 sitting/walking after short tests')
    p.add_argument('--compatibility', action='store_true', help='three-frame ORIGINAL/B1/old evidence-mode startup runs')
    a = p.parse_args()
    out = a.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    build = a.build_dir.resolve()
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = ':'.join(map(str, [build/'lib', ROOT/'Thirdparty/DBoW2/lib',
        ROOT/'Thirdparty/g2o/lib', ROOT/'third_party_ort/onnxruntime-linux-x64-1.17.3/lib']))+':'+env.get('LD_LIBRARY_PATH', '')
    env.update(QT_QPA_PLATFORM='offscreen', LIBGL_ALWAYS_SOFTWARE='1')
    summary = []
    def run(name, command, expected=0, contains='', timeout=120, config=None):
        folder = out/name; folder.mkdir()
        (folder/'build_yolo11/validation').mkdir(parents=True)
        (folder/'command.txt').write_text(shlex.join(map(str, command))+'\n')
        (folder/'commit.txt').write_text(subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True))
        if config:
            (folder/'effective.yaml').write_bytes(config.read_bytes())
            (folder/'config.sha256').write_text(hashlib.sha256(config.read_bytes()).hexdigest()+'\n')
        start = time.monotonic()
        with (folder/'run.log').open('w') as log:
            try:
                result = subprocess.run(list(map(str, command)), cwd=folder, env=env,
                                        stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
                code = result.returncode
            except subprocess.TimeoutExpired:
                code = 124
        text = (folder/'run.log').read_text(errors='replace')
        entry = dict(name=name, exit_code=code, duration_seconds=time.monotonic()-start,
                     passed=code==expected and contains in text)
        summary.append(entry)
        (folder/'exit_code.txt').write_text(str(code)+'\n')
        (folder/'duration.txt').write_text(str(entry['duration_seconds'])+'\n')
        (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(name, code, flush=True)
        assert entry['passed'], (name, text[-5000:])
        return folder

    tests = ['instance_association_test', 'identity_state_integration_test',
             'b4_evidence_expiry_test','b4_state_machine_test','instance_track_lifecycle_test',
             'frontend_diagnostics_test','keyframe_decision_guard_test','map_admission_test',
             'fine_feature_policy_test','coarse_feature_filter_test','coarse_pose_quality_test',
             'feature_policy_test','feature_semantic_test','frontend_mask_test','b2_candidate_test']
    for name in tests: run(name, [build/'bin'/name])
    for name in NAMES:
        config = CONFIGS/(name+'.yaml')
        run('config_'+name,[build/'bin/yolo_seg_smoke','--config-only',config], config=config)
    base = (ROOT/'configs/frontend_modes/rgbd_tum3/mode0_original.yaml').read_text()
    for name, field, code in [
            ('default', '', 0), ('negative_mode','Mode: -1',2), ('large_mode','Mode: 4',2),
            ('zero_cost','MaxCost: 0',2), ('nan_margin','MinMargin: .Nan',2),
            ('forced_pair','MaxCost: 0.90',2), ('bad_evidence','MinEvidenceTerms: 1',2),
            ('fractional_mode','Mode: 1.5',2)]:
        config = out/(name+'.yaml'); config.write_text(base+'\n'+('InstanceAssociation.'+field+'\n' if field else ''))
        run('invalid_'+name,[build/'bin/yolo_seg_smoke','--config-only',config],code,
            'Mode=3' if name=='default' else 'InstanceAssociation',config=config)

    if a.compatibility:
        for name in ['mode0_original','mode1_b1_hard_mask','mode2_evidence_only']:
            source=ROOT/'configs/frontend_modes/rgbd_tum3'/(name+'.yaml')
            text=source.read_text()
            for key in ['Yolo.ShowMask','Yolo.SaveMask','Yolo.SaveOverlay',
                        'DynamicFrontend.Debug.ShowOverlay','DynamicFrontend.Debug.SaveOverlay']:
                text=re.sub(r'^'+re.escape(key)+r':.*$',key+': 0',text,flags=re.M)
            if name=='mode0_original':
                text=re.sub(r'^Yolo.ModelPath:.*$','Yolo.ModelPath: "/nonexistent/no_model_needed.onnx"',text,flags=re.M)
            config=out/(name+'.yaml');config.write_text(text)
            folder=run('compat_'+name,[build/'bin/ablation_sequence','rgbd',ROOT/'Vocabulary/ORBvoc.txt',
                config,'/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_sitting_static',
                '/home/aaa/dynamic_slam_experiments/associations/fr3_sitting_static_associations.txt','3','0','1'],
                contains='[RUN] frames=3',timeout=120,config=config)
            if name=='mode0_original': assert '[YOLO] loaded' not in (folder/'run.log').read_text()
    real = []
    def audit(folder, mode, expected):
        rows = list(csv.DictReader((folder/'dynamic_frontend.csv').open()))
        frames = [r for r in rows if r['row_type']=='FRAME']
        instances = [r for r in rows if r['row_type']=='INSTANCE' and int(r['instance_id'])>=0]
        assert len(frames)==expected
        assoc = list(csv.DictReader(next(folder.glob('instance_association_*.csv')).open()))
        visible = [r for r in assoc if r['row_type']=='INSTANCE']
        assert len({(r['frame_id'],r['track_id']) for r in visible})==len(visible)
        assert all(float(r['previous_matched_frame'])==float(r['frame_id'])-1
                   for r in visible if r['allow_geometry_pair']=='1')
        b4 = list(csv.DictReader(next(folder.glob('b4_state_summary_*.csv')).open()))
        assert len({(r['frame_id'],r['track_id']) for r in b4})==len(b4)
        if mode==3:
            for r in instances:
                if r['allow_state_inheritance']=='0':
                    assert r['previous_b4_state']=='P' and r['allow_coarse']=='0'
                    assert r['current_b4_state']=='P' and r['allow_fine']==r['allow_map']=='0'
                    assert r['coarse_S_feature_uses']==r['fine_S_feature_uses']=='0'
                if r['allow_geometry_pair']=='0': assert r['b2_reliable']=='0'
        fine = list(csv.DictReader(next(folder.glob('fine_feature_policy_*.csv')).open()))
        assert all(int(r[k])==0 for r in fine for k in ['P_at_optimizer','D_at_optimizer','forbidden_at_optimizer'])
        maps = list(csv.DictReader((folder/'map_admission_final_audit.csv').open()))
        assert len(maps)==1 and int(maps[0]['forbidden_observations'])==int(maps[0]['unaligned_keyframes'])==0
        for name in ['CameraTrajectory.txt','KeyFrameTrajectory.txt']: assert (folder/name).stat().st_size
        stats = dict(run=folder.name, frames=len(frames), instances=len(visible),
            camera_poses=len((folder/'CameraTrajectory.txt').read_text().splitlines()),
            status=dict(Counter(r['association_status'] for r in visible)),
            events=dict(Counter(r['association_status'] for r in assoc if r['row_type']=='EVENT')),
            states=dict(Counter(r['current_state'] for r in b4)),
            untrusted=sum(r['allow_state_inheritance']=='0' for r in visible),
            geometry_blocked=sum(r['allow_geometry_pair']=='0' for r in visible),
            max_track_id=max((int(r['track_id']) for r in visible), default=-1))
        real.append(stats); (out/'sequence_summary.json').write_text(json.dumps(real,indent=2)+'\n')
        print(json.dumps(stats),flush=True)

    if a.short_sequences or a.full_sequences:
        dataset = Path('/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_sitting_static')
        association = Path('/home/aaa/dynamic_slam_experiments/associations/fr3_sitting_static_associations.txt')
        if dataset.is_dir() and association.is_file():
            config=CONFIGS/'A3_controlled.yaml'
            folder=run('system_reset', [build/'bin/identity_system_reset_test',ROOT/'Vocabulary/ORBvoc.txt',
                config,dataset,association],timeout=180,config=config)
            resets=list(csv.DictReader(next(folder.glob('instance_association_*.csv')).open()))
            visible=[r for r in resets if r['row_type']=='INSTANCE']
            sequences=sorted({int(r['sequence_id']) for r in visible})
            assert len(sequences)==3
            previous_ids=set()
            for sequence in sequences:
                current={int(r['track_id']) for r in visible if int(r['sequence_id'])==sequence}
                assert not previous_ids.intersection(current)
                previous_ids.update(current)
                first=min(int(r['frame_id']) for r in visible if int(r['sequence_id'])==sequence)
                assert all(r['allow_state_inheritance']==r['allow_geometry_pair']=='0' for r in visible
                    if int(r['sequence_id'])==sequence and int(r['frame_id'])==first)
        stages = [(100, list(range(4)))]
        if a.full_sequences: stages.append((0,[3]))
        for limit, modes in stages:
            for mode in modes:
                for seq in ['sitting_static','walking_xyz']:
                    dataset = Path('/home/aaa/Datasets/TUM')/('rgbd_dataset_freiburg3_'+seq)
                    association = Path('/home/aaa/dynamic_slam_experiments/associations')/('fr3_'+seq+'_associations.txt')
                    if not dataset.is_dir() or not association.is_file():
                        (out/('NOT_RUN_'+seq+'.txt')).write_text(f'Missing {dataset} or {association}\n')
                        continue
                    config = CONFIGS/(NAMES[mode]+'.yaml')
                    cmd = [build/'bin/ablation_sequence','rgbd',ROOT/'Vocabulary/ORBvoc.txt',config,
                           dataset,association,str(limit),'0','1']
                    folder = run(f'{NAMES[mode]}_{seq}_{limit or "full"}',cmd,timeout=900,config=config)
                    expected = limit or sum(bool(l.strip()) and not l.lstrip().startswith('#')
                                            for l in association.read_text().splitlines())
                    audit(folder,mode,expected)
    print('PASS requested identity-association regressions',flush=True)


if __name__=='__main__': main()
