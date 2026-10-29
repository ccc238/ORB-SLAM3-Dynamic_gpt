#!/usr/bin/env python3
"""Fresh-output expiry unit/config regression and optional 100-frame RGB-D runs."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, default=ROOT/'build_b4_expiry')
    parser.add_argument('--output', type=Path, required=True, help='must not already exist')
    parser.add_argument('--short-sequences', action='store_true')
    args = parser.parse_args()
    build = args.build_dir.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    environment = os.environ.copy()
    environment['LD_LIBRARY_PATH'] = ':'.join(map(str, (
        build/'lib', ROOT/'Thirdparty/DBoW2/lib', ROOT/'Thirdparty/g2o/lib',
        ROOT/'third_party_ort/onnxruntime-linux-x64-1.17.3/lib')))+':'+environment.get('LD_LIBRARY_PATH', '')
    environment['QT_QPA_PLATFORM'] = 'offscreen'
    environment['LIBGL_ALWAYS_SOFTWARE'] = '1'
    summary = []

    def run(name, command, expected=0, text_required='', timeout=120):
        folder = output/name
        folder.mkdir()
        (folder/'build_yolo11/validation').mkdir(parents=True)
        (folder/'command.txt').write_text(shlex.join(map(str, command))+'\n')
        with (folder/'run.log').open('w') as log:
            result = subprocess.run(list(map(str, command)), cwd=folder, env=environment,
                                    stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
        text = (folder/'run.log').read_text()
        summary.append({'name': name, 'exit_code': result.returncode,
                        'passed': result.returncode == expected and text_required in text})
        (folder/'exit_code.txt').write_text(str(result.returncode)+'\n')
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(name, result.returncode, flush=True)
        assert summary[-1]['passed'], (name, text[-5000:])
        return folder

    tests = ['b4_evidence_expiry_test', 'b4_state_machine_test', 'instance_track_lifecycle_test',
             'frontend_diagnostics_test', 'keyframe_decision_guard_test', 'map_admission_test',
             'fine_feature_policy_test', 'coarse_feature_filter_test', 'coarse_pose_quality_test',
             'feature_policy_test', 'feature_semantic_test', 'frontend_mask_test', 'b2_candidate_test']
    for name in tests:
        run(name, [build/'bin'/name])

    # Config-only Mode 0 needs no model; new parameters are always validated.
    base = (ROOT/'configs/frontend_modes/rgbd_tum3/mode0_original.yaml').read_text()
    cases = [
        ('default_off', '', 0, '[B4 EVIDENCE EXPIRY CONFIG] Enable=0'),
        ('valid_on', 'Enable: 1\nMaxGapSeconds: 0.20\nMaxStaticAgeSeconds: 0.30', 0, 'Enable=1 MaxGapSeconds=0.2 MaxStaticAgeSeconds=0.3'),
        ('equal_limits', 'MaxGapSeconds: 0.25\nMaxStaticAgeSeconds: 0.25', 0, 'MaxGapSeconds=0.25'),
        ('zero_gap', 'MaxGapSeconds: 0', 2, 'B4.EvidenceExpiry'),
        ('negative_age', 'MaxStaticAgeSeconds: -0.1', 2, 'B4.EvidenceExpiry'),
        ('nan_gap', 'MaxGapSeconds: .Nan', 2, 'B4.EvidenceExpiry'),
        ('inf_age', 'MaxStaticAgeSeconds: .Inf', 2, 'B4.EvidenceExpiry'),
        ('reversed', 'MaxGapSeconds: 0.40\nMaxStaticAgeSeconds: 0.30', 2, 'B4.EvidenceExpiry'),
        ('fractional_switch', 'Enable: 0.5', 2, 'B4.EvidenceExpiry.Enable'),
        ('string_gap', 'MaxGapSeconds: "bad"', 2, 'B4.EvidenceExpiry.MaxGapSeconds'),
    ]
    for name, fields, code, message in cases:
        path = output/(name+'.yaml')
        path.write_text(base+'\n'+'\n'.join('B4.EvidenceExpiry.'+line for line in fields.splitlines())+'\n')
        run('config_'+name, [build/'bin/yolo_seg_smoke', '--config-only', path], code, message)

    configs = ROOT/'configs/frontend_modes/rgbd_tum3'
    for switch in ('off', 'on'):
        config = configs/f'mode3_b4_evidence_expiry_{switch}.yaml'
        run('config_mode3_'+switch, [build/'bin/yolo_seg_smoke', '--config-only', config])

    if args.short_sequences:
        runs = []
        for switch in ('off', 'on'):
            source = configs/f'mode3_b4_evidence_expiry_{switch}.yaml'
            for sequence in ('sitting_static', 'walking_xyz'):
                folder = output/f'short100_{switch}_{sequence}'
                # Disable display and image saving equally for both ablations.
                text = source.read_text()
                for key in ('DynamicFrontend.Debug.ShowOverlay', 'DynamicFrontend.Debug.SaveOverlay'):
                    text = re.sub(r'^'+re.escape(key)+r':.*$', key+': 0', text, flags=re.M)
                config = output/f'effective_{switch}_{sequence}.yaml'
                config.write_text(text)
                dataset = Path('/home/aaa/Datasets/TUM')/('rgbd_dataset_freiburg3_'+sequence)
                association = Path('/home/aaa/dynamic_slam_experiments/associations')/('fr3_'+sequence+'_associations.txt')
                command = [build/'bin/ablation_sequence', 'rgbd', ROOT/'Vocabulary/ORBvoc.txt',
                           config, dataset, association, '100', '0', '1']
                started = time.monotonic()
                run(folder.name, command, timeout=180)
                (folder/'duration.txt').write_text(str(time.monotonic()-started)+'\n')
                (folder/'source_config.yaml').write_text(source.read_text())
                (folder/'effective_config.yaml').write_text(config.read_text())
                (folder/'commit.txt').write_text(subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True))
                (folder/'config.sha256').write_text(hashlib.sha256(config.read_bytes()).hexdigest()+'\n')
                rows = list(csv.DictReader((folder/'dynamic_frontend.csv').open()))
                frames = [row for row in rows if row['row_type'] == 'FRAME']
                instances = [row for row in rows if row['row_type'] == 'INSTANCE']
                assert len(frames) == 100
                assert all((folder/name).stat().st_size > 0 for name in ('CameraTrajectory.txt', 'KeyFrameTrajectory.txt'))
                b4 = list(csv.DictReader(next(folder.glob('b4_state_summary_*.csv')).open()))
                fine_rows = list(csv.DictReader(next(folder.glob('fine_feature_policy_*.csv')).open()))
                assert all(int(row[key]) == 0 for row in fine_rows
                           for key in ('P_at_optimizer', 'D_at_optimizer', 'forbidden_at_optimizer'))
                map_audit = list(csv.DictReader((folder/'map_admission_final_audit.csv').open()))
                assert len(map_audit) == 1
                assert int(map_audit[0]['forbidden_observations']) == int(map_audit[0]['unaligned_keyframes']) == 0
                assert len({(row['frame_id'], row['track_id']) for row in b4}) == len(b4)
                for row in b4:
                    assert row['transition_reason'] == row['current_update_reason']
                    if row['static_state_expired'] == '1':
                        assert row['expiry_previous_state'] == 'S' and row['expiry_current_state'] == 'P'
                    if row['has_reliable_history'] == '0': assert row['evidence_age_seconds'] == ''
                if switch == 'off': assert all(row['counter_expired'] == row['static_state_expired'] == '0' for row in b4)
                for row in instances:
                    if row['static_state_expired'] == '1':
                        assert row['previous_b4_state'] == 'P' and row['allow_coarse'] == '0'
                        if row['current_b4_state'] == 'P':
                            assert row['allow_fine'] == row['allow_map'] == '0'
                stats = {'switch': switch, 'sequence': sequence, 'frames': len(frames),
                         'camera_poses': sum(1 for line in (folder/'CameraTrajectory.txt').read_text().splitlines() if line.strip()),
                         'b4_rows': len(b4), 'states': dict(Counter(row['current_state'] for row in b4)),
                         'counter_expired': sum(row['counter_expired'] == '1' for row in b4),
                         'static_state_expired': sum(row['static_state_expired'] == '1' for row in b4),
                         'expiry_coarse_rejected': sum(row['static_state_expired'] == '1' and row['allow_coarse'] == '0' for row in instances)}
                runs.append(stats)
                (output/'short_sequence_summary.json').write_text(json.dumps(runs, indent=2)+'\n')
    print('PASS expiry unit/config regressions and requested short sequences', flush=True)


if __name__ == '__main__':
    main()
