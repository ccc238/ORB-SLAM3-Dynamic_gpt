#!/usr/bin/env python3
"""Reproducible multi-class checks; never overwrite existing run directories."""
import argparse
import ast
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--build-dir', type=Path, default=ROOT / 'build_multiclass')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--short-sequences', action='store_true')
    args = parser.parse_args()
    build = args.build_dir.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(QT_QPA_PLATFORM='offscreen', LIBGL_ALWAYS_SOFTWARE='1')
    env['LD_LIBRARY_PATH'] = ':'.join(str(p) for p in (
        build / 'lib', ROOT / 'third_party_ort/onnxruntime-linux-x64-1.17.3/lib',
        ROOT / 'Thirdparty/DBoW2/lib', ROOT / 'Thirdparty/g2o/lib')) + ':' + env.get('LD_LIBRARY_PATH', '')
    summary = []

    def run(name, command, expected=0, contains='', config=None, timeout=180):
        folder = output / name
        folder.mkdir()
        (folder / 'command.txt').write_text(shlex.join(map(str, command)) + '\n')
        (folder / 'commit.txt').write_text(subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True))
        if config:
            (folder / 'effective.yaml').write_bytes(config.read_bytes())
            (folder / 'config.sha256').write_text(hashlib.sha256(config.read_bytes()).hexdigest() + '\n')
        start = time.monotonic()
        with (folder / 'run.log').open('w') as log:
            try:
                code = subprocess.run(list(map(str, command)), cwd=folder, env=env,
                                      stdout=log, stderr=subprocess.STDOUT, timeout=timeout).returncode
            except subprocess.TimeoutExpired:
                code = 124
        text = (folder / 'run.log').read_text(errors='replace')
        passed = code == expected and contains in text
        summary.append(dict(name=name, exit_code=code, passed=passed,
                            duration_seconds=time.monotonic() - start))
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        (folder / 'exit_code.txt').write_text(str(code) + '\n')
        print(name, code, 'PASS' if passed else 'FAIL', flush=True)
        if not passed:
            raise RuntimeError(text[-5000:])
        return folder, text

    run('multiclass_frontend_test', [build / 'bin/multiclass_frontend_test'], contains='PASS')
    configs = ROOT / 'configs/multiclass/rgbd_tum3'
    for group in ['person_only_action', 'selected_classes_action', 'selected_classes_evidence']:
        config = configs / (group + '.yaml')
        _, text = run('config_' + group, [build / 'bin/yolo_seg_smoke', '--config-only', config], config=config)
        metadata = re.search(r'^\[YOLO\] model_class_names=(.*)$', text, re.M)
        if metadata:
            labels = ast.literal_eval(metadata.group(1))
            raw = config.read_text()
            ids = ast.literal_eval(re.search(r'^Yolo.DynamicClasses:\s*(.*)$', raw, re.M).group(1))
            names = ast.literal_eval(re.search(r'^Semantic.SelectedClassNames:\s*(.*)$', raw, re.M).group(1))
            assert [labels[i] for i in ids] == names, 'Selected names disagree with actual model metadata'

    base = (configs / 'selected_classes_action.yaml').read_text()
    invalid = [
        ('duplicate_ids', 'Yolo.DynamicClasses', '[0, 0]', 'duplicate/out-of-range'),
        ('out_of_range', 'Yolo.DynamicClasses', '[80]', 'duplicate/out-of-range'),
        ('fractional_ids', 'Yolo.DynamicClasses', '[0.5]', 'must be integers'),
        ('wrong_names', 'Semantic.SelectedClassNames', '["person"]', 'must align'),
        ('no_yolo', 'Yolo.Enable', '0', 'requires'),
        ('missing_model', 'Yolo.ModelPath', '"/nonexistent/multiclass_model.onnx"', 'does not exist'),
        ('wrong_shape', 'Yolo.NumClasses', '81', 'layout/class count'),
    ]
    for name, key, value, message in invalid:
        config = output / (name + '.yaml')
        raw = re.sub(r'^' + re.escape(key) + r':.*$', key + ': ' + value, base, flags=re.M)
        config.write_text(raw)
        run('invalid_' + name, [build / 'bin/yolo_seg_smoke', '--config-only', config],
            expected=2, contains=message, config=config)

    if args.short_sequences:
        for sequence in ['sitting_static', 'walking_xyz']:
            dataset = Path('/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_' + sequence)
            association = Path('/home/aaa/dynamic_slam_experiments/associations/fr3_' + sequence + '_associations.txt')
            if not dataset.is_dir() or not association.is_file():
                summary.append(dict(name=sequence, skipped=True, reason='missing dataset or associations'))
                (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
                continue
            config = configs / 'selected_classes_action.yaml'
            folder, _ = run('short_' + sequence, [build / 'bin/ablation_sequence', 'rgbd',
                ROOT / 'Vocabulary/ORBvoc.txt', config, dataset, association, 100, 0, 0],
                contains='[RUN] frames=100', config=config, timeout=300)
            fine_file = next(folder.glob('fine_feature_policy_*.csv'))
            fine = list(csv.DictReader(fine_file.open()))
            assert fine and all(int(row[key]) == 0 for row in fine for key in (
                'P_at_optimizer', 'D_at_optimizer', 'forbidden_at_optimizer'))
            maps = list(csv.DictReader((folder / 'map_admission_final_audit.csv').open()))
            assert maps and all(int(row['forbidden_observations']) == 0 and
                                int(row['unaligned_keyframes']) == 0 for row in maps)
            for trajectory in ['CameraTrajectory.txt', 'KeyFrameTrajectory.txt']:
                assert (folder / trajectory).stat().st_size > 0
            print(sequence, 'optimizer/map audit PASS', flush=True)


if __name__ == '__main__':
    main()
