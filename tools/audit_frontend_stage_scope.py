#!/usr/bin/env python3
"""Audit batch-2 scope against the frozen source and extraction logs."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_directory', type=Path)
    args = parser.parse_args()
    frozen = ROOT / 'freezes/pre_two_stage_20261022_104737'
    hashes = json.loads((frozen / 'source_sha256.json').read_text())
    paths = ('src/ORBextractor.cc', 'src/Frame.cc', 'src/B2MotionEvidence.cc',
             'src/B3SemanticGeometryFusion.cc', 'src/B4TemporalStateMachine.cc',
             'src/Deep/InstanceTrackManager.cc', 'src/LocalMapping.cc', 'src/LoopClosing.cc',
             'src/Optimizer.cc', 'src/MapPoint.cc', 'src/KeyFrame.cc',
             'Examples/RGB-D/TUM3.yaml', 'Examples/RGB-D/TUM3_B4_balanced_validation.yaml')
    checks = {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == hashes[p] for p in paths}
    with tarfile.open(frozen / 'source_snapshot.tar.gz', 'r:gz') as archive:
        old = archive.extractfile('src/Tracking.cc').read().decode()
    new = (ROOT / 'src/Tracking.cc').read_text()
    # Entry-level mask selection/diagnostics are the ONLY allowed Tracking edits.
    start = 'Sophus::SE3f Tracking::GrabImageRGBD('
    end = 'void Tracking::RunB3SemanticGeometryFusion('
    checks['Tracking.outside_GrabImageRGBD_unchanged'] = (
        old[:old.index(start)] == new[:new.index(start)] and
        old[old.index(end):] == new[new.index(end):])
    comparisons = []
    for phase in ('short100', 'full'):
        for seq in ('sitting_static', 'walking_xyz'):
            extracted = {}
            for mode in ('0', '1', '2'):
                directory = args.batch_directory / f'{phase}_mode_{mode}_{seq}'
                lines = [dict(re.findall(r'(\w+)=([^ ]+)', line)) for line in
                         (directory / 'run.log').read_text().splitlines()
                         if line.startswith('[DYNAMIC FRONTEND]')]
                extracted[mode] = {int(r['frame_id']): r for r in lines}
                checks[f'{directory.name}.mode_and_mask'] = bool(lines) and all(
                    r['dynamic_frontend_mode'] == mode and
                    (mode == '1' or int(r['b1_mask_rejected']) == 0) for r in lines)
                # Main B1 features must never lie inside the raw person union.
                if mode == '1':
                    checks[f'{directory.name}.person_excluded'] = all(int(r['person_main_orb']) == 0 for r in lines)
            baseline, evidence = extracted['0'], extracted['2']
            identical = baseline.keys() == evidence.keys() and all(
                all(baseline[i][k] == evidence[i][k] for k in ('raw_fast', 'final_orb')) for i in baseline)
            checks[f'{phase}.{seq}.mode0_mode2_extraction_counts_equal'] = identical
            comparisons.append(dict(phase=phase, sequence=seq, frames=len(evidence),
                                    extraction_counts_equal=identical,
                                    person_orb_sum=sum(int(r['person_main_orb']) for r in evidence.values())))
    result = dict(passed=all(checks.values()), checks=checks, comparisons=comparisons)
    with (args.batch_directory / 'scope_audit.json').open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps(result, indent=2))
    if not result['passed']:
        raise RuntimeError('Scope/extraction audit failed')


if __name__ == '__main__':
    main()
