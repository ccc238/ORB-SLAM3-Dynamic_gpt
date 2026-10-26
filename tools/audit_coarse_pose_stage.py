#!/usr/bin/env python3
"""Audit batch-6 ordering, branch preservation and once-only evidence."""
import argparse
from collections import Counter, defaultdict
import csv
import difflib
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / 'results/frontend_stage_validation/batch5_feature_policy_20261022_164439'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_directory', type=Path)
    parser.add_argument('--short-only', action='store_true')
    args = parser.parse_args()
    summary = json.loads((args.batch_directory / 'summary.json').read_text())
    if args.short_only:
        summary = [run for run in summary if run['phase'] == 'short100']
    with tarfile.open(REFERENCE / 'implemented_sources.tar.gz') as archive:
        original = archive.extractfile('src/Tracking.cc').read().decode()
    current = (ROOT / 'src/Tracking.cc').read_text()
    # Everything in Track up to the original common boundary is preserved,
    # including initialization, Atlas, motion/reference/relocalization branches.
    start = 'void Tracking::Track()'
    boundary = '        // If we have an initial estimation of the camera pose and matching.'
    old_prefix = original[original.index(start):original.index(boundary)]
    new_prefix = current[current.index(start):current.index('        // All original initial-tracking branches converge here.')]
    assert old_prefix == new_prefix, 'Original initial-tracking branches were changed'
    # Preserve all downstream SLAM algorithms. Only Track and CreateNewKeyFrame
    # receive timing/guard code; StereoInitialization receives a bootstrap log.
    functions = ('TrackReferenceKeyFrame', 'TrackWithMotionModel', 'Relocalization',
                 'TrackLocalMap', 'NeedNewKeyFrame', 'CreateMapInAtlas',
                 'MonocularInitialization')
    def function(source, name):
        match = re.search(r'^.*Tracking::' + name + r'\([^;]*?\)\s*\{', source, re.M)
        assert match, name
        tail = source[match.start():]
        next_function = re.search(r'\n(?:void|bool|Sophus::SE3f|int|float|double) Tracking::', tail[1:])
        return tail[:next_function.start()+1] if next_function else tail
    for name in functions:
        assert function(original, name) == function(current, name), name + ' changed'
    # B2/B3/optimization/backend algorithms are unchanged from the pre-stage
    # freeze. B4 already received earlier lifecycle changes; it is audited by
    # its unit test and shared-track CSV audit instead of this older hash set.
    frozen = ROOT / 'freezes/pre_two_stage_20261022_104737'
    hashes = json.loads((frozen / 'source_sha256.json').read_text())
    unchanged = ('src/B2MotionEvidence.cc', 'src/B3SemanticGeometryFusion.cc',
                 'src/LocalMapping.cc', 'src/Optimizer.cc', 'src/ORBmatcher.cc',
                 'src/MapPoint.cc', 'src/KeyFrame.cc', 'src/ORBextractor.cc')
    for name in unchanged:
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == hashes[name], name
    diff = ''.join(difflib.unified_diff(original.splitlines(True), current.splitlines(True),
                                      fromfile='batch5/src/Tracking.cc', tofile='batch6/src/Tracking.cc'))
    patch_path = args.batch_directory / 'tracking_changes_from_batch5.patch'
    if not patch_path.exists():
        patch_path.write_text(diff)
    else:
        assert patch_path.read_text() == diff, 'Source changed during validation'
    reports = []
    for run in summary:
        assert run['passed']
        directory = Path(run['directory'])
        frames = defaultdict(list)
        for line in (directory / 'run.log').read_text().splitlines():
            if line.startswith('[TWO STAGE]'):
                fields = dict(re.findall(r'(\w+)=([^ ]+)', line))
                frames[int(fields['frame'])].append(fields)
        assert len(frames) == run['expected_frames']
        ordinary = bootstrap = enqueued = invalid = 0
        for frame, events in frames.items():
            counts = Counter(r['event'] for r in events)
            assert counts['evidence_complete'] == counts['frame_complete'] == 1
            evidence = next(r for r in events if r['event'] == 'evidence_complete')
            assert evidence['b4_updates'] == '1' and evidence['applied'] == '0'
            names = [r['event'] for r in events]
            if counts['coarse_boundary']:
                ordinary += 1
                assert counts['coarse_boundary'] == counts['fine_complete'] == 1
                assert evidence['pose_source'] == 'coarse'
                assert names.index('coarse_boundary') < names.index('evidence_complete') < names.index('fine_complete')
                coarse = next(r for r in events if r['event'] == 'coarse_boundary')
                assert coarse['pose_reliable'] == evidence['pose_reliable']
                if coarse['pose_reliable'] == '1':
                    assert coarse['initial_success'] == coarse['pose_finite'] == '1'
                else:
                    invalid += 1
            else:
                bootstrap += 1
                assert evidence['pose_source'] == 'bootstrap_or_unavailable'
                assert evidence['pose_reliable'] == '0'
                assert counts['fine_complete'] == counts['keyframe_decision'] == counts['keyframe_enqueue'] == 0
            if counts['keyframe_decision']:
                decision = next(r for r in events if r['event'] == 'keyframe_decision')
                assert names.index('fine_complete') < names.index('keyframe_decision')
                # Preserve the original RECENTLY_LOST bookkeeping: NeedNewKeyFrame
                # may still be queried after a skipped fine stage, but no regular
                # RGB-D KeyFrame may be inserted without a completed fine stage.
                if decision['fine_completed'] == '0':
                    assert counts['keyframe_enqueue'] == 0
            if counts['keyframe_enqueue']:
                enqueued += 1
                enqueue = next(r for r in events if r['event'] == 'keyframe_enqueue')
                assert enqueue['fine_completed'] == '1'
                assert names.index('keyframe_decision') < names.index('keyframe_enqueue') < names.index('frame_complete')
        assert ordinary > 0 and bootstrap >= 1
        # Instance evidence CSVs are written once, after Track returns; they
        # must still have unique (frame, track) keys despite the earlier update.
        b4_paths = list(directory.glob('b4_state_summary_*.csv'))
        assert len(b4_paths) == 1
        with b4_paths[0].open() as stream:
            rows = list(csv.DictReader(stream))
        keys = [(r['frame_id'], r['track_id']) for r in rows]
        assert len(keys) == len(set(keys))
        assert all(r['applied_policy'] == 'NONE_EVIDENCE_ONLY' for r in rows)
        reports.append(dict(directory=str(directory), passed=True, frames=len(frames),
                            ordinary_frames=ordinary, bootstrap_or_unavailable_frames=bootstrap,
                            failed_coarse_frames=invalid, regular_keyframes_enqueued=enqueued,
                            max_evidence_updates=1, no_policy_applied=True))
    assert len(reports) == (2 if args.short_only else 4)
    report = dict(passed=True, original_initial_tracking_branches_unchanged=True,
                  unchanged_functions=list(functions), unchanged_algorithms=list(unchanged), runs=reports)
    report_name = 'coarse_pose_short_audit.json' if args.short_only else 'coarse_pose_stage_audit.json'
    with (args.batch_directory / report_name).open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
