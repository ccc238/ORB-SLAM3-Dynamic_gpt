#!/usr/bin/env python3
"""Audit B2->B3->B4 evidence blocking using saved logs, not motion truth."""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results', type=Path)
    args = parser.parse_args()
    report = []
    for folder in sorted(args.results.glob('full_*')):
        if not (folder/'exit_code.txt').is_file():
            continue
        assert (folder/'exit_code.txt').read_text().strip() == '0', folder
        b2 = rows(next(folder.glob('b2_evidence_*/b2_instance_evidence.csv')))
        b3 = rows(next(folder.glob('b3_fusion_summary_*.csv')))
        b4 = rows(next(folder.glob('b4_state_summary_*.csv')))
        key = lambda r: (int(r['frame_id']), int(r['track_id']))
        fusion = {key(r): r for r in b3}
        temporal = {key(r): r for r in b4}
        assert len(fusion) == len(b3) and len(temporal) == len(b4), 'Duplicate per-frame state update'
        previous = {}
        blocked = {key(r) for r in b2 if r['evidence_update_blocked'] == '1'}
        frozen_checks = 0
        for r in b4:
            track = int(r['track_id'])
            counters = tuple(int(r[k]) for k in ['static_count', 'dynamic_count', 'recovery_count'])
            if key(r) in blocked:
                before = previous.get(track, (0, 0, 0))
                assert all(now <= old for now, old in zip(counters, before)), (folder, r)
                frozen_checks += 1
            previous[track] = counters
        for r in b2:
            blocked_now = r['evidence_update_blocked'] == '1'
            assert (r['geometry_reliable'] == '0') == blocked_now
            assert int(r['valid_count']) <= int(r['matched_count']) <= int(r['candidate_count'])
            if r['reliability_enabled'] == '1':
                assert int(r['match_invalid_count']) == int(r['candidate_count']) - int(r['matched_count'])
                assert int(r['depth_invalid_count']) + int(r['depth_edge_count']) <= int(r['matched_count'])
                if int(r['numeric_invalid_count']):
                    assert blocked_now
            for field in ['median_r3d', 'median_normalized_r3d', 'high_residual_ratio', 'background_sigma_r3d']:
                assert math.isfinite(float(r[field])), (folder, field, r)
            if blocked_now:
                assert fusion[key(r)]['fusion_reliable'] == '0'
                assert temporal[key(r)]['b3_reliable'] == '0'
        report.append(dict(run=folder.name, instance_frames=len(b2),
                           blocked_instance_frames=len(blocked), frozen_counter_checks=frozen_checks,
                           b4_rows=len(b4),
                           geometry_states=dict(Counter(r['geometry_state'] for r in b2)),
                           geometry_reasons=dict(Counter(r['decision_reason'] for r in b2)),
                           b4_states=dict(Counter(r['current_state'] for r in b4)),
                           b4_transitions=dict(Counter(r['transition_reason'] for r in b4)),
                           coarse_failure_reasons=dict(Counter(r['fallback_reason'] for r in rows(
                               next(folder.glob('coarse_pose_quality_*.csv'))) if r['reliable']=='0')),
                           passed=True))
    if not report:
        raise RuntimeError('No completed full runs found')
    path = args.results/'evidence_blocking_audit.json'
    if path.exists():
        raise RuntimeError('Refusing to overwrite '+str(path))
    path.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
