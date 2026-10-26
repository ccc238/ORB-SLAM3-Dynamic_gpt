#!/usr/bin/env python3
"""Check pre-Frame identities against B2/B3/B4 CSV and once-only updates."""
import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import re


def rows(directory, pattern):
    paths = list(directory.glob(pattern))
    if len(paths) != 1:
        raise RuntimeError(f'Expected one {pattern} under {directory}')
    with paths[0].open() as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_directory', type=Path)
    args = parser.parse_args()
    summary = json.loads((args.batch_directory / 'summary.json').read_text())
    audits = []
    for run in summary:
        directory = Path(run['directory'])
        ttl_match = re.search(r'^B4.TrackTTL:\s*(\d+)', (directory / 'effective_config.yaml').read_text(), re.M)
        ttl = int(ttl_match[1]) if ttl_match else 15
        expected = {}
        frame_ids = set()
        pending = None
        for line in (directory / 'run.log').read_text().splitlines():
            if line.startswith('[INSTANCE TRACK]'):
                assert pending is None, 'Association dispatched twice before one Frame'
                fields = dict(re.findall(r'(\w+)=([^ ]*)', line))
                ids = [int(x) for x in fields['track_ids'].split(';') if x]
                assert len(ids) == int(fields['persons']) == len(set(ids))
                assert all(i >= 0 for i in ids)
                pending = (ids, float(fields['timestamp']))
            if line.startswith('[DYNAMIC FRONTEND]'):
                assert pending is not None, 'Frame constructed before IDs were assigned'
                frame = int(re.search(r'frame_id=(\d+)', line)[1])
                assert frame not in frame_ids
                frame_ids.add(frame)
                for instance, track in enumerate(pending[0]):
                    expected[(frame, instance)] = (track, pending[1])
                pending = None
        assert pending is None and len(frame_ids) == run['expected_frames']
        b2 = rows(directory, 'b2_evidence_*/b2_instance_evidence.csv')
        b3 = rows(directory, 'b3_fusion_summary_*.csv')
        b4_all = rows(directory, 'b4_state_summary_*.csv')
        b4 = [r for r in b4_all if int(r['instance_id']) >= 0]
        for records in (b2, b3, b4):
            keys = [(int(r['frame_id']), int(r['instance_id'])) for r in records]
            assert len(keys) == len(set(keys)), 'Duplicate instance update'
            for key, row in zip(keys, records):
                assert key in expected and int(row['track_id']) == expected[key][0], 'Shared ID mismatch'
                assert abs(float(row['timestamp']) - expected[key][1]) < 1e-6
        assert {(int(r['frame_id']), int(r['instance_id'])) for r in b3} == set(expected)
        assert {(int(r['frame_id']), int(r['instance_id'])) for r in b4} == set(expected)
        multiplicity = Counter((r['frame_id'], r['track_id']) for r in b4_all)
        assert all(n == 1 for n in multiplicity.values()), 'B4 state update/log doubled'
        assert all(int(r['missed_frames']) == 0 for r in b4), 'Detected/unreliable is not missing'
        assert all(r['applied_policy'] == 'NONE_EVIDENCE_ONLY' for r in b4_all)
        tracks = defaultdict(list)
        for row in b4:
            tracks[int(row['track_id'])].append(row)
        recoveries = []
        frozen_updates = 0
        for track, records in tracks.items():
            records.sort(key=lambda r: int(r['frame_id']))
            previous = None
            for row in records:
                age = int(row['track_age'])
                assert age == (int(previous['track_age']) + 1 if previous else 1), 'Age advanced more than once'
                if row['b3_reliable'] == '0':
                    frozen_updates += 1
                    assert row['previous_state'] == row['current_state'], 'Unreliable changed state'
                    if previous:
                        assert all(row[k] == previous[k] for k in ('static_count', 'dynamic_count', 'recovery_count'))
                if previous:
                    gap = int(row['frame_id']) - int(previous['frame_id']) - 1
                    if gap:
                        assert gap <= ttl, 'Expired ID unexpectedly restored'
                        recoveries.append(dict(track_id=track, frame_id=int(row['frame_id']), missed_frames=gap))
                previous = row
        audits.append(dict(directory=str(directory), passed=True, frames=len(frame_ids),
                           visible_instance_frames=len(b4), max_updates_per_frame_track=max(multiplicity.values(), default=0),
                           frozen_updates=frozen_updates, track_ttl=ttl,
                           ttl_recoveries=recoveries))
    assert len(audits) == 4 and all(run['passed'] for run in summary), 'Expected both short and full runs'
    with (args.batch_directory / 'shared_track_audit.json').open('x') as stream:
        json.dump(audits, stream, indent=2)
        stream.write('\n')
    print(json.dumps(audits, indent=2))


if __name__ == '__main__':
    main()
