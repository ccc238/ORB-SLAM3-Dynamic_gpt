#!/usr/bin/env python3
"""Check per-frame annotation invariants and unchanged extraction counts."""
import argparse
import json
from pathlib import Path
import re


def records(directory, prefix):
    rows = [dict(re.findall(r'(\w+)=([^ ]+)', line)) for line in
            (directory / 'run.log').read_text().splitlines() if line.startswith(prefix)]
    result = {int(r['frame_id']): r for r in rows}
    assert len(rows) == len(result), 'Duplicate frame diagnostics'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_directory', type=Path)
    parser.add_argument('--reference', type=Path, required=True)
    args = parser.parse_args()
    summary = json.loads((args.batch_directory / 'summary.json').read_text())
    report = []
    for case in summary:
        directory = Path(case['directory'])
        annotated = records(directory, '[FEATURE SEMANTICS]')
        extracted = records(directory, '[DYNAMIC FRONTEND]')
        reference = records(args.reference / directory.name, '[DYNAMIC FRONTEND]')
        assert case['passed'] and len(annotated) == case['expected_frames']
        assert annotated.keys() == extracted.keys() == reference.keys()
        totals = {k: 0 for k in ('background','person_interior','person_boundary','unknown')}
        for frame, row in annotated.items():
            assert int(row['N']) == int(extracted[frame]['final_orb']) == int(row['normal'])
            assert row['excluded'] == '0' and row['aligned'] == '1'
            assert sum(int(row[k]) for k in totals) == int(row['N'])
            assert all(extracted[frame][k] == reference[frame][k] for k in ('raw_fast','final_orb','b1_mask_rejected'))
            for k in totals:
                totals[k] += int(row[k])
        assert all(totals[k] > 0 for k in ('background','person_interior','person_boundary'))
        report.append(dict(directory=str(directory), passed=True, frames=len(annotated),
                           feature_region_totals=totals, excluded_features=0,
                           extraction_counts_match_annotation_disabled_reference=True))
    assert len(report) == 4
    with (args.batch_directory / 'feature_semantic_audit.json').open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
