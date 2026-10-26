#!/usr/bin/env python3
"""Audit proposal-only counts, partition definitions and no-action markers."""
import argparse
import csv
import json
from pathlib import Path
import re


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('batch_directory',type=Path)
    args=p.parse_args()
    summary=json.loads((args.batch_directory/'summary.json').read_text())
    audits=[]
    for run in summary:
        directory=Path(run['directory'])
        paths=list(directory.glob('feature_policy_summary_*.csv'))
        assert len(paths)==1
        with paths[0].open() as stream:
            rows=list(csv.DictReader(stream))
        assert len(rows)==run['expected_frames'] and run['passed']
        semantic={}
        for line in (directory/'run.log').read_text().splitlines():
            if line.startswith('[FEATURE SEMANTICS]'):
                fields=dict(re.findall(r'(\w+)=([^ ]+)',line))
                semantic[int(fields['frame_id'])]=fields
        assert len(semantic)==len(rows)
        seen=set()
        different=[]
        totals={k:0 for k in ('coarse_allowed_count','fine_allowed_count','map_allowed_count',
            'background_count','S_count','P_count','D_count','boundary_count','unknown_count')}
        for r in rows:
            frame=int(r['frame_id']); assert frame not in seen; seen.add(frame)
            values={k:int(v) for k,v in r.items() if k not in ('timestamp',)}
            assert all(v>=0 for v in values.values())
            n=values['N']; s=semantic[frame]
            assert values['evidence_only']==1 and values['policy_applied']==0 and values['actual_excluded_count']==0
            assert values['coarse_allowed_count']==values['background_count']+values['previous_S_count']
            assert values['fine_allowed_count']==values['map_allowed_count']==values['background_count']+values['S_count']
            assert sum(values[k] for k in ('background_count','S_count','P_count','D_count','boundary_count','unknown_count'))==n
            assert n==int(s['N'])==int(s['normal']) and s['excluded']=='0' and s['aligned']=='1'
            assert values['background_count']==int(s['background']) and values['boundary_count']==int(s['person_boundary'])
            assert values['unknown_count']>=int(s['unknown'])
            assert values['S_count']+values['P_count']+values['D_count']+values['unknown_count']-int(s['unknown'])==int(s['person_interior'])
            if frame==0:
                assert values['previous_S_count']==0 and values['S_count']==0
                assert values['coarse_allowed_count']==values['fine_allowed_count']==values['background_count']
            if values['coarse_allowed_count']!=values['fine_allowed_count']:
                different.append({k:values[k] for k in ('frame_id','coarse_allowed_count','fine_allowed_count','map_allowed_count','previous_S_count','S_count')})
            for k in totals: totals[k]+=values[k]
        audits.append(dict(directory=str(directory),csv=str(paths[0]),passed=True,frames=len(rows),
                           totals=totals,timing_difference_frames=len(different),timing_difference_examples=different[:10]))
    assert len(audits)==4
    with (args.batch_directory/'feature_policy_audit.json').open('x') as stream:
        json.dump(audits,stream,indent=2); stream.write('\n')
    print(json.dumps(audits,indent=2))


if __name__=='__main__': main()
