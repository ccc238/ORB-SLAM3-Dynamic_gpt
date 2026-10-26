#!/usr/bin/env python3
"""Audit batch11 artifacts without modifying existing run outputs."""
import argparse
from collections import Counter
import csv
import json
import re
import tarfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def function(source,name):
    match=re.search(r'^.*Tracking::'+name+r'\([^;]*?\)\s*\{',source,re.M)
    assert match,name
    tail=source[match.start():]
    end=re.search(r'\n(?:void|bool|Sophus::SE3f|int|float|double) Tracking::',tail[1:])
    return tail[:end.start()+1] if end else tail

def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch',type=Path)
    args=parser.parse_args()
    with tarfile.open(ROOT/'results/frontend_stage_validation/batch11_map_admission_build_01/pre_batch11_sources.tar.gz') as archive:
        for name in ('src/B2MotionEvidence.cc','src/B3SemanticGeometryFusion.cc',
                     'src/B4TemporalStateMachine.cc','src/MapPoint.cc','src/Optimizer.cc',
                     'src/FineFeaturePolicy.cc','src/FeaturePolicyEvaluator.cc'):
            assert archive.extractfile(name).read()==(ROOT/name).read_bytes(),name+' algorithm changed'
        before=archive.extractfile('src/Tracking.cc').read().decode()
    now=(ROOT/'src/Tracking.cc').read_text()
    for name in ('Track','TrackLocalMap','NeedNewKeyFrame','TrackWithMotionModel','TrackReferenceKeyFrame',
                 'Relocalization','RunRGBDGeometryEvidence','RunB4TemporalStateMachine','MonocularInitialization'):
        assert function(before,name)==function(now,name),name+' tracking logic changed'
    records=json.loads((args.batch/'summary.json').read_text())
    report=[]
    for record in records:
        folder=Path(record['directory'])
        if record['mode']!='map3':
            assert not list(folder.glob('map_admission_*.csv')), 'Disabled map admission wrote artifacts'
            continue
        paths=list(folder.glob('map_admission_[0-9]*.csv'))
        assert len(paths)==1
        events=rows(paths[0]);audit=rows(folder/'map_admission_final_audit.csv')
        assert len(audit)==1 and int(audit[0]['active_keyframes'])>0
        assert audit[0]['forbidden_observations']==audit[0]['unaligned_keyframes']=='0'
        totals=Counter()
        stages=Counter()
        for event in events:
            stages[event['stage']]+=1
            for key in ('map_candidates_total','map_reject_P','map_reject_D','map_reject_boundary',
                        'map_reject_unknown','map_points_created'):
                totals[key]+=int(event[key])
            assert sum(int(event[k]) for k in ('map_reject_P','map_reject_D','map_reject_boundary',
                       'map_reject_unknown'))<=int(event['map_candidates_total'])
        assert totals['map_points_created']>0 and totals['map_reject_P']>0
        feature_paths=list(folder.glob('feature_policy_*.csv'))
        fine_paths=list(folder.glob('fine_feature_policy_*.csv'))
        fine=rows(fine_paths[0]);inputs=[r for r in fine if r['phase']=='pre_pose_optimization']
        assert len(inputs)==record['expected_frames']-1
        assert all(r['P_at_optimizer']==r['D_at_optimizer']==r['forbidden_at_optimizer']=='0' for r in inputs)
        assert record['passed'] and record['exit_code']==0
        b4_paths=list(folder.glob('b4_state_summary_*.csv'))
        assert len(b4_paths)==1
        history={};seen=set()
        for state in rows(b4_paths[0]):
            key=(state['frame_id'],state['track_id'])
            assert key not in seen,'B4 updated twice per frame/track'
            seen.add(key)
            prior=history.get(state['track_id'])
            if prior and state['transition_reason']=='UNRELIABLE_EVIDENCE_FREEZE':
                assert state['current_state']==prior['current_state']
                assert all(state[k]==prior[k] for k in ('static_count','dynamic_count','recovery_count'))
            history[state['track_id']]=state
        report.append(dict(phase=record['phase'],sequence=record['sequence'],directory=str(folder),
            trajectory_rows=record['trajectory_rows'],tracking_counts=record['tracking_counts'],
            stage_event_rows=dict(stages),totals=dict(totals),final_audit=audit[0],
            fine_forbidden_at_optimizer=0))
    destination=args.batch/'map_admission_audit.json'
    if destination.exists():
        raise FileExistsError(destination)
    destination.write_text(json.dumps(report,indent=2)+'\n')
    columns=['phase','sequence','map_candidates_total','map_reject_P','map_reject_D',
        'map_reject_boundary','map_reject_unknown','map_points_created','active_keyframes',
        'forward_observations','reverse_observations','forbidden_observations','unaligned_keyframes']
    csv_path=args.batch/'map_admission_summary.csv'
    with csv_path.open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
        for result in report:
            values=dict(phase=result['phase'],sequence=result['sequence'],**result['totals'])
            values.update({k:result['final_audit'][k] for k in columns if k in result['final_audit']})
            writer.writerow(values)
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    main()
