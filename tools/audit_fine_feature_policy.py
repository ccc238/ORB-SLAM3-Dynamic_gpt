#!/usr/bin/env python3
"""Batch10: double fine-filtering, final optimizer inputs, failure guards and shared IDs."""
import argparse
from collections import Counter,defaultdict
import csv
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT=Path(__file__).resolve().parents[1]

def rows(directory,pattern):
    paths=list(directory.glob(pattern));assert len(paths)==1,(directory,pattern)
    with paths[0].open() as stream:return list(csv.DictReader(stream))

def function(source,name):
    match=re.search(r'^.*Tracking::'+name+r'\([^;]*?\)\s*\{',source,re.M);assert match,name
    tail=source[match.start():]
    end=re.search(r'\n(?:void|bool|Sophus::SE3f|int|float|double) Tracking::',tail[1:])
    return tail[:end.start()+1] if end else tail

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_directory',type=Path)
    args=parser.parse_args()
    with tarfile.open(ROOT/'results/frontend_stage_validation/batch10_fine_policy_build_01/pre_batch10_sources.tar.gz') as archive:
        before=archive.extractfile('src/Tracking.cc').read().decode()
        assert archive.extractfile('src/B4TemporalStateMachine.cc').read()==(ROOT/'src/B4TemporalStateMachine.cc').read_bytes()
    now=(ROOT/'src/Tracking.cc').read_text()
    unchanged=('TrackReferenceKeyFrame','TrackWithMotionModel','Relocalization','OptimizeNormalInitialPose',
        'FinalizeCoarsePoseQuality','RunRGBDGeometryEvidence','RunB4TemporalStateMachine',
        'CreateMapInAtlas','StereoInitialization','MonocularInitialization')
    for name in unchanged:assert function(before,name)==function(now,name),name+' unexpectedly changed'
    fine=function(now,'TrackLocalMap')
    assert fine.index('UpdateLocalMap();')<fine.index('SearchLocalPoints();')<fine.index('ApplyFineFeaturePolicy(')
    assert fine.index('ApplyFineFeaturePolicy(')<fine.index('VerifyFineOptimizerInputs(')<fine.index('Optimizer::PoseOptimization(')
    for name in ('NeedNewKeyFrame','CreateNewKeyFrame'):
        body=function(now,name)
        assert 'FinePolicyAllowsKeyFrame(mbFineOptimizationExecuted,mbFineTrackingFailed)' in body
        if name=='CreateNewKeyFrame':assert body.index('FinePolicyAllowsKeyFrame(')<body.index('new KeyFrame(')<body.index('new MapPoint(')
    helper=(ROOT/'src/FineFeaturePolicy.cc').read_text()
    assert not any(token in helper for token in ('SetBadFlag','new MapPoint','mDescriptors.row','mvKeys.erase'))
    frozen=json.loads((ROOT/'freezes/pre_two_stage_20261022_104737/source_sha256.json').read_text())
    for name in ('src/Optimizer.cc','src/ORBmatcher.cc','src/LocalMapping.cc','src/MapPoint.cc',
                 'src/KeyFrame.cc','src/ORBextractor.cc','src/B3SemanticGeometryFusion.cc'):
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==frozen[name],name
    with tarfile.open(ROOT/'results/frontend_stage_validation/batch8_explicit_b2_20261022_203400/final_implemented_sources.tar.gz') as archive:
        for name in ('src/B2MotionEvidence.cc','include/B2MotionEvidence.h'):
            assert archive.extractfile(name).read()==(ROOT/name).read_bytes(),name
    reports=[]
    for run in json.loads((args.batch_directory/'summary.json').read_text()):
        assert run['passed'],run
        directory=Path(run['directory']);active=run['mode']=='fine3'
        b4=rows(directory,'b4_state_summary_*.csv')
        multiplicity=Counter((r['frame_id'],r['track_id']) for r in b4)
        assert max(multiplicity.values(),default=0)<=1
        previous={}
        for r in b4:
            if int(r['instance_id'])<0:continue
            prior=previous.get(r['track_id'])
            assert int(r['track_age'])==(int(prior['track_age'])+1 if prior else 1)
            if r['b3_reliable']=='0':
                assert r['previous_state']==r['current_state']
                if prior:assert all(r[k]==prior[k] for k in ('static_count','dynamic_count','recovery_count'))
            previous[r['track_id']]=r
        b2=rows(directory,'b2_evidence_*/b2_instance_evidence.csv')
        b3=rows(directory,'b3_fusion_summary_*.csv')
        expected={(r['frame_id'],r['instance_id']):r['track_id'] for r in b3}
        for records in (b2,[r for r in b4 if int(r['instance_id'])>=0]):
            keys=[(r['frame_id'],r['instance_id']) for r in records]
            assert len(keys)==len(set(keys)) and all(expected[key]==r['track_id'] for key,r in zip(keys,records))
        assert all(r['applied_action']=='NONE_EVIDENCE_ONLY' for r in b3)
        log=(directory/'run.log').read_text()
        assert log.count('[B2 READONLY]')==run['expected_frames']
        events=defaultdict(list)
        for line in log.splitlines():
            if line.startswith('[TWO STAGE]') or line.startswith('[FINE POLICY]'):
                r=dict(re.findall(r'(\w+)=([^ ]+)',line))
                events[int(r['frame'])].append(r.get('event','fine_'+r.get('phase','')))
        for event in events.values():
            assert event.count('evidence_complete')==event.count('frame_complete')==1
        report=dict(directory=str(directory),phase=run['phase'],mode=run['mode'],sequence=run['sequence'],passed=True)
        if active:
            fine_rows=rows(directory,'fine_feature_policy_*.csv')
            grouped=defaultdict(list)
            for r in fine_rows:grouped[r['frame_id']].append(r)
            assert len(grouped)==run['expected_frames']-1
            for frame,records in grouped.items():
                assert [r['phase'] for r in records]==['pre_local_map','pre_pose_optimization','result']
                for r in records:
                    assert int(r['fine_matches_before'])-sum(int(r[k]) for k in ('fine_reject_P','fine_reject_D','fine_reject_boundary','fine_reject_unknown'))==int(r['fine_matches_after'])
                    assert r['P_at_optimizer']==r['D_at_optimizer']==r['forbidden_at_optimizer']=='0'
                assert records[-1]['fine_fallback']=='0' and records[-1]['optimizer_executed']=='1'
                event=events[int(frame)]
                order=['coarse_boundary','evidence_complete','fine_pre_local_map','fine_pre_pose_optimization','fine_result','fine_complete']
                assert [event.index(x) for x in order]==sorted(event.index(x) for x in order)
            for r in b4:
                expected_action='NONE_NO_CURRENT_INSTANCE' if int(r['instance_id'])<0 else (
                    'FINE_'+r['recommended_policy'] if r['frame_id'] in grouped else 'NONE_FINE_NOT_RUN')
                assert r['evidence_only']=='0' and r['applied_policy']==expected_action
            inputs=[r for r in fine_rows if r['phase']=='pre_pose_optimization']
            report.update(fine_frames=len(inputs),P_at_optimizer=0,D_at_optimizer=0,
                first_pass_rejected=sum(int(r[k]) for r in fine_rows if r['phase']=='pre_local_map'
                    for k in ('fine_reject_P','fine_reject_D','fine_reject_boundary','fine_reject_unknown')),
                second_pass_P_rejected=sum(int(r['fine_reject_P']) for r in inputs),
                second_pass_D_rejected=sum(int(r['fine_reject_D']) for r in inputs),
                second_pass_boundary_rejected=sum(int(r['fine_reject_boundary']) for r in inputs),
                second_pass_unknown_rejected=sum(int(r['fine_reject_unknown']) for r in inputs),fine_failure_frames=0)
        else:
            assert not list(directory.glob('fine_feature_policy_*.csv'))
            assert all(r['evidence_only']=='1' and r['applied_policy']=='NONE_EVIDENCE_ONLY' for r in b4)
        reports.append(report)
    with (args.batch_directory/'fine_policy_audit.json').open('x') as stream:
        json.dump(dict(passed=True,unchanged_tracking_functions=unchanged,runs=reports),stream,indent=2)
    print(json.dumps(reports,indent=2))

if __name__=='__main__':main()
