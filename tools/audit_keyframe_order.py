#!/usr/bin/env python3
"""Batch12: prove final-stage provenance and reject unsafe regular keyframes."""
import argparse
from collections import Counter,defaultdict
import csv
import json
from pathlib import Path
import re
import tarfile

ROOT=Path(__file__).resolve().parents[1]

def records(folder,pattern):
    paths=list(folder.glob(pattern))
    assert len(paths)==1,(folder,pattern)
    with paths[0].open() as stream:return list(csv.DictReader(stream))

def function(source,name):
    match=re.search(r'^.*Tracking::'+name+r'\([^;]*?\)\s*\{',source,re.M);assert match,name
    tail=source[match.start():]
    end=re.search(r'\n(?:void|bool|Sophus::SE3f|int|float|double|KeyFrameGateResult) Tracking::',tail[1:])
    return tail[:end.start()+1] if end else tail

def audit_scope():
    archive_path=ROOT/'results/frontend_stage_validation/batch12_keyframe_order_build_01/pre_batch12_sources.tar.gz'
    with tarfile.open(archive_path) as archive:
        frozen=('src/B2MotionEvidence.cc','src/B3SemanticGeometryFusion.cc','src/B4TemporalStateMachine.cc',
            'src/MapPoint.cc','src/Optimizer.cc','src/LocalMapping.cc','src/ORBmatcher.cc','src/LoopClosing.cc',
            'src/MapAdmission.cc','src/Frame.cc','src/KeyFrame.cc','src/Deep/DynamicDetector.cc',
            'src/FineFeaturePolicy.cc','src/FeaturePolicyEvaluator.cc')
        for name in frozen:assert archive.extractfile(name).read()==(ROOT/name).read_bytes(),name
        before=archive.extractfile('src/Tracking.cc').read().decode()
        for entry in archive.getmembers():
            if entry.isfile() and entry.name.startswith('configs/frontend_modes/'):
                assert archive.extractfile(entry.name).read()==(ROOT/entry.name).read_bytes(),entry.name
    now=(ROOT/'src/Tracking.cc').read_text()
    unchanged=('TrackLocalMap','TrackWithMotionModel','TrackReferenceKeyFrame','Relocalization',
               'RunRGBDGeometryEvidence','RunB4TemporalStateMachine','FinalizeCoarsePoseQuality',
               'OptimizeNormalInitialPose','MonocularInitialization')
    for name in unchanged:assert function(before,name)==function(now,name),name
    # Verify the original heuristic, including all thresholds and early returns,
    # only receives an explicitly named parameter instead of a mutable counter.
    old=function(before,'NeedNewKeyFrame')
    old=old[old.index('{')+1:]
    old=old.replace('    if(mbRGBDEvidenceReady && mpDynamicDetector->Frontend().finePolicyEnable &&\n'
        '        !FinePolicyAllowsKeyFrame(mbFineOptimizationExecuted,mbFineTrackingFailed)) return false;\n','')
    new=function(now,'EvaluateOriginalKeyFrameNeed');new=new[new.index('{')+1:]
    strip=lambda s:re.sub(r'\s+','',re.sub(r'//[^\n]*|/\*.*?\*/','',s,flags=re.S))
    assert strip(old.replace('mnMatchesInliers','decisionInliers'))==strip(new),'Original KF heuristic changed'
    need=function(now,'NeedNewKeyFrame');create=function(now,'CreateNewKeyFrame')
    assert need.index('CurrentKeyFrameGate()')<need.index('EvaluateOriginalKeyFrameNeed(mKeyFrameDecision.finalInliers)')
    assert create.index('CurrentKeyFrameGate(true)')<create.index('SetNotStop(true)')<create.index('new KeyFrame(')<create.index('new MapPoint(')
    track=function(now,'Track')
    assert track.index('RunRGBDGeometryEvidence(coarseReliable')<track.index('bOK = TrackLocalMap();')
    assert track.index('bOK = TrackLocalMap();')<track.index('RecordKeyFrameStage("final_snapshot")')<track.index('NeedNewKeyFrame();')<track.index('CreateNewKeyFrame();')
    return dict(passed=True,frozen_sources=frozen,unchanged_functions=unchanged,original_keyframe_heuristic_unchanged=True)

def audit_run(folder,expected,require_map_admission=True):
    rows=records(folder,'keyframe_stage_*.csv');groups=defaultdict(list)
    for row in rows:groups[row['frame_id']].append(row)
    assert len(groups)==expected
    fine={r['frame_id']:r for r in records(folder,'fine_feature_policy_*.csv') if r['phase']=='result'}
    quality={r['frame_id']:r for r in records(folder,'coarse_pose_quality_*.csv')}
    fallback=set(r['frame_id'] for r in records(folder,'coarse_feature_filter_*.csv') if r['coarse_fallback']=='1')
    events=defaultdict(list)
    for line in (folder/'run.log').read_text().splitlines():
        if line.startswith('[TWO STAGE]') or line.startswith('[KEYFRAME STAGE]'):
            values=dict(re.findall(r'(\w+)=([^ ]+)',line))
            if 'frame' in values:events[values['frame']].append(values.get('event','kf_'+values.get('phase','')))
    summaries=[];regular=bootstrap=0
    for frame,group in groups.items():
        summaries_for_frame=[r for r in group if r['phase']=='frame_summary']
        assert len(summaries_for_frame)==1
        summary=summaries_for_frame[0];summaries.append(summary)
        if summary['bootstrap']=='1':
            bootstrap+=1
            assert summary['created']=='1' and summary['reason']=='BOOTSTRAP_INITIAL_MAP_REQUIRED'
            assert sum(r['phase']=='bootstrap' for r in group)==1
            assert not any(r['phase'] in ('need_decision','create_guard','created') for r in group)
            continue
        for row in group:
            if row['phase'] in ('need_decision','create_guard','created'):
                assert row['gate_allowed']=='1' and row['coarse_fallback']=='0' and row['coarse_reliable']=='1'
                assert all(row[k]=='1' for k in ('evidence_complete','fine_completed','optimizer_executed',
                    'fine_success','semantic_policy_available','pose_finite','final_pose_unchanged'))
                assert row['fine_failed']=='0'
                assert row['semantic_frame_id']==row['final_frame_id']==frame
                assert row['decision_inliers']==row['final_inliers']==row['current_inliers']==fine[frame]['fine_inliers']
                assert int(row['final_inliers'])>=int(row['required_static_support'])
                assert frame not in fallback and quality[frame]['reliable']=='1'
                event=events[frame]
                order=['coarse_boundary','evidence_complete','kf_final_snapshot','fine_complete','kf_need_decision']
                assert [event.index(x) for x in order]==sorted(event.index(x) for x in order)
        created=[r for r in group if r['phase']=='created']
        assert len(created)<=1
        if created:
            regular+=1
            phases=[r['phase'] for r in group]
            assert phases.index('final_snapshot')<phases.index('need_decision')<phases.index('create_guard')<phases.index('created')<phases.index('frame_summary')
            assert all(created[0][k]=='1' for k in ('need_evaluated','needed','create_attempted','created'))
        assert int(summary['created'])==len(created)
        if frame in fallback or quality.get(frame,{}).get('reliable')=='0':
            assert summary['created']=='0' and summary['need_evaluated']=='0'
    # Frontend per-frame insertion flags are independent of the CSV stage logger.
    front={dict(re.findall(r'(\w+)=([^ ]+)',line))['frame_id']:dict(re.findall(r'(\w+)=([^ ]+)',line))
        for line in (folder/'run.log').read_text().splitlines() if line.startswith('[FRONTEND]')}
    # Batch13 emits an independent structured frame snapshot. Backend stdout
    # fragments may interleave with a frontend line; prefer complete CSV values
    # when available, without changing legacy runs or weakening stage checks.
    if (folder/'dynamic_frontend.csv').exists():
        front={r['frame_id']:{'keyframe':r['keyframe_created'],'pose_inliers':r.get('pose_inliers') or r['fine_inliers']}
               for r in records(folder,'dynamic_frontend.csv') if r['row_type']=='FRAME'}
    assert len(front)==expected
    assert all(front[r['frame_id']]['keyframe']==r['created'] for r in summaries)
    for row in summaries:
        frame=row['frame_id']
        if frame in fine:
            assert front[frame]['pose_inliers']==row['final_inliers']==fine[frame]['fine_inliers']
    if require_map_admission:
        audit=records(folder,'map_admission_final_audit.csv')[0]
        assert audit['forbidden_observations']==audit['unaligned_keyframes']=='0'
    else:
        assert not list(folder.glob('map_admission_*.csv'))
    b4=records(folder,'b4_state_summary_*.csv');seen=set();previous={}
    for row in b4:
        key=(row['frame_id'],row['track_id']);assert key not in seen;seen.add(key)
        prior=previous.get(row['track_id'])
        if prior and row['transition_reason']=='UNRELIABLE_EVIDENCE_FREEZE':
            assert all(row[k]==prior[k] for k in ('current_state','static_count','dynamic_count','recovery_count'))
        previous[row['track_id']]=row
    return dict(passed=True,frames=expected,regular_keyframes_enqueued=regular,bootstrap_keyframes=bootstrap,
        decision_frames=sum(r['need_evaluated']=='1' for r in summaries),
        coarse_vs_final_different_frames=sum(r['need_evaluated']=='1' and r['coarse_inliers']!=r['final_inliers'] for r in summaries),
        unsafe_regular_keyframes=0,decision_inlier_mismatches=0,final_pose_mismatches=0,
        blocked_frame_reasons=dict(Counter(r['reason'] for r in summaries if r['gate_allowed']=='0')),
        coarse_fallback_frames=len(fallback),fine_failure_frames=sum(r['fine_fallback']=='1' for r in fine.values()),
        forbidden_map_observations=0 if require_map_admission else None,
        map_admission_checked=require_map_admission,b4_single_update_and_unreliable_freeze=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('batch',type=Path);args=p.parse_args()
    scope=audit_scope();reports=[]
    for run in json.loads((args.batch/'summary.json').read_text()):
        if run['mode'] in ('kf3','fine3'):
            reports.append(dict(phase=run['phase'],mode=run['mode'],sequence=run['sequence'],directory=run['directory'],
                **audit_run(Path(run['directory']),run['expected_frames'],run['mode']=='kf3')))
    output=args.batch/'keyframe_order_summary.json'
    with output.open('x') as stream:json.dump(dict(scope=scope,runs=reports),stream,indent=2)
    with (args.batch/'keyframe_order_summary.csv').open('x',newline='') as stream:
        keys=('phase','mode','sequence','frames','regular_keyframes_enqueued','bootstrap_keyframes',
              'unsafe_regular_keyframes','decision_inlier_mismatches','decision_frames',
              'coarse_vs_final_different_frames','coarse_fallback_frames','fine_failure_frames')
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader()
        for report in reports:writer.writerow({k:report[k] for k in keys})
    print(json.dumps(dict(scope=scope,runs=reports),indent=2))

if __name__=='__main__':main()
