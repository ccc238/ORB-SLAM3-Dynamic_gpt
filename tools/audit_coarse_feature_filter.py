#!/usr/bin/env python3
"""Batch-9 coarse action/rollback audit; no trajectory or original result is changed."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT=Path(__file__).resolve().parents[1]

def records(directory,pattern):
    paths=list(directory.glob(pattern))
    assert len(paths)==1,(directory,pattern,paths)
    with paths[0].open() as stream:return list(csv.DictReader(stream))

def function(source,name):
    match=re.search(r'^.*Tracking::'+name+r'\([^;]*?\)\s*\{',source,re.M)
    assert match,name
    tail=source[match.start():]
    next_function=re.search(r'\n(?:void|bool|Sophus::SE3f|int|float|double) Tracking::',tail[1:])
    return tail[:next_function.start()+1] if next_function else tail

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_directory',type=Path)
    args=parser.parse_args()
    with tarfile.open(ROOT/'results/frontend_stage_validation/batch9_coarse_filter_build_01/pre_batch9_sources.tar.gz') as archive:
        before=archive.extractfile('src/Tracking.cc').read().decode()
    now=(ROOT/'src/Tracking.cc').read_text()
    unchanged=('Track','Relocalization','TrackLocalMap','NeedNewKeyFrame','CreateNewKeyFrame',
               'CreateMapInAtlas','MonocularInitialization','StereoInitialization')
    for name in unchanged:assert function(before,name)==function(now,name),name+' changed'
    for name in ('TrackReferenceKeyFrame','TrackWithMotionModel'):
        source=function(now,name)
        assert source.count('OptimizeNormalInitialPose(')==1
        assert 'OptimizeInitialPoseWithDiagnostics()' not in source
    frozen=json.loads((ROOT/'freezes/pre_two_stage_20261022_104737/source_sha256.json').read_text())
    for name in ('src/Optimizer.cc','src/ORBmatcher.cc','src/LocalMapping.cc',
                 'src/MapPoint.cc','src/KeyFrame.cc','src/ORBextractor.cc','src/B3SemanticGeometryFusion.cc'):
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==frozen[name],name
    with tarfile.open(ROOT/'results/frontend_stage_validation/batch8_explicit_b2_20261022_203400/final_implemented_sources.tar.gz') as archive:
        for name in ('include/B2MotionEvidence.h','src/B2MotionEvidence.cc'):
            assert archive.extractfile(name).read()==(ROOT/name).read_bytes(),name
    reports=[]
    for run in json.loads((args.batch_directory/'summary.json').read_text()):
        assert run['passed'],run
        directory=Path(run['directory'])
        quality={int(r['frame_id']):r for r in records(directory,'coarse_pose_quality_*.csv')}
        policy=records(directory,'feature_policy_summary_*.csv')
        assert len(quality)==len(policy)==run['expected_frames']
        log=(directory/'run.log').read_text()
        assert log.count('[B2 READONLY]')==run['expected_frames']
        assert 'state_unchanged=1' in log or 'unchanged=1' in log
        b3=records(directory,'b3_fusion_summary_*.csv')
        b4=records(directory,'b4_state_summary_*.csv')
        updates=Counter((r['frame_id'],r['track_id']) for r in b4)
        assert max(updates.values(),default=0)<=1,'B4 update/log repeated'
        previous={}
        for r in b4:
            if int(r['instance_id'])<0:continue
            prior=previous.get(r['track_id'])
            assert int(r['track_age'])==(int(prior['track_age'])+1 if prior else 1)
            if r['b3_reliable']=='0':
                assert r['previous_state']==r['current_state']
                if prior:assert all(r[key]==prior[key] for key in ('static_count','dynamic_count','recovery_count'))
            previous[r['track_id']]=r
        assert all(r['applied_action']=='NONE_EVIDENCE_ONLY' for r in b3)
        assert all(r['applied_policy']=='NONE_EVIDENCE_ONLY' for r in b4)
        fallback_frames=set()
        filters=[]
        if run['mode']=='3':
            filters=records(directory,'coarse_feature_filter_*.csv')
            for r in filters:
                before_count=int(r['coarse_matches_before']);rejected=int(r['coarse_policy_rejected'])
                after=int(r['coarse_matches_after']);assert before_count-rejected==after
                if r['coarse_policy_applied']=='1':
                    assert r['coarse_fallback']=='0' and r['coarse_fallback_reason']=='NA'
                    assert int(r['coarse_inliers'])>=30 and after>=30
                if r['coarse_fallback']=='1':
                    frame=int(r['frame_id']);fallback_frames.add(frame)
                    assert int(r['restored_inliers'])>=0
                    assert quality[frame]['reliable']=='0'
                    assert quality[frame]['fallback_reason']==r['coarse_fallback_reason']
            assert any(int(r['coarse_policy_rejected'])>0 for r in filters)
        else:
            assert not list(directory.glob('coarse_feature_filter_*.csv'))
        for r in policy:
            n=int(r['N']);excluded=int(r['actual_excluded_count'])
            assert 0<=excluded<=n
            assert int(r['evidence_only'])==int(run['mode']!='3')
            assert int(r['policy_applied'])==int(excluded>0)
            assert int(r['coarse_allowed_count'])==int(r['background_count'])+int(r['previous_S_count'])
            if excluded:assert excluded==n-int(r['coarse_allowed_count'])
            if int(r['frame_id']) in fallback_frames:assert excluded==0
        for r in b3:
            if int(r['frame_id']) in fallback_frames:assert r['fusion_reliable']=='0'
        for r in b4:
            if int(r['frame_id']) in fallback_frames and int(r['instance_id'])>=0:
                assert r['previous_state']==r['current_state']
                assert 'UNRELIABLE_EVIDENCE_FREEZE' in r['transition_reason']
        events={}
        for line in log.splitlines():
            if line.startswith('[TWO STAGE]'):
                row=dict(re.findall(r'(\w+)=([^ ]+)',line))
                events.setdefault(int(row['frame']),[]).append(row['event'])
        assert len(events)==run['expected_frames']
        for event in events.values():
            assert event.count('evidence_complete')==event.count('frame_complete')==1
            if 'coarse_boundary' in event:
                assert event.index('coarse_boundary')<event.index('evidence_complete')<event.index('fine_complete')
        reports.append(dict(directory=str(directory),mode=run['mode'],phase=run['phase'],
            sequence=run['sequence'],passed=True,filter_attempts=len(filters),
            applied_attempts=sum(r['coarse_policy_applied']=='1' for r in filters),
            matches_before=sum(int(r['coarse_matches_before']) for r in filters),
            rejected_matches=sum(int(r['coarse_policy_rejected']) for r in filters),
            matches_after=sum(int(r['coarse_matches_after']) for r in filters),
            applied_rejected_matches=sum(int(r['coarse_policy_rejected']) for r in filters
                if r['coarse_policy_applied']=='1'),
            branch_counts=dict(Counter(r['branch'] for r in filters)),
            fallback_frames=len(fallback_frames),fallback_reasons=dict(Counter(
                r['coarse_fallback_reason'] for r in filters if r['coarse_fallback']=='1')),
            quality_unreliable_frames=sum(r['reliable']=='0' for r in quality.values())))
    output=args.batch_directory/'coarse_filter_audit.json'
    with output.open('x') as stream:json.dump(dict(passed=True,unchanged_tracking_functions=unchanged,runs=reports),stream,indent=2)
    print(json.dumps(reports,indent=2))

if __name__=='__main__':main()
