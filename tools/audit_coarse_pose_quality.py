#!/usr/bin/env python3
"""Batch-7 quality gates, fallback freezing, ordering and original-path audit."""
import argparse
from collections import Counter,defaultdict
import csv
import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_directory',type=Path)
    parser.add_argument('--short-only',action='store_true')
    args=parser.parse_args()
    with tarfile.open(ROOT/'results/frontend_stage_validation/batch6_coarse_pose_20261022_171118/final_implemented_sources.tar.gz') as archive:
        previous=archive.extractfile('src/Tracking.cc').read().decode()
    current=(ROOT/'src/Tracking.cc').read_text()
    def function(source,name):
        m=re.search(r'^.*Tracking::'+name+r'\([^;]*?\)\s*\{',source,re.M);assert m
        tail=source[m.start():]
        n=re.search(r'\n(?:void|bool|Sophus::SE3f|int|float|double) Tracking::',tail[1:])
        return tail[:n.start()+1] if n else tail
    for name in ('TrackLocalMap','NeedNewKeyFrame','StereoInitialization','MonocularInitialization'):
        assert function(previous,name)==function(current,name),name
    for name in ('TrackReferenceKeyFrame','TrackWithMotionModel','Relocalization'):
        new=function(current,name).replace('OptimizeInitialPoseWithDiagnostics()','Optimizer::PoseOptimization(&mCurrentFrame)')
        new=new.replace('    if (mbRGBDEvidenceReady && mpDynamicDetector->Frontend().useCoarsePoseForB2)\n        mCoarsePoseContext.relocalizing = true;\n','')
        assert function(previous,name)==new,'Original optimizer call sequence changed: '+name
    marker='        // All original initial-tracking branches converge here.'
    a=previous[previous.index('void Tracking::Track()'):previous.index(marker)]
    b=current[current.index('void Tracking::Track()'):current.index(marker)]
    for line in ('                    CoarsePoseQuality coarseQualityMM;\n',
                 '                        coarseQualityMM = mCoarsePoseQuality;\n',
                 '                        mCoarsePoseQuality = coarseQualityMM;\n'):
        b=b.replace(line,'')
    assert a==b,'Initial tracking control flow changed'
    hashes=json.loads((ROOT/'freezes/pre_two_stage_20261022_104737/source_sha256.json').read_text())
    for path in ('src/B2MotionEvidence.cc','src/B3SemanticGeometryFusion.cc','src/LocalMapping.cc',
                 'src/Optimizer.cc','src/ORBmatcher.cc','src/MapPoint.cc','src/ORBextractor.cc'):
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==hashes[path],path
    reports=[]
    runs=json.loads((args.batch_directory/'summary.json').read_text())
    if args.short_only:runs=[run for run in runs if run['phase']=='short100']
    for run in runs:
        assert run['passed']
        directory=Path(run['directory'])
        def rows(pattern):
            paths=list(directory.glob(pattern));assert len(paths)==1
            with paths[0].open() as stream:return list(csv.DictReader(stream))
        quality=rows('coarse_pose_quality_*.csv');assert len(quality)==run['expected_frames']
        fallback={}
        for row in quality:
            frame=int(row['frame_id']);assert frame not in fallback
            minimum=int(row['min_coarse_inliers']);ratio=float(row['min_coarse_inlier_ratio'])
            n=int(row['match_count']);i=int(row['inlier_count'])
            actual_ratio=i/n if n else 0.
            assert abs(float(row['inlier_ratio'])-actual_ratio)<1e-6
            reason=''
            if row['new_map']=='1':reason='NEW_MAP'
            elif row['initialized']=='0':reason='NOT_INITIALIZED'
            elif row['relocalizing']=='1':reason='RELOCALIZATION'
            elif row['just_recovered']=='1':reason='JUST_RECOVERED'
            elif row['initial_success']=='0':reason='COARSE_TRACK_FAILED'
            elif int(row['background_features'])<minimum:reason='INSUFFICIENT_COARSE_FEATURES'
            elif i<minimum:reason='INSUFFICIENT_COARSE_INLIERS'
            elif int(row['background_inliers'])<minimum:reason='INSUFFICIENT_COARSE_FEATURES'
            elif row['finite_pose']=='0' or row['pure_vo']=='1' or i>n or float(row['inlier_ratio'])<ratio:reason='POSE_NOT_RELIABLE'
            assert row['fallback_reason']==reason
            assert row['reliable']==str(int(not reason)) and row['fallback']==str(int(bool(reason)))
            fallback[frame]=bool(reason)
        for row in rows('b2_evidence_*/b2_instance_evidence.csv'):
            if fallback[int(row['frame_id'])]:assert row['pose_valid']=='0'
        for row in rows('b3_fusion_summary_*.csv'):
            if fallback[int(row['frame_id'])]:assert row['fusion_reliable']=='0'
        b4=rows('b4_state_summary_*.csv')
        for row in b4:
            if int(row['instance_id'])>=0 and fallback[int(row['frame_id'])]:
                assert row['previous_state']==row['current_state'] and row['b3_reliable']=='0'
        events=defaultdict(list)
        for line in (directory/'run.log').read_text().splitlines():
            if line.startswith('[TWO STAGE]'):
                r=dict(re.findall(r'(\w+)=([^ ]+)',line));events[int(r['frame'])].append(r)
        assert len(events)==len(quality)
        for frame,sequence in events.items():
            names=[r['event'] for r in sequence]
            assert names.count('evidence_complete')==names.count('frame_complete')==1
            evidence=next(r for r in sequence if r['event']=='evidence_complete')
            assert evidence['pose_reliable']==str(int(not fallback[frame]))
            if 'fine_complete' in names:
                assert names.index('evidence_complete')<names.index('fine_complete')
            if 'keyframe_enqueue' in names:
                assert names.index('fine_complete')<names.index('keyframe_decision')<names.index('keyframe_enqueue')
                assert next(r for r in sequence if r['event']=='keyframe_enqueue')['fine_completed']=='1'
        reports.append(dict(directory=str(directory),passed=True,frames=len(quality),
                            reliable_frames=sum(not x for x in fallback.values()),
                            fallback_reasons=dict(Counter(r['fallback_reason'] for r in quality if r['fallback']=='1')),
                            nontrivial_inlier_ratio_frames=sum(0<float(r['inlier_ratio'])<1 for r in quality)))
    assert len(reports)==(2 if args.short_only else 4)
    name='coarse_quality_short_audit.json' if args.short_only else 'coarse_quality_audit.json'
    with (args.batch_directory/name).open('x') as stream:
        json.dump(dict(passed=True,original_tracking_branches_and_optimizer_calls_preserved=True,runs=reports),stream,indent=2)
        stream.write('\n')
    print(json.dumps(reports,indent=2))


if __name__=='__main__':main()
