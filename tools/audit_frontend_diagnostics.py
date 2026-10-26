#!/usr/bin/env python3
"""Batch13: unified CSV scopes, same-frame joins, counters, times and frozen algorithms."""
import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import tarfile
from audit_keyframe_order import records, function, audit_run

ROOT=Path(__file__).resolve().parents[1]

def source_audit():
    freeze=ROOT/'results/frontend_stage_validation/batch13_diagnostics_build_01/pre_batch13_sources.tar.gz'
    frozen=['src/B2MotionEvidence.cc','src/B3SemanticGeometryFusion.cc','src/B4TemporalStateMachine.cc',
            'src/Deep/InstanceTrackManager.cc','src/ORBextractor.cc','src/ORBmatcher.cc','src/Optimizer.cc',
            'src/LocalMapping.cc','src/LoopClosing.cc','src/MapAdmission.cc','src/FrameSemanticAnnotator.cc',
            'src/FeaturePolicyEvaluator.cc','src/FineFeaturePolicy.cc','src/KeyFrame.cc',
            'include/KeyFrameDecisionGuard.h','include/FineFeaturePolicy.h','include/CoarseFeatureFilter.h']
    with tarfile.open(freeze) as archive:
        for name in frozen: assert (ROOT/name).read_bytes()==archive.extractfile(name).read(),name
        before=archive.extractfile('src/Tracking.cc').read().decode()
        after=(ROOT/'src/Tracking.cc').read_text()
        unchanged=['TrackWithMotionModel','TrackReferenceKeyFrame','Relocalization','NeedNewKeyFrame',
                   'EvaluateOriginalKeyFrameNeed','CreateNewKeyFrame','CurrentKeyFrameGate','FinalizeCoarsePoseQuality',
                   'RunB3SemanticGeometryFusion','RunB4TemporalStateMachine','UpdateDynamicLogs']
        for name in unchanged: assert function(before,name)==function(after,name),name
        for name,line in [('TrackLocalMap','    DiagnosticTimer fineTimer(mbRGBDEvidenceReady && mbCollectStageDiagnostics ? &mDiagnosticStages.fineMs : nullptr);\n')]:
            assert function(before,name)==function(after,name).replace(line,''),name
        track=function(after,'Track').replace('        DiagnosticTimer coarseTimer(mbRGBDEvidenceReady && mbCollectStageDiagnostics ? &mDiagnosticStages.coarseMs : nullptr);\n','').replace('        coarseTimer.Stop();\n','')
        assert track==function(before,'Track'),'Tracking control flow changed'
        collector=(ROOT/'src/TrackingDiagnostics.cc').read_text()
        assert 'view.stateActionOverlayEnabled=cfg.diagnosticShowOverlay;' in collector
        assert 'view.stateActionOverlayEnabled=cfg.diagnosticShowOverlay ||' not in collector
        assert 'FrameDrawer::DrawStateActionOverlay(rendered,mCurrentFrame.mvKeys,' in after
        for path in (ROOT/'configs/frontend_modes').rglob('*.yaml'):
            rel=str(path.relative_to(ROOT))
            if rel in archive.getnames(): assert path.read_bytes()==archive.extractfile(rel).read(),rel
    return {'passed':True,'frozen_files':frozen,'unchanged_tracking_functions':unchanged,
            'tracking_control_flow_unchanged_except_read_only_timers':True}

def audit(folder):
    stats=json.loads((folder/'validation.json').read_text()); expected=stats['expected_frames']
    rows=records(folder,'dynamic_frontend.csv')
    frames={r['frame_id']:r for r in rows if r['row_type']=='FRAME'}
    assert len(frames)==expected
    assert len([r for r in rows if r['row_type']=='FRAME'])==expected
    groups=defaultdict(list)
    for r in rows: groups[r['frame_id']].append(r)
    old_b3={(r['frame_id'],r['instance_id'],r['track_id']):r for r in records(folder,'b3_fusion_summary_*.csv')}
    old_b4={(r['frame_id'],r['instance_id'],r['track_id']):r for r in records(folder,'b4_state_summary_*.csv')}
    coarse=defaultdict(list)
    for r in records(folder,'coarse_feature_filter_*.csv'):coarse[r['frame_id']].append(r)
    fine={r['frame_id']:r for r in records(folder,'fine_feature_policy_*.csv') if r['phase']=='pre_pose_optimization'}
    fine_final={r['frame_id']:r for r in records(folder,'fine_feature_policy_*.csv') if r['phase']=='result'}
    map_counts=defaultdict(Counter)
    mapkeys=['map_candidates_total','map_reject_P','map_reject_D','map_reject_boundary','map_reject_unknown','map_points_created']
    for r in records(folder,'map_admission_[0-9]*.csv'):
        for key in mapkeys:map_counts[r['frame_id']][key]+=int(r[key])
    joins=0
    for fid,f in frames.items():
        group=[r for r in groups[fid] if r['row_type']=='FEATURE_GROUP']
        assert sum(int(r['feature_count']) for r in group)==int(f['final_orb'])
        assert f['map_counts_pending']=='0' and f['map_counts_scope']=='SOURCE_KEYFRAME_FRAME'
        for key in mapkeys:assert int(f[key])==map_counts[fid][key],(fid,key)
        for key in ['yolo_ms','orb_ms','b2_ms','b3_ms','b4_ms']:assert float(f[key])>=0,(fid,key)
        if fid in fine:
            assert float(f['coarse_ms'])>=0 and float(f['fine_ms'])>=0
            for key in ['fine_matches_before','fine_reject_P','fine_reject_D','fine_reject_boundary','fine_reject_unknown','fine_matches_after']:
                assert f[key]==fine[fid][key],(fid,key)
            assert f['fine_inliers']==fine_final[fid]['fine_inliers']
        if coarse[fid]:
            assert int(f['coarse_attempts'])==len(coarse[fid])
            for new,old in [('coarse_matches_before','coarse_matches_before'),('coarse_rejected','coarse_policy_rejected'),
                            ('coarse_matches_after','coarse_matches_after'),('coarse_inliers','coarse_inliers')]:
                assert f[new]==coarse[fid][-1][old],(fid,new)
        for row in groups[fid]:
            if row['row_type']!='FRAME':
                assert not row['coarse_matches_before'] and not row['fine_inliers'] and not row['map_points_created'],'Duplicated frame counters'
            if row['row_type']=='FEATURE_GROUP':
                region=row['semantic_region'];prev=row['previous_b4_state'];curr=row['current_b4_state']
                owner=int(row['instance_id'])>=0 and int(row['track_id'])>=0
                allowed=lambda state:region=='BACKGROUND' or (region=='PERSON_INTERIOR' and owner and state=='S')
                assert int(row['allow_coarse'])==allowed(prev)
                assert int(row['allow_fine'])==int(row['allow_map'])==allowed(curr)
            if row['row_type']!='INSTANCE':continue
            key=(fid,row['instance_id'],row['track_id'])
            b4=old_b4[key]
            for new,old in [('previous_b4_state','previous_state'),('current_b4_state','current_state'),
                            ('Ns','static_count'),('Nd','dynamic_count'),('Nr','recovery_count'),
                            ('missed_frames','missed_frames'),('b4_transition_reason','transition_reason')]:
                assert row[new]==b4[old],(key,new)
            if row['semantic_region']=='NOT_VISIBLE':
                assert not row['b2_raw_r3d'] and row['fine_action']=='NOT_VISIBLE'
                continue
            b3=old_b3[key];joins+=1
            for new,old in [('b2_raw_r3d','raw_r3d_median'),('b2_norm_r3d','norm_r3d_median'),('b2_high_ratio','high_ratio')]:
                assert abs(float(row[new])-float(b3[old]))<1e-6,(key,new)
            for new,old in [('b2_reliable','geometry_reliable'),('b2_reason','geometry_reason'),
                            ('b3_fusion_category','fusion_category'),('b3_reliable','fusion_reliable'),('b3_reason','fusion_reason')]:
                assert row[new]==b3[old],(key,new)
    assert joins==len(old_b3)
    online=records(folder,'dynamic_frontend_online.csv')
    assert len(online)==len(rows)
    assert all(r['map_counts_pending']=='1' and not r['map_points_created'] for r in online if r['row_type']=='FRAME')
    pngs=list((folder/'dynamic_frontend_overlays').glob('*.png'))
    assert len(pngs)==(expected-1)//100+1 and all(p.stat().st_size>1000 for p in pngs)
    keyframes=audit_run(folder,expected)
    assert keyframes['passed']
    return {'directory':str(folder),'passed':True,'frames':expected,'instance_joins':joins,
            'row_types':dict(Counter(r['row_type'] for r in rows)),'png_count':len(pngs),
            'map_points_created':sum(int(f['map_points_created']) for f in frames.values()),
            'map_reject_P':sum(int(f['map_reject_P']) for f in frames.values()),
            'map_reject_D':sum(int(f['map_reject_D']) for f in frames.values()),
            'fine_reject_P':sum(int(f['fine_reject_P'] or 0) for f in frames.values()),
            'fine_reject_D':sum(int(f['fine_reject_D'] or 0) for f in frames.values()),
            'timing_mean_ms':{key:sum(values)/len(values) for key in ['yolo_ms','orb_ms','coarse_ms','b2_ms','b3_ms','b4_ms','fine_ms']
                              for values in [[float(r[key]) for r in frames.values() if float(r[key])>=0]]},
            'keyframe_order':keyframes}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',nargs='*',type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();assert not args.output.exists(),'Refusing overwrite'
    result={'source_audit':source_audit(),'runs':[audit(folder) for folder in args.runs]}
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
