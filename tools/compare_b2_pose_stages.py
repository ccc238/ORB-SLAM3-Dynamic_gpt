#!/usr/bin/env python3
"""Audit explicit B2 input and compare final/coarse evidence without retuning."""
import argparse
from collections import Counter,defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import struct
import tarfile

ROOT=Path(__file__).resolve().parents[1]
STATES=('GEOM_STATIC_CANDIDATE','GEOM_MOVING_CANDIDATE','GEOM_UNCERTAIN','GEOM_UNRELIABLE')
METRICS=('median_r3d','median_normalized_r3d','high_residual_ratio')
TRANSITIONS=('P_TO_S','P_TO_D','S_TO_P','D_TO_P')

def read_rows(directory,pattern):
    paths=list(directory.glob(pattern))
    assert len(paths)==1,(directory,pattern)
    with paths[0].open() as stream:return list(csv.DictReader(stream))

def yaml_values(path):
    return dict(re.findall(r'^([^#%\s][^:\n]*):\s*(.*?)\s*$',path.read_text(),re.M))

def f32(x):return struct.unpack('f',struct.pack('f',float(x)))[0]

def quantile(values,p):
    values=sorted(values);x=(len(values)-1)*p;i=int(x);j=min(i+1,len(values)-1)
    return values[i]+(values[j]-values[i])*(x-i)

def distribution(values):
    values=[x for x in values if math.isfinite(x) and x>=0]
    result={'count':len(values)}
    for key,p in (('p05',.05),('p25',.25),('p50',.5),('p75',.75),('p90',.9),('p95',.95),('p99',.99)):
        result[key]=quantile(values,p) if values else ''
    result.update(mean=statistics.mean(values) if values else '',
                  std=statistics.pstdev(values) if values else '',
                  max=max(values) if values else '')
    return result

def output_csv(path,records):
    assert records
    with path.open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)

def instance_key(row):return (int(row['frame_id']),float(row['timestamp']),int(row['instance_id']))

def export_reasons(runs,destination):
    records=[]
    for run in runs:
        counts=Counter(r['decision_reason'] for r in read_rows(Path(run['directory']),
                       'b2_evidence_*/b2_instance_evidence.csv') if r['geometry_state']=='GEOM_UNRELIABLE')
        for reason,count in sorted(counts.items()):
            records.append(dict(phase=run['phase'],sequence=run['sequence'],
                pose_source='coarse' if run['mode']=='2' else 'final',reason=reason,instance_frames=count))
    output_csv(destination/'b2_unreliable_reason_comparison.csv',records)

def source_audit():
    with tarfile.open(ROOT/'results/frontend_stage_validation/batch7_coarse_quality_20261022_173255/final_implemented_sources.tar.gz') as archive:
        detector=archive.extractfile('src/Deep/DynamicDetector.cc').read()
    assert detector==(ROOT/'src/Deep/DynamicDetector.cc').read_bytes(),'YAML threshold parser changed'
    frozen=json.loads((ROOT/'freezes/pre_two_stage_20261022_104737/source_sha256.json').read_text())
    for name in ('src/Optimizer.cc','src/ORBmatcher.cc','src/LocalMapping.cc','src/MapPoint.cc',
                 'src/KeyFrame.cc','src/ORBextractor.cc','src/B3SemanticGeometryFusion.cc'):
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==frozen[name],name
    b2=(ROOT/'src/B2MotionEvidence.cc').read_text()
    geometry=b2[b2.index('void B2MotionEvidence::ComputeGeometryResiduals'):b2.index('void B2MotionEvidence::MatchAssociatedInstances')]
    assert 'current.Tcw * mPreviousFrame.Tcw.inverse()' not in geometry
    assert geometry.count('currentFromPrevious = mCurrentFromPrevious')==2
    tracking=(ROOT/'src/Tracking.cc').read_text()
    track=tracking[tracking.index('void Tracking::Track()'):tracking.index('void Tracking::StereoInitialization()')]
    assert track.index('RunRGBDGeometryEvidence(coarseReliable, "coarse")') < track.index('bOK = TrackLocalMap()')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch_directory',type=Path)
    parser.add_argument('--short-only',action='store_true')
    parser.add_argument('--reason-only',action='store_true',help='Add only a new reason CSV to an existing comparison')
    args=parser.parse_args()
    source_audit()
    runs=json.loads((args.batch_directory/'summary.json').read_text())
    if args.short_only:runs=[r for r in runs if r['phase']=='short100']
    assert len(runs)==(4 if args.short_only else 8)
    destination=args.batch_directory/('comparison_short' if args.short_only else 'comparison')
    if args.reason_only:
        assert destination.is_dir()
        export_reasons(runs,destination)
        print('Reason comparison saved: '+str(destination))
        return
    destination.mkdir(exist_ok=False)
    categories=[];transitions=[];quantiles=[];background=[];point_quantiles=[];paired=[];audits=[]
    grouped={}
    all_b2_yaml=None
    common_config=None
    common_runtime_model=None
    for run in runs:
        assert run['passed']
        directory=Path(run['directory'])
        source='coarse' if run['mode']=='2' else 'final'
        assert run['mode'] in ('2','final')
        values=yaml_values(directory/'effective_config.yaml')
        normalized_config={k:v for k,v in values.items() if k!='TwoStage.UseCoarsePoseForB2'}
        if common_config is None:common_config=normalized_config
        assert normalized_config==common_config,'Camera/ORB/other parameters differ beyond pose-source switch'
        metadata_paths=list(directory.glob('b2_evidence_*/b2_metadata.txt'));assert len(metadata_paths)==1
        provenance=dict(line.split('=',1) for line in metadata_paths[0].read_text().splitlines() if '=' in line)
        identity=(run['runtime_sha256'],provenance['yolo_model_sha256'],provenance['git_commit'])
        if common_runtime_model is None:common_runtime_model=identity
        assert identity==common_runtime_model,'Runtime/model/commit differs between groups'
        assert provenance['yaml_sha256']==run['config_sha256']==hashlib.sha256((directory/'effective_config.yaml').read_bytes()).hexdigest()
        b2yaml={k:v for k,v in values.items() if k.startswith('B2.')}
        if all_b2_yaml is None:all_b2_yaml=b2yaml
        assert b2yaml==all_b2_yaml,'Different B2 definitions/thresholds between runs'
        assert values['TwoStage.CoarseFilter.Enable']==values['TwoStage.FinePolicy.Enable']==values['TwoStage.MapAdmission.Enable']=='0'
        assert all(values[k+'.EvidenceOnly']=='1' for k in ('B2','B3','B4'))
        assert values['TwoStage.UseCoarsePoseForB2']==('1' if source=='coarse' else '0')
        rows=read_rows(directory,'b2_evidence_*/b2_instance_evidence.csv')
        by_instance={instance_key(r):r for r in rows};assert len(by_instance)==len(rows)
        grouped[(run['phase'],run['sequence'],source)]=(run,by_instance)
        count=Counter(r['geometry_state'] for r in rows)
        assert set(count)<=set(STATES)
        prefix=dict(phase=run['phase'],sequence=run['sequence'],pose_source=source)
        categories.append(dict(prefix,instance_frames=len(rows),**{s:count[s] for s in STATES},
                               pose_weak_count=sum(r['decision_reason']=='POSE_WEAK' for r in rows)))
        b3=read_rows(directory,'b3_fusion_summary_*.csv')
        assert all(r['applied_action']=='NONE_EVIDENCE_ONLY' for r in b3)
        b4=read_rows(directory,'b4_state_summary_*.csv')
        assert all(r['applied_policy']=='NONE_EVIDENCE_ONLY' for r in b4)
        assert len({(r['frame_id'],r['timestamp'],r['track_id']) for r in b4})==len(b4)
        reasons=Counter(r['transition_reason'] for r in b4)
        visible=[r for r in b4 if int(r['instance_id'])>=0]
        states=Counter(r['current_state'] for r in visible)
        transitions.append(dict(prefix,**{s:reasons[s] for s in TRANSITIONS},
                                S_instance_frames=states['S'],P_instance_frames=states['P'],D_instance_frames=states['D']))
        for cohort in ('metric_available','reliability_gate_passed'):
            eligible=rows if cohort=='metric_available' else [r for r in rows if r['geometry_state']!='GEOM_UNRELIABLE']
            for metric in METRICS:
                quantiles.append(dict(prefix,cohort=cohort,metric=metric,total_instance_frames=len(rows),
                                      **distribution([float(r[metric]) for r in eligible])))
            for metric in ('background_sample_count','background_median_r3d','background_sigma_r3d'):
                background.append(dict(prefix,cohort=cohort,metric=metric,total_instance_frames=len(rows),
                                       **distribution([float(r[metric]) for r in eligible])))
        features=read_rows(directory,'b2_evidence_*/b2_candidate_features.csv')
        for cohort in ('metric_available','normalized_geometry_valid'):
            eligible=features if cohort=='metric_available' else [r for r in features if r['valid']=='1']
            for metric in ('r3d','r3d_normalized'):
                point_quantiles.append(dict(prefix,cohort=cohort,metric=metric,level='individual_candidate_match',
                                            **distribution([float(r[metric]) for r in eligible])))
        # Re-evaluate the SAME six-field three-zone rule using effective float32 thresholds.
        moving=[f32(values['B2.Geometry.'+k]) for k in ('RawMovingThreshold','NormMovingThreshold','HighRatioMovingThreshold')]
        stationary=[f32(values['B2.Geometry.'+k]) for k in ('RawStaticThreshold','NormStaticThreshold','HighRatioStaticThreshold')]
        for r in rows:
            if r['geometry_state']=='GEOM_UNRELIABLE':
                assert r['decision_reason'] not in ('NONE','GEOM_FAILURE_NONE','')
                continue
            data=[float(r[k]) for k in METRICS]
            assert all(x>=0 for x in data)
            expected=STATES[1] if all(x>=y for x,y in zip(data,moving)) else (
                STATES[0] if all(x<=y for x,y in zip(data,stationary)) else STATES[2])
            assert r['geometry_state']==expected,(r,expected)
        log=(directory/'run.log').read_text().splitlines()
        inputs={};readonly={};events=defaultdict(list);ordering={}
        for index,line in enumerate(log):
            if line.startswith(('[B2 INPUT]','[B2 READONLY]','[TWO STAGE]')):
                fields=dict(re.findall(r'(\w+)=([^ ]+)',line));frame=int(fields['frame'])
                if line.startswith('[B2 INPUT]'):
                    assert frame not in inputs;inputs[frame]=fields;ordering.setdefault(frame,{})['input']=index
                elif line.startswith('[B2 READONLY]'):
                    assert frame not in readonly;readonly[frame]=fields
                else:events[frame].append(fields);ordering.setdefault(frame,{})[fields['event']]=index
        assert len(inputs)==len(readonly)==run['expected_frames']
        for frame,r in inputs.items():
            assert r['pose_source']==source and r['explicit_relative']=='1'
            assert int(r['previous_frame'])==frame-1
            expected_pair=(frame>0 and r['current_pose_reliable']=='1' and
                           inputs[frame-1]['current_pose_reliable']=='1')
            assert r['pair_pose_reliable']==str(int(expected_pair)),'Wrong previous pose/reliability history'
            assert all(readonly[frame][k]=='1' for k in ('map_points_unchanged','frame_pose_unchanged','descriptors_unchanged'))
            assert readonly[frame]['applied_action']=='NONE_EVIDENCE_ONLY'
            if source=='coarse' and 'fine_complete' in ordering[frame]:
                assert ordering[frame]['input']<ordering[frame]['evidence_complete']<ordering[frame]['fine_complete']
        if source=='coarse':
            quality=read_rows(directory,'coarse_pose_quality_*.csv')
            assert len(quality)==run['expected_frames']
            assert all(r['reliable']==inputs[int(r['frame_id'])]['current_pose_reliable'] for r in quality)
        for r in rows:
            if inputs[int(r['frame_id'])]['pair_pose_reliable']=='0':
                assert r['pose_valid']=='0' and r['geometry_state']=='GEOM_UNRELIABLE' and r['decision_reason']=='POSE_WEAK'
        audits.append(dict(prefix,passed=True,frames=run['expected_frames'],
                           immutable_frame_checks=len(readonly),thresholds_unchanged=True))
    for phase,sequence in sorted({(r['phase'],r['sequence']) for r in runs}):
        fr,final=grouped[(phase,sequence,'final')];cr,coarse=grouped[(phase,sequence,'coarse')]
        assert final.keys()==coarse.keys(),'YOLO instance identity/order changed across modes'
        for key in sorted(final):
            a,b=final[key],coarse[key]
            record=dict(phase=phase,sequence=sequence,frame_id=key[0],timestamp=key[1],instance_id=key[2],
                        final_track_id=a['track_id'],coarse_track_id=b['track_id'],
                        final_state=a['geometry_state'],coarse_state=b['geometry_state'])
            for metric in METRICS:
                record['final_'+metric]=a[metric];record['coarse_'+metric]=b[metric]
                x,y=float(a[metric]),float(b[metric])
                record['delta_'+metric]=y-x if x>=0 and y>=0 else ''
            paired.append(record)
        common=[key for key in final if final[key]['geometry_state']!='GEOM_UNRELIABLE' and coarse[key]['geometry_state']!='GEOM_UNRELIABLE']
        for source,records in (('final',final),('coarse',coarse)):
            for metric in METRICS:
                quantiles.append(dict(phase=phase,sequence=sequence,pose_source=source,cohort='paired_gate_passed',
                                      metric=metric,total_instance_frames=len(records),
                                      **distribution([float(records[k][metric]) for k in common])))
        # Same input/model/ORB, independent backend scheduling: compare extraction, not exact trajectories.
        def extraction(run):
            lines=(Path(run['directory'])/'run.log').read_text().splitlines()
            return [tuple(re.search(k+r'=(\d+)',line)[1] for k in ('raw_fast','final_orb','b1_mask_rejected')) for line in lines if line.startswith('[DYNAMIC FRONTEND]')]
        assert extraction(fr)==extraction(cr),'Main ORB extraction changed across stages'
    output_csv(destination/'b2_category_comparison.csv',categories)
    output_csv(destination/'b4_transition_comparison.csv',transitions)
    output_csv(destination/'b2_metric_quantiles_comparison.csv',quantiles)
    output_csv(destination/'b2_background_reference_comparison.csv',background)
    output_csv(destination/'b2_point_residual_quantiles_comparison.csv',point_quantiles)
    output_csv(destination/'paired_instance_metrics.csv',paired)
    export_reasons(runs,destination)
    metadata=dict(formulas={
        'back_project':'[(u-cx)*z/fx, (v-cy)*z/fy, z], z in meters',
        'T_current_from_previous':'Tcw_current * inverse(Tcw_previous); both snapshots from the same pose stage',
        'raw_r3d_i':'norm(P_current_i - T_current_from_previous * P_previous_i), meters',
        'R':'median(raw_r3d_i) over valid geometry matches',
        'bg_median':'median(background raw_r3d), depth-bin reference or current-frame global fallback',
        'bg_sigma':'max(1.4826 * median(abs(bg_raw - bg_median)), SigmaFloorMeters)',
        'e_i':'max(0, raw_r3d_i - bg_median) / bg_sigma',
        'background_selection':'floor(current_depth_m/DepthBinMeters); use depth bin only if MinSamplesPerBin met, else current-frame global; require MinBackgroundSamples',
        'E':'median(e_i)',
        'P':'count(e_i >= B2.Geometry.Decision.MovingThreshold) / valid_geometry_count',
        'moving':'R >= RawMovingThreshold AND E >= NormMovingThreshold AND P >= HighRatioMovingThreshold',
        'static':'R <= RawStaticThreshold AND E <= NormStaticThreshold AND P <= HighRatioStaticThreshold',
        'r2d':'recorded only; not a motion decision threshold'},
        actual_yaml_b2=all_b2_yaml,runtime_float32_thresholds={k:f32(v) for k,v in all_b2_yaml.items() if k in (
            'B2.Geometry.RawMovingThreshold','B2.Geometry.NormMovingThreshold','B2.Geometry.HighRatioMovingThreshold',
            'B2.Geometry.RawStaticThreshold','B2.Geometry.NormStaticThreshold','B2.Geometry.HighRatioStaticThreshold',
            'B2.Geometry.Decision.MovingThreshold')},
        cohorts={'metric_available':'nonnegative finite metric, including unreliable frames if a metric exists',
                 'reliability_gate_passed':'B2 static/moving/uncertain, not B3 fusion reliability',
                 'paired_gate_passed':'same frame/timestamp/instance passed B2 gates in BOTH pose-stage runs'},
        common_runtime_sha256=common_runtime_model[0],common_model_sha256=common_runtime_model[1],
        common_git_commit=common_runtime_model[2],
        limitations='Independent runs include asynchronous mapping variation; no motion ground truth, no automatic retuning.',
        audits=audits,passed=True)
    with (destination/'comparison_metadata.json').open('x') as stream:json.dump(metadata,stream,indent=2);stream.write('\n')
    # Reuse existing once-only/freeze and policy audits on separate four-run manifests.
    for source,mode in (('coarse','2'),('final','final')):
        if args.short_only:continue
        folder=destination/(source+'_audits');folder.mkdir()
        (folder/'summary.json').write_text(json.dumps([r for r in runs if r['mode']==mode],indent=2)+'\n')
    print(json.dumps(dict(passed=True,directory=str(destination),categories=categories,transitions=transitions),indent=2))

if __name__=='__main__':main()
