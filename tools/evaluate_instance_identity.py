#!/usr/bin/env python3
"""Independent manual identity/motion labels; never use SLAM states as truth."""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--diagnostics', required=True, type=Path, help='dynamic_frontend.csv')
    p.add_argument('--annotations', type=Path, help='human-completed template')
    p.add_argument('--output', required=True, type=Path, help='new directory')
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=False)
    rows = list(csv.DictReader(a.diagnostics.open()))
    instances = {(r['frame_id'],r['track_id']):r for r in rows
                 if r['row_type']=='INSTANCE' and int(r['instance_id'])>=0}
    fields = ['frame_id','timestamp','track_id','gt_person_id','motion_label','visible',
              'association_correct','notes']
    with (a.output/'manual_annotation_template.csv').open('w') as f:
        w = csv.DictWriter(f,fields);w.writeheader()
        for r in instances.values():
            w.writerow({k:r[k] for k in ['frame_id','timestamp','track_id']})
    if not a.annotations:
        print('Template only: fill independent gt_person_id, STATIC/MOVING/AMBIGUOUS/OCCLUDED, visible=1/0, association_correct=1/0.');return
    labels = list(csv.DictReader(a.annotations.open()))
    groups = defaultdict(list); truth = {}; seen=set(); seen_gt=set()
    for label in labels:
        key = label['frame_id'], label['track_id']
        if key in seen: raise ValueError('Duplicate annotation '+str(key))
        seen.add(key)
        if key not in instances: raise ValueError('Annotation not in this run: '+str(key))
        if label['visible']!='1' or not label['gt_person_id']: continue
        if label['association_correct'] not in ('0','1') or label['motion_label'] not in ('STATIC','MOVING','AMBIGUOUS','OCCLUDED'):
            raise ValueError('Visible GT rows need explicit human association_correct and motion_label')
        gt_key=label['frame_id'],label['gt_person_id']
        if gt_key in seen_gt: raise ValueError('One GT person assigned to multiple detections in a frame')
        seen_gt.add(gt_key)
        r=instances[key];groups[label['gt_person_id']].append((r,label));truth[key]=label
    if not truth: raise ValueError('No independent visible identity/motion annotations; refusing zero-valued accuracy claims')
    metrics = dict(visible_instance_rows=len(instances), annotated_visible_rows=len(truth),
                   id_switches=0, fragments=0, wrong_S_inheritance=0,
                   wrong_S_coarse_optimizer_feature_uses=0, wrong_S_fine_optimizer_feature_uses=0)
    recoveries=[]
    for gt,observations in groups.items():
        observations.sort(key=lambda x:int(x[0]['frame_id']))
        previous=None; pending={}
        for r,label in observations:
            if previous:
                switched=r['track_id']!=previous['track_id']
                metrics['id_switches']+=switched
                metrics['fragments']+=switched or int(r['frame_id'])>int(previous['frame_id'])+1
            previous=r
            wrong=(label['association_correct']=='0' and r['allow_state_inheritance']=='1'
                   and r['previous_b4_state']=='S')
            metrics['wrong_S_inheritance']+=wrong
            if wrong:
                metrics['wrong_S_coarse_optimizer_feature_uses']+=int(r['coarse_S_feature_uses'])
                metrics['wrong_S_fine_optimizer_feature_uses']+=int(r['fine_S_feature_uses'])
            if r['association_status']=='REACQUIRED': pending[r['track_id']]=r
            if r['current_b4_state']=='S' and r['track_id'] in pending:
                start=pending.pop(r['track_id'])
                recoveries.append(dict(gt_person_id=gt,track_id=r['track_id'],
                    return_frame=start['frame_id'],static_frame=r['frame_id'],
                    seconds=float(r['timestamp'])-float(start['timestamp'])))
        for track,start in pending.items():
            recoveries.append(dict(gt_person_id=gt,track_id=track,return_frame=start['frame_id'],
                                   static_frame='',seconds=''))
    for motion,name in [('STATIC','static_retention'),('MOVING','dynamic_leakage')]:
        available=used=0
        for r in rows:
            key=r['frame_id'],r['track_id'];label=truth.get(key)
            if not label or label['motion_label']!=motion:continue
            if r['row_type']=='FEATURE_GROUP' and r['semantic_region']=='PERSON_INTERIOR':
                available+=int(r['feature_count'])
            elif r['row_type']=='INSTANCE': used+=int(r['fine_optimizer_feature_uses'])
        metrics[name+'_available_interior_features']=available
        metrics[name+'_fine_optimizer_feature_uses']=used
        metrics[name+'_use_ratio']=used/available if available else None
    metrics['limitations']='Metrics apply only to annotated detector-visible rows; gaps/sparse labels affect fragments. Optimizer feature uses cover normal initial-pose branches and TrackLocalMap, including fallback/retries, not distinct points; Relocalization is not instrumented. Missing detections require separate GT annotations; no precision/ATE conclusion.'
    (a.output/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    with (a.output/'reacquisition_static_recovery.csv').open('w') as f:
        w=csv.DictWriter(f,['gt_person_id','track_id','return_frame','static_frame','seconds']);w.writeheader();w.writerows(recoveries)
    print(json.dumps(metrics,indent=2))


if __name__=='__main__':main()
