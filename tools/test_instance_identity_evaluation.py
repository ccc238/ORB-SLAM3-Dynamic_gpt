#!/usr/bin/env python3
"""Synthetic labels test the evaluation interface, not real identity accuracy."""
import csv
import json
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='identity_eval_') as temporary:
    root=Path(temporary);rows=[];labels=[]
    for f,track in [(0,0),(1,1),(2,1)]:
        state='P' if f==1 else 'S'
        rows.append(dict(row_type='INSTANCE',frame_id=str(f),timestamp=str(f/10),track_id=str(track),
            instance_id='0',allow_state_inheritance='1',previous_b4_state='S' if f<2 else 'P',
            current_b4_state=state,association_status='REACQUIRED' if f==1 else 'CONTINUOUS_CONFIRMED',
            coarse_S_feature_uses='3' if f==1 else '2',fine_S_feature_uses='0' if f==1 else '2',
            fine_optimizer_feature_uses='0' if f==1 else '2'))
        rows.append(dict(row_type='FEATURE_GROUP',frame_id=str(f),track_id=str(track),
                         semantic_region='PERSON_INTERIOR',feature_count='4'))
        labels.append(dict(frame_id=str(f),timestamp=str(f/10),track_id=str(track),gt_person_id='human_A',
            motion_label='MOVING' if f==1 else 'STATIC',visible='1',association_correct='0' if f==1 else '1',notes='synthetic'))
    def save(path, records):
        fields=sorted({k for r in records for k in r})
        with path.open('w') as file:
            writer=csv.DictWriter(file,fields);writer.writeheader();writer.writerows(records)
    save(root/'diagnostics.csv',rows);save(root/'human.csv',labels)
    command=['python3',str(ROOT/'tools/evaluate_instance_identity.py'),'--diagnostics',str(root/'diagnostics.csv'),
             '--annotations',str(root/'human.csv'),'--output',str(root/'result')]
    subprocess.run(command,check=True,capture_output=True,text=True)
    metrics=json.loads((root/'result/metrics.json').read_text())
    assert metrics['id_switches']==metrics['fragments']==metrics['wrong_S_inheritance']==1
    assert metrics['wrong_S_coarse_optimizer_feature_uses']==3 and metrics['wrong_S_fine_optimizer_feature_uses']==0
    assert metrics['static_retention_use_ratio']==.5 and metrics['dynamic_leakage_use_ratio']==0
    recovery=list(csv.DictReader((root/'result/reacquisition_static_recovery.csv').open()))
    assert len(recovery)==1 and abs(float(recovery[0]['seconds'])-.1)<1e-8
    # Existing outputs cannot be overwritten.
    assert subprocess.run(command,capture_output=True).returncode!=0
    labels[0]['association_correct']='';save(root/'missing_truth.csv',labels)
    command[command.index('--annotations')+1]=str(root/'missing_truth.csv')
    command[command.index('--output')+1]=str(root/'invalid')
    assert subprocess.run(command,capture_output=True).returncode!=0
print('PASS synthetic independent annotation metrics, recovery, missing-truth and no-overwrite checks')
