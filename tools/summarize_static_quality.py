#!/usr/bin/env python3
"""Audit completed paired experiments; describe frame-level weight distributions."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import statistics
from collections import Counter

def quantile(values,p):
    values=sorted(values)
    if not values:return ''
    index=(len(values)-1)*p;low=int(index);high=min(low+1,len(values)-1)
    return values[low]+(values[high]-values[low])*(index-low)

def write_csv(path,rows):
    if not rows:return
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('validation',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--sequences',nargs='+',default=['sitting_static','walking_xyz'])
    a=p.parse_args();base=a.validation.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    rows=list(csv.DictReader((base/'comparison.csv').open()))
    checks=json.loads((base/'checks.json').read_text())
    assert all(r.get('passed',False) for r in checks),'Validation contains skipped or failed checks'
    full=[r for r in rows if r['stage']=='full']
    audit=[];distributions=[];aggregate=[]
    for seq in a.sequences:
        for group in ['off','on']:
            runs=[r for r in full if r['sequence']==seq and r['quality']==group]
            assert len(runs)==a.repeats and len({r['repeat'] for r in runs})==a.repeats
            summary=dict(sequence=seq,quality=group,repeats=len(runs))
            for key in ['ate_rmse','ate_median','rpe_trans_rmse','rpe_rot_rmse','track_mean_ms','quality_compute_mean_ms','quality_valid_ratio','weight_mean']:
                values=[float(r[key]) for r in runs if r.get(key)]
                if values:
                    summary[key+'_median']=statistics.median(values)
                    summary[key+'_mean']=statistics.mean(values)
                    summary[key+'_std']=statistics.pstdev(values)
                    summary[key+'_min']=min(values);summary[key+'_max']=max(values)
            summary['lost_frames_total']=sum(int(float(r['lost_frames'])) for r in runs)
            aggregate.append(summary)
            for r in runs:
                run=f"full_{seq}_{group}_r{r['repeat']}";folder=base/run
                assert int((folder/'exit_code.txt').read_text())==0
                fine=list(csv.DictReader(next(folder.glob('fine_feature_policy_*.csv')).open()))
                forbidden=sum(int(x[k]) for x in fine for k in ['P_at_optimizer','D_at_optimizer','forbidden_at_optimizer'])
                assert forbidden==0
                maps=list(csv.DictReader((folder/'map_admission_final_audit.csv').open()))
                assert all(int(x['forbidden_observations'])==int(x['unaligned_keyframes'])==0 for x in maps)
                b4=list(csv.DictReader(next(folder.glob('b4_state_summary_*.csv')).open())) if list(folder.glob('b4_state_summary_*.csv')) else []
                if b4:
                    ids=[(x['frame_id'],x['track_id']) for x in b4];assert len(ids)==len(set(ids))
                record=dict(run=run,exit_code=0,forbidden_optimizer_inputs=forbidden,forbidden_map_observations=0)
                results=[x for x in fine if x.get('phase')=='result']
                if results:record['fine_inliers_mean']=statistics.mean(int(x['fine_inliers']) for x in results)
                record['active_keyframes']=int(maps[-1]['active_keyframes'])
                visible=[x for x in b4 if int(x['instance_id'])>=0]
                for state,count in Counter(x['current_state'] for x in visible).items():record['b4_'+state+'_instance_frames']=count
                b3=list(csv.DictReader(next(folder.glob('b3_fusion_summary_*.csv')).open()))
                for state,count in Counter(x['fusion_category'] for x in b3).items():record['b3_'+state]=count
                if group=='on':
                    q=list(csv.DictReader(next(folder.glob('static_quality_*.csv')).open()))
                    assert len(q)==int(float(r['frames']))
                    observed=[x for x in q if int(x['allowed_matches'])]
                    for x in observed:
                        n=int(x['allowed_matches']);valid=int(x['depth_quality_valid']);neutral=int(x['neutral_weight_count'])
                        assert n==valid+neutral
                        assert 0<float(x['weight_min'])<=float(x['weight_mean'])<=1+1e-12
                        assert 0<float(x['effective_sample_size'])<=n+1e-6
                    for key in ['weight_min','weight_mean','effective_sample_size','quality_compute_ms']:
                        values=[float(x[key]) for x in observed]
                        distributions.append(dict(run=run,statistic=key,denominator='frames_with_optimizer_observations',
                            count=len(values),mean=statistics.mean(values),p05=quantile(values,.05),
                            p50=quantile(values,.50),p95=quantile(values,.95),maximum=max(values)))
                    reasons={}
                    for x in q:reasons[x['skip_reason']]=reasons.get(x['skip_reason'],0)+1
                    record.update(quality_frames=len(q),quality_skip_reasons=json.dumps(reasons))
                    n=sum(int(x['allowed_matches']) for x in q)
                    record['neutral_observation_ratio']=sum(int(x['neutral_weight_count']) for x in q)/max(1,n)
                else:assert not list(folder.glob('static_quality_*.csv'))
                audit.append(record)
    hashes=json.loads((base/'runtime_hashes.json').read_text())
    assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest for path,digest in hashes.items())
    write_csv(out/'aggregate.csv',aggregate)
    write_csv(out/'frame_weight_distributions.csv',distributions)
    write_csv(out/'invariant_audit.csv',audit)
    (out/'audit.json').write_text(json.dumps(dict(passed=True,runs=len(full),checks=len(checks),
        notes='Weight quantiles are FRAME SUMMARY distributions, not per-feature weight histogram; no motion ground truth inferred.'),indent=2))
    print(json.dumps(aggregate,indent=2))

if __name__=='__main__':main()
