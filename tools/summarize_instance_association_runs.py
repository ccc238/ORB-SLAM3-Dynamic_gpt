#!/usr/bin/env python3
"""Summarize measured policy audits/ID continuity, NOT identity accuracy."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import tarfile

ROOT=Path(__file__).resolve().parents[1]


def read(path):
    with path.open() as f:return list(csv.DictReader(f))


def write(path, rows):
    if not rows:return
    with path.open('w') as f:
        w=csv.DictWriter(f,list(rows[0]));w.writeheader();w.writerows(rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--validation',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    source_hashes={}
    for folder in ['include','src','tools','configs/instance_association','docs']:
        for path in (ROOT/folder).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:
                source_hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    source_hashes['CMakeLists.txt']=hashlib.sha256((ROOT/'CMakeLists.txt').read_bytes()).hexdigest()
    (a.output/'source_sha256.json').write_text(json.dumps(source_hashes,indent=2)+'\n')
    with tarfile.open(a.output/'source_snapshot.tar.gz','w:gz') as archive:
        for name in source_hashes: archive.add(ROOT/name,arcname=name)
    (a.output/'worktree.patch').write_bytes(subprocess.check_output(['git','diff','--binary'],cwd=ROOT))
    for name,cmd in [('commit.txt',['git','rev-parse','HEAD']),('branch.txt',['git','branch','--show-current']),
                     ('git_status.txt',['git','status','--short'])]:
        (a.output/name).write_bytes(subprocess.check_output(cmd,cwd=ROOT))
    runs=[];tracks=[]
    for measured in json.loads((a.validation/'sequence_summary.json').read_text()):
        folder=a.validation/measured['run'];rows=read(folder/'dynamic_frontend.csv')
        visible=[r for r in rows if r['row_type']=='INSTANCE' and int(r['instance_id'])>=0]
        frames=[r for r in rows if r['row_type']=='FRAME']
        association=read(next(folder.glob('instance_association_*.csv')))
        identities=[r for r in association if r['row_type']=='INSTANCE']
        counts=Counter(r['association_status'] for r in identities)
        untrusted=[r for r in visible if r['allow_state_inheritance']=='0']
        timing={r['frame_id']:float(r['association_ms']) for r in identities}
        runs.append(dict(run=folder.name,frames=len(frames),exit_code=int((folder/'exit_code.txt').read_text()),
            camera_poses=measured['camera_poses'],visible_instances=len(visible),
            new=counts['NEW_TRACK'],continuous=counts['CONTINUOUS_CONFIRMED'],
            reacquired=counts['REACQUIRED'],ambiguous=counts['AMBIGUOUS'],
            unique_system_ids=len({r['track_id'] for r in identities}),
            untrusted_instances=len(untrusted),
            untrusted_authorized_coarse=sum(r['allow_coarse']=='1' for r in untrusted),
            untrusted_actual_coarse_uses=sum(int(r['coarse_optimizer_feature_uses']) for r in untrusted),
            untrusted_actual_coarse_S_uses=sum(int(r['coarse_S_feature_uses']) for r in untrusted),
            untrusted_actual_fine_uses=sum(int(r['fine_optimizer_feature_uses']) for r in untrusted),
            untrusted_map_authorized=sum(r['allow_map']=='1' for r in untrusted),
            association_ms_mean_visible_frames=statistics.mean(timing.values()) if timing else '',
            state_S_visible=sum(r['current_b4_state']=='S' for r in visible),
            state_P_visible=sum(r['current_b4_state']=='P' for r in visible),
            state_D_visible=sum(r['current_b4_state']=='D' for r in visible),
            tracking_not_OK=sum(r['tracking_state']!='2' for r in frames)))
        groups=defaultdict(list)
        for r in identities:groups[r['track_id']].append(r)
        for track, observations in groups.items():
            observations.sort(key=lambda r:int(r['frame_id']))
            gaps=[int(b['frame_id'])-int(c['frame_id'])-1 for c,b in zip(observations,observations[1:])]
            tracks.append(dict(run=folder.name,track_id=track,visible_frames=len(observations),
                first_frame=observations[0]['frame_id'],last_frame=observations[-1]['frame_id'],
                maximum_observation_gap=max(gaps,default=0),
                ambiguous_frames=sum(r['association_status']=='AMBIGUOUS' for r in observations),
                reacquisitions=sum(r['association_status']=='REACQUIRED' for r in observations)))
    write(a.output/'run_audit_summary.csv',runs)
    write(a.output/'system_track_continuity.csv',tracks)
    (a.output/'scope.txt').write_text('No human identity/motion ground truth. System ID continuity is descriptive only, not ID switches/fragments against truth. Coarse fallback may use P; actual uses include optimization retries. No ATE improvement claim.\n')
    print(json.dumps(runs,indent=2))


if __name__=='__main__':main()
