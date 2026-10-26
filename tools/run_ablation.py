#!/usr/bin/env python3
"""Exclusive run directory with configuration, command, hashes, trajectories and timings."""
import argparse
import datetime
import hashlib
import json
import os
import re
import select
from pathlib import Path
import subprocess
import time


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['rgbd', 'stereo'], required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--sequence', type=Path, required=True)
    p.add_argument('--associations', type=Path, help='t_left path_left t_right path_right')
    p.add_argument('--output', type=Path, required=True, help='must not already exist')
    p.add_argument('--max-frames', type=int, default=0, help='0: entire sequence')
    p.add_argument('--timeout', type=int, default=1800)
    p.add_argument('--viewer', action='store_true', help='default: System viewer disabled')
    p.add_argument('--show-masks', action='store_true', help='live binary mask + person overlay; combine with --viewer for Current Frame and map; Q/Esc stops')
    p.add_argument('--no-pace', action='store_true', help='default: dataset timestamp pacing')
    args = p.parse_args()
    executable = root/'build_yolo11/bin/ablation_sequence'
    config = args.config.resolve(strict=True)
    sequence = args.sequence.resolve(strict=True)
    if not executable.is_file(): p.error(f'not built: {executable}')
    association = args.associations.resolve(strict=True) if args.associations else None
    if association is None and args.mode != 'stereo': p.error('--associations required for RGB-D')
    config_text=config.read_text()
    if args.show_masks:
        if not re.search(r'^Yolo.Enable:\s*1\s*(?:#.*)?$',config_text,re.M):
            p.error('--show-masks requires a configuration with Yolo.Enable: 1')
        if re.search(r'^Yolo.ShowMask:',config_text,re.M):
            config_text=re.sub(r'^Yolo.ShowMask:.*$','Yolo.ShowMask: 1',config_text,flags=re.M)
        else: config_text+='\nYolo.ShowMask: 1\n'
    show_masks=bool(re.search(r'^Yolo.ShowMask:\s*1\s*(?:#.*)?$',config_text,re.M))
    if show_masks and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        p.error('live masks require a graphical desktop; run this command from the Ubuntu desktop terminal')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    effective = out/'effective_config.yaml'
    effective.write_text(config_text)
    if association is None:
        def read_camera(name):
            records = []
            for line in (sequence/f'mav0/{name}/data.csv').read_text().splitlines():
                if not line.strip() or line.startswith('#'): continue
                stamp, filename = line.strip().split(',')[:2]
                records.append((stamp, filename.strip()))
            return records
        right = dict(read_camera('cam1'))
        association = out/'stereo_associations.txt'
        lines = []
        for stamp, name in read_camera('cam0'):
            if stamp in right:
                seconds = f'{int(stamp)//10**9}.{int(stamp)%10**9:09d}'
                lines.append(f'{seconds} mav0/cam0/data/{name} {seconds} mav0/cam1/data/{right[stamp]}')
        if not lines: raise RuntimeError('No synchronized EuRoC pairs')
        association.write_text('\n'.join(lines)+'\n')
    command = [str(executable), args.mode, str(root/'Vocabulary/ORBvoc.txt'), str(effective),
               str(sequence), str(association), str(args.max_frames), str(int(args.viewer)), str(int(not args.no_pace))]
    manifest = dict(command=command, config_source=str(config), config_sha256=digest(config),
                    effective_config_sha256=digest(effective), show_masks=show_masks, viewer=args.viewer,
                    association=str(association), association_sha256=digest(association),
                    executable_sha256=digest(executable),
                    start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    text = effective.read_text()
    yolo_enabled = bool(re.search(r'^Yolo.Enable:\s*1\s*(?:#.*)?$',text,re.M))
    model_match = re.search(r'^Yolo.ModelPath:\s*[\"\']?([^\"\'\n]+)',text,re.M)
    model = Path(model_match.group(1).strip()) if model_match else None
    manifest['local_model_sha256'] = digest(model) if yolo_enabled and model and model.is_file() else None
    (out/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    start = time.perf_counter()
    print('Running. Results: '+str(out),flush=True)
    if show_masks:
        targets='Current Frame, mask or overlay' if args.viewer else 'mask or overlay'
        print(f'Live preview: focus a {targets} window; Q or Esc stops and saves the processed trajectory.',flush=True)
    environment = os.environ.copy()
    environment['LD_LIBRARY_PATH'] = ':'.join([
        str(root/'build_yolo11/lib'),
        str(root/'third_party_ort/onnxruntime-linux-x64-1.17.3/lib'),
        environment.get('LD_LIBRARY_PATH','')])
    with open(out/'run.log','w') as log:
        try:
            # Keep a complete reproducible log while exposing per-frame
            # frontend diagnostics immediately in the user's terminal.
            process = subprocess.Popen(command,cwd=out,stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT,text=True,bufsize=1,
                                       env=environment)
            deadline=time.monotonic()+args.timeout
            status=None
            while True:
                remaining=deadline-time.monotonic()
                if remaining <= 0:
                    process.kill()
                    process.wait()
                    status=124
                    break
                ready, _, _ = select.select([process.stdout], [], [], min(0.2,remaining))
                if ready:
                    line=process.stdout.readline()
                    if line:
                        log.write(line)
                        log.flush()
                        print(line,end='',flush=True)
                        continue
                if process.poll() is not None:
                    # Drain bytes still buffered after child exit.
                    for line in process.stdout:
                        log.write(line)
                        print(line,end='',flush=True)
                    log.flush()
                    status=process.returncode
                    break
        except subprocess.TimeoutExpired:
            status = 124
    manifest.update(exit_code=status, duration_s=time.perf_counter()-start)
    (out/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (out/'exit_code.txt').write_text(str(status)+'\n')
    counts = {}
    for name in ['CameraTrajectory.txt','KeyFrameTrajectory.txt']:
        path = out/name
        counts[name] = len(path.read_text().splitlines()) if path.exists() else 0
    (out/'trajectory_line_count.txt').write_text('\n'.join(f'{v} {k}' for k,v in counts.items())+'\n')
    validation_status = status if status else (0 if all(counts.values()) else 3)
    manifest['validation_status'] = validation_status
    (out/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(output=str(out),exit_code=status,validation_status=validation_status,trajectory_lines=counts),indent=2))
    raise SystemExit(validation_status)


if __name__=='__main__': main()
