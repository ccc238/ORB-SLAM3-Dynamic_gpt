#!/usr/bin/env python3
"""GUI smoke test: run under xvfb-run; verify windows, then send Q via XTest."""
import ctypes
import argparse
import ctypes.util
import datetime
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',action='store_true',help='also require Current Frame and Pangolin map windows')
    parser.add_argument('--focus-current-frame',action='store_true',help='send Q to Current Frame instead of overlay (requires --viewer)')
    args=parser.parse_args()
    if args.focus_current_frame and not args.viewer:
        parser.error('--focus-current-frame requires --viewer')
    root=Path(__file__).resolve().parents[1]
    run=root/'results/yolo_preview_validation'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    command=[sys.executable,str(root/'tools/run_ablation.py'),'--mode','rgbd',
             '--config',str(root/'configs/ablation/rgbd_tum3/yolo_only.yaml'),
             '--sequence','/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz',
             '--associations','/home/aaa/dynamic_slam_experiments/associations/fr3_walking_xyz_associations.txt',
             '--output',str(run),'--max-frames','60','--show-masks','--timeout','120']
    if args.viewer: command.append('--viewer')
    process=subprocess.Popen(command,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    try:
        deadline=time.monotonic()+60
        windows=''; overlay_window=None
        current_found=False; map_found=False
        while time.monotonic()<deadline and process.poll() is None:
            windows=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
            current_found='ORB-SLAM3: Current Frame' in windows
            map_found='ORB-SLAM3: Map Viewer' in windows
            if 'YOLO person - rgbd - mask (white=exclude)' in windows:
                found=re.search(r'(0x[0-9a-fA-F]+)\s+"YOLO person - rgbd - overlay"',windows)
                if args.focus_current_frame:
                    found=re.search(r'(0x[0-9a-fA-F]+)\s+"ORB-SLAM3: Current Frame"',windows)
                if found and (not args.viewer or (current_found and map_found)):
                    overlay_window=int(found.group(1),16)
                    break
            time.sleep(.2)
        if overlay_window is None:
            raise RuntimeError('Expected preview/Viewer windows not found:\n'+windows)
        # Let several frames reach Tracking before asking for graceful shutdown.
        time.sleep(2)
        x11=ctypes.CDLL(ctypes.util.find_library('X11'))
        xtst=ctypes.CDLL(ctypes.util.find_library('Xtst'))
        x11.XOpenDisplay.argtypes=[ctypes.c_char_p]; x11.XOpenDisplay.restype=ctypes.c_void_p
        x11.XSetInputFocus.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_ulong]
        x11.XKeysymToKeycode.argtypes=[ctypes.c_void_p,ctypes.c_ulong]; x11.XKeysymToKeycode.restype=ctypes.c_uint
        x11.XFlush.argtypes=[ctypes.c_void_p]
        x11.XCloseDisplay.argtypes=[ctypes.c_void_p]
        xtst.XTestFakeKeyEvent.argtypes=[ctypes.c_void_p,ctypes.c_uint,ctypes.c_int,ctypes.c_ulong]
        display=x11.XOpenDisplay(None)
        if not display: raise RuntimeError('Cannot open X display for GUI event test')
        try:
            x11.XSetInputFocus(display,overlay_window,2,0)
            key=x11.XKeysymToKeycode(display,ord('q'))
            xtst.XTestFakeKeyEvent(display,key,1,0)
            xtst.XTestFakeKeyEvent(display,key,0,0)
            x11.XFlush(display)
        finally: x11.XCloseDisplay(display)
        stdout,_=process.communicate(timeout=60)
        print(stdout,flush=True)
        if process.returncode: raise RuntimeError('Preview runner failed')
        stats=(run/'tracking_timing.yaml').read_text()
        if not re.search(r'^preview_user_stopped:\s*1$',stats,re.M):
            raise RuntimeError('Q did not trigger graceful preview shutdown')
        frames=int(re.search(r'^frames:\s*(\d+)',stats,re.M).group(1))
        if not 0<frames<60: raise RuntimeError('Unexpected processed frame count')
        result=dict(passed=True,mask_window_found=True,overlay_window_found=True,
                    viewer_enabled=args.viewer,current_frame_window_found=current_found,
                    map_window_found=map_found,q_target='current_frame' if args.focus_current_frame else 'overlay',
                    stopped_with_q=True,processed_frames=frames,run=str(run))
        (run/'preview_gui_test.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
            process.wait()


if __name__=='__main__': main()
