#!/usr/bin/env python3
"""Checks use local model/data; no downloads and no original binaries are executed."""
from pathlib import Path
import datetime
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT/'build_yolo11/bin'
IMAGE = '/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/rgb/1341846313.592026.png'


def main():
    report = ROOT/'build_yolo11/validation'/('integration_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    report.mkdir()
    results = []
    model_path='/home/aaa/models/yolo/yolo11s-seg.onnx'
    fixtures={
        'disabled_missing_model': 'Yolo.Enable: 0\nYolo.ModelPath: "/definitely_missing/yolo.onnx"\nMaskPostProcess.Enable: 0\n',
        'enabled_missing_model': 'Yolo.Enable: 1\nYolo.ModelPath: "/definitely_missing/yolo.onnx"\nMaskPostProcess.Enable: 0\n',
        'dependency_error': 'Yolo.Enable: 0\nMaskPostProcess.Enable: 1\n',
        'preview_dependency_error': 'Yolo.Enable: 0\nYolo.ShowMask: 1\n',
        'b2_requires_yolo': 'Yolo.Enable: 0\nB2.Enable: 1\nB2.EvidenceOnly: 1\n',
        'config_conflict': 'Yolo.Enable: 0\nYolo.DilateRadius: 3\nMaskPostProcess.Enable: 0\n',
        'legacy_disabled': 'Yolo.Enable: 0\nYolo.DilateRadius: 3\n',
        'legacy_enabled': f'Yolo.Enable: 1\nYolo.ModelPath: "{model_path}"\nYolo.DilateRadius: 3\n',
        'b2_decision_mode_error': f'Yolo.Enable: 1\nYolo.ModelPath: "{model_path}"\nB2.Enable: 1\nB2.EvidenceOnly: 0\n',
        'b2_evidence_only': f'Yolo.Enable: 1\nYolo.ModelPath: "{model_path}"\nB2.Enable: 1\nB2.EvidenceOnly: 1\n',
    }
    for name,mask,overlay in [('save_mask_only',1,0),('save_overlay_only',0,1)]:
        fixtures[name]=f'Yolo.Enable: 1\nYolo.ModelPath: "{model_path}"\nYolo.SaveMask: {mask}\nYolo.SaveOverlay: {overlay}\nYolo.OutputDir: "{report}"\n'
    for name,body in fixtures.items(): (report/(name+'.yaml')).write_text('%YAML:1.0\n'+body)
    def check(name, command, expected=0, contains=None):
        p = subprocess.run([str(x) for x in command],cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=120)
        (report/(name+'.log')).write_text(p.stdout)
        ok = p.returncode==expected and (contains is None or contains in p.stdout)
        results.append(dict(name=name,returncode=p.returncode,expected=expected,passed=ok))
        print(f'{name}: {"PASS" if ok else "FAIL"}',flush=True)
        if not ok: raise RuntimeError(p.stdout)
    for group in ['baseline','yolo_only','yolo_dilate']:
        check(group,[BIN/'yolo_seg_smoke',ROOT/f'configs/ablation/rgbd_tum3/{group}.yaml',IMAGE,report/group])
    for name, expected, message in [
        ('disabled_missing_model',0,'PASS disabled'),
        ('enabled_missing_model',2,'YOLO model does not exist'),
        ('dependency_error',2,'requires Yolo.Enable=1'),
        ('preview_dependency_error',2,'Yolo.ShowMask=1 requires Yolo.Enable=1'),
        ('b2_requires_yolo',2,'B2.Enable=1 requires Yolo.Enable=1'),
        ('config_conflict',2,'Configuration conflict'),
        ('legacy_disabled',0,'CONFIG migration'),
        ('legacy_enabled',0,'CONFIG migration'),
        ('b2_decision_mode_error',2,'B2 formal decision mode is not implemented yet'),
        ('b2_evidence_only',0,'[B2 CONFIG]')]:
        check(name,[BIN/'yolo_seg_smoke',report/(name+'.yaml'),IMAGE],expected,message)
    check('frontend_mask_and_single_dilation',[BIN/'frontend_mask_test',report/'yolo_only_mask.png',report/'yolo_dilate_mask.png'])
    for name in ['save_mask_only','save_overlay_only']:
        parent=report
        before=set(parent.glob('masks_*'))
        check(name,[BIN/'yolo_seg_smoke',parent/(name+'.yaml'),IMAGE])
        created=set(parent.glob('masks_*'))-before
        if len(created)!=1: raise RuntimeError('Expected exclusive saved-image directory')
        directory=created.pop()
        masks=list(directory.glob('*_mask.png')); overlays=list(directory.glob('*_overlay.png'))
        expected=(3,0) if name=='save_mask_only' else (0,3)
        if (len(masks),len(overlays))!=expected: raise RuntimeError('Save toggles are not independent')
    for camera in ['rgbd_tum3','stereo_euroc']:
        texts = [(ROOT/f'configs/ablation/{camera}/{group}.yaml').read_text() for group in ['baseline','yolo_only','yolo_dilate']]
        normalized = ['\n'.join(line for line in t.splitlines() if not line.startswith(('Yolo.Enable:','MaskPostProcess.Enable:'))) for t in texts]
        if len(set(normalized))!=1: raise RuntimeError(f'{camera}: ablation parameters differ')
        results.append(dict(name=camera+'_config_consistency',passed=True))
    original = subprocess.check_output(['git','show','HEAD:src/Tracking.cc'],cwd=ROOT,text=True)
    current = (ROOT/'src/Tracking.cc').read_text()
    def mono(s): return s[s.index('Sophus::SE3f Tracking::GrabImageMonocular'):s.index('void Tracking::GrabImuData')]
    if mono(original)!=mono(current): raise RuntimeError('Monocular entry changed')
    results.append(dict(name='monocular_entry_unchanged',passed=True))
    original_frame=subprocess.check_output(['git','show','HEAD:src/Frame.cc'],cwd=ROOT,text=True)
    current_frame=(ROOT/'src/Frame.cc').read_text()
    def mono_frame(s):
        return s[s.index('Frame::Frame(const cv::Mat &imGray, const double'):s.index('void Frame::AssignFeaturesToGrid')]
    if mono_frame(original_frame)!=mono_frame(current_frame): raise RuntimeError('Monocular Frame constructor changed')
    results.append(dict(name='monocular_frame_unchanged',passed=True))
    model = Path('/home/aaa/models/yolo/yolo11s-seg.onnx')
    results.append(dict(name='model_sha256',value=hashlib.sha256(model.read_bytes()).hexdigest(),passed=True))
    check('original_artifacts',['sha256sum','-c',ROOT/'docs/yolo11_before/original_artifacts.sha256'])
    (report/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    print(f'Report: {report}',flush=True)


if __name__=='__main__': main()
