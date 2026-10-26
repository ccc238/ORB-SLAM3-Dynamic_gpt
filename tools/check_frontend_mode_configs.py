#!/usr/bin/env python3
"""Non-overwriting startup checks, including deliberately invalid YAML fixtures."""
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    output = ROOT / 'results/frontend_stage_validation' / ('config_checks_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    configs = ROOT / 'configs/frontend_modes/rgbd_tum3'
    base = {i: (configs / name).read_text() for i, name in enumerate((
        'mode0_original.yaml', 'mode1_b1_hard_mask.yaml',
        'mode2_evidence_only.yaml', 'mode3_action_reserved.yaml'))}
    cases = [(f'mode{i}', base[i], 2 if i == 3 else 0, 'not implemented' if i == 3 else '') for i in range(4)]
    def replace(text, key, value):
        result, count = re.subn(r'^' + re.escape(key) + r':.*$', key + ': ' + value, text, flags=re.M)
        if count != 1:
            raise RuntimeError(f'Expected exactly one {key}, got {count}')
        return result
    cases += [
        ('mode0_missing_model', replace(base[0], 'Yolo.ModelPath', '"/nonexistent/frontend_test_model.onnx"'), 0, ''),
        ('mode2_missing_model', replace(base[2], 'Yolo.ModelPath', '"/nonexistent/frontend_test_model.onnx"'), 2, ''),
        ('invalid_mode', replace(base[0], 'DynamicFrontend.Mode', '4'), 2, 'DynamicFrontend.Mode'),
        ('fractional_mode', replace(base[0], 'DynamicFrontend.Mode', '0.5'), 2, 'DynamicFrontend.Mode'),
        ('mode0_b2_conflict', replace(base[0], 'B2.Enable', '1'), 2, 'requires'),
        ('mode2_mask_conflict', replace(base[2], 'MaskFeatureReject.Enable', '1'), 2, 'requires'),
        ('mode2_action_conflict', replace(base[2], 'B4.EvidenceOnly', '0'), 2, 'EvidenceOnly'),
        ('mode2_coarse_unimplemented', replace(base[2], 'TwoStage.CoarseFilter.Enable', '1'), 2, 'not implemented'),
        ('mode2_fine_unimplemented', replace(base[2], 'TwoStage.FinePolicy.Enable', '1'), 2, 'not implemented'),
        ('mode2_map_unimplemented', replace(base[2], 'TwoStage.MapAdmission.Enable', '1'), 2, 'not implemented'),
        ('mode2_coarse_pose_evidence', replace(base[2], 'TwoStage.UseCoarsePoseForB2', '1'), 0, ''),
        ('ambiguous_without_mode', re.sub(r'^DynamicFrontend.Mode:.*\n', '', base[2], flags=re.M), 2, 'explicit'),
        ('legacy_b4', (ROOT / 'Examples/RGB-D/TUM3_B4_balanced_validation.yaml').read_text(), 0, ''),
    ]
    rows = []
    cases += [
        ('b2_explicit_coarse', (configs/'mode2_b2_explicit_coarse_evidence.yaml').read_text(),0,''),
        ('b2_explicit_final', (configs/'mode2_b2_explicit_final_reference.yaml').read_text(),0,''),
    ]
    quality = (configs / 'mode2_coarse_quality_evidence.yaml').read_text()
    cases += [
        ('coarse_quality', quality, 0, ''),
        ('coarse_quality_inliers_zero', replace(quality,'TwoStage.MinCoarseInliers','0'),2,'thresholds'),
        ('coarse_quality_ratio_invalid', replace(quality,'TwoStage.MinCoarseInlierRatio','1.1'),2,'thresholds'),
        ('coarse_quality_inliers_fractional', replace(quality,'TwoStage.MinCoarseInliers','30.5'),2,'integer'),
        ('coarse_quality_fallback_required', replace(quality,'TwoStage.Fallback.Enable','0'),2,'Fallback.Enable'),
    ]
    proposal = (configs / 'mode2_feature_policy_evidence.yaml').read_text()
    cases += [
        ('feature_policy_evidence', proposal, 0, ''),
        ('feature_policy_action_rejected', replace(proposal, 'FeaturePolicy.EvidenceOnly', '0'), 2, 'statistics only'),
        ('feature_policy_requires_annotations', replace(proposal, 'FeatureSemantic.Enable', '0'), 2, 'requires'),
        ('feature_policy_disabled', replace(proposal, 'FeaturePolicy.Enable', '0'), 0, ''),
    ]
    coarse_action = (configs/'mode3_coarse_filter_validation.yaml').read_text()
    cases += [
        ('mode3_coarse_only',coarse_action,0,''),
        ('mode3_missing_model',replace(coarse_action,'Yolo.ModelPath','"/nonexistent/frontend_test_model.onnx"'),2,''),
        ('mode3_requires_coarse',replace(coarse_action,'TwoStage.CoarseFilter.Enable','0'),2,'coarse-only requires'),
        ('mode3_requires_coarse_pose',replace(coarse_action,'TwoStage.UseCoarsePoseForB2','0'),2,'coarse-only requires'),
        ('mode3_requires_fallback',replace(coarse_action,'TwoStage.Fallback.Enable','0'),2,'Fallback.Enable'),
        ('mode3_requires_annotation',replace(coarse_action,'FeatureSemantic.Enable','0'),2,'coarse-only requires'),
        ('mode3_requires_policy',replace(coarse_action,'FeaturePolicy.Enable','0'),2,'coarse-only requires'),
        ('mode3_requires_explicit_policy_action',replace(coarse_action,'FeaturePolicy.EvidenceOnly','1'),2,'coarse-only requires'),
        ('mode3_fine_requires_b4_action',replace(coarse_action,'TwoStage.FinePolicy.Enable','1'),2,'EvidenceOnly'),
        ('mode3_no_map_action',replace(coarse_action,'TwoStage.MapAdmission.Enable','1'),2,'not implemented'),
        ('mode3_b4_stays_evidence',replace(coarse_action,'B4.EvidenceOnly','0'),2,'EvidenceOnly'),
    ]
    fine_action=(configs/'mode3_fine_policy_validation.yaml').read_text()
    cases += [
        ('mode3_fine_action',fine_action,0,''),
        ('fine_requires_b4_action',replace(fine_action,'B4.EvidenceOnly','1'),2,'EvidenceOnly'),
        ('fine_cannot_disable_coarse',replace(fine_action,'TwoStage.CoarseFilter.Enable','0'),2,'requires'),
        ('fine_with_map_admission',replace(fine_action,'TwoStage.MapAdmission.Enable','1'),0,''),
        ('fine_no_pending_emergency',replace(fine_action,'B4.Pending.UseForPose','1'),2,'no pending emergency'),
        ('fine_no_pending_map_switch',replace(fine_action,'B4.Pending.AllowMapCreation','1'),2,'map-creation'),
        ('fine_no_dynamic_map_switch',replace(fine_action,'B4.Dynamic.AllowMapCreation','1'),2,'map-creation'),
        ('fine_keeps_b2_readonly',replace(fine_action,'B2.EvidenceOnly','0'),2,'read-only'),
        ('fine_keeps_b3_readonly',replace(fine_action,'B3.EvidenceOnly','0'),2,'read-only'),
    ]
    for name, text, expected, message in cases:
        fixture = output / (name + '.yaml')
        fixture.write_text(text)
        completed = subprocess.run([str(ROOT / 'build_yolo11/bin/yolo_seg_smoke'), '--config-only', str(fixture)],
                                   capture_output=True, text=True, timeout=60)
        log = completed.stdout + completed.stderr
        (output / (name + '.log')).write_text(log)
        passed = completed.returncode == expected and (not message or message in log)
        rows.append(dict(case=name, exit_code=completed.returncode, expected=expected, passed=passed))
        (output / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
        print(f'{name}: {"PASS" if passed else "FAIL"} exit={completed.returncode}', flush=True)
        if not passed:
            raise RuntimeError(log)
    print('CONFIG CHECKS PASSED ' + str(output))


if __name__ == '__main__':
    main()
