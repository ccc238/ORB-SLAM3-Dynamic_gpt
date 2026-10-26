# B3 freeze inventory — 2026-10-20

This is an immutable-input inventory made immediately before further B4-A
iteration.  It deliberately does not alter B2 residual computation or B3
fusion rules.

## Source state

- `HEAD`: `fe5b4ade86b247580cc9565e0b21d86a2b81faaf`
- Worktree: dirty (B1/B2/B3 additions, build products, results, and B4-A WIP
  are not represented by `HEAD`).  No all-files commit was made because that
  would silently include pre-existing user changes and generated artifacts.

## Located configuration inputs

| Path | SHA-256 |
| --- | --- |
| `Examples/RGB-D/TUM3.yaml` | `c4166a8902935b0b370e90c366f7577d77eabeea21a02f41eecf02d956211d80` |
| `configs/ablation/rgbd_tum3/baseline.yaml` | `406967645fdf4ac8a280a82d431607e265c782eb03daca63928db728653983bc` |
| `configs/ablation/rgbd_tum3/yolo_only.yaml` | `743da54b2ee533d1385e094cddeaaeed3ec7bea226f029e75ae1581683cbd623` |
| `configs/ablation/rgbd_tum3/yolo_dilate.yaml` | `8a617d9e6d1d529dca160389e8a9cbe4de1ebadb706f0f0fc814b1621e97444a` |

## Located B2/B3 sitting run

Run directory: `results/tum_b2_b3_sitting_20261020_153741`

| Artifact | SHA-256 |
| --- | --- |
| `run.log` | `1b13ffb2146ef1b9824a020227a2b01404d2f404615a15176e45a77145c39069` |
| `b3_fusion_summary_1792481865627141451.csv` | `6977c06efdebab52248607121570852db669cae8140f42ab269994bef490c968` |
| `b2_evidence_1792481865627236613_7973/b2_instance_evidence.csv` | `78e37f16cec8e7a8d589c8472e0da5ec01450185b70cd2561a650248394d1e86` |
| `CameraTrajectory.txt` | `a5408bbc9a3803211e4af9a22c107bb03bc5064cf7a7cd702035586225e16477` |

The directory also contains its effective configuration, KeyFrame trajectory,
semantic/tracking timing logs, B2 candidate-feature CSV, and B3 track analysis.

## Not located in this repository worktree

- B0/B1 labelled configurations;
- a walking run log and its B2/B3 CSV;
- ATE/RPE result files.

Those artifacts must be supplied or copied from their external experiment
location before a complete, self-contained B3 baseline archive can be made.
