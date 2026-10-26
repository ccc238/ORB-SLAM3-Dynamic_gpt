# RGB-D dynamic frontend, batches 0–2

## Batch 0 recovery point

`freezes/pre_two_stage_20261022_104737/` contains the previous runnable
source archive, SHA256 manifest, runtime/model, actual B0/B1/B2/B3/B4
configurations, sitting/walking CSV/log/trajectories and current ATE/RPE.
The visualization supplement preserves 164 PNGs. No Git write was performed.
An external HEAD change from fe5b4ade to 34c523ee was observed; the core
source still matched the archive and that fact is recorded separately.

`results/frontend_stage_validation/batch0_legacy_20261022_113857/`
contains both 100-frame and complete-sequence legacy B4-A checks. Both
complete runs produced valid trajectories; sitting had zero MOVING/D and
walking had MOVING/D evidence. This is reproducibility of the existing
evidence behavior, not bitwise-identical multithreaded trajectories.

## Explicit mode interface

Configuration is parsed once alongside existing YOLO/B1/B2/B3/B4 settings
in `DynamicDetector`. `DynamicFrontendConfig` defaults Mode to 0 and all
TwoStage submodules to 0 except Fallback=1. Old YAML without a Mode key
stays on a printed `LEGACY_UNSPECIFIED` path, preserving old B2/B3/B4-A
experiments. Supplying TwoStage keys without explicit Mode is an error.

| Mode | YOLO | MaskFeatureReject | B2/B3/B4 | TwoStage.Enable | Status |
|---|---|---|---|---|---|
| 0 ORIGINAL | 0 | 0 | all 0 | 0 | full-image feature path; no semantic model loading |
| 1 B1_HARD_MASK | 1 | 1 | all 0 | 0 | original B1 hard-mask behavior |
| 2 TWO_STAGE_EVIDENCE_ONLY | 1 | 0 | all 1, all EvidenceOnly=1 | 1 | full-image ORB plus evidence; later coarse/fine scheduling remains pending |
| 3 TWO_STAGE_ACTION | 1 | 0 | all 1 | 1 | recognized/validated but startup explicitly rejected until action is implemented |

Mode is authoritative; incompatible subordinate settings fail startup rather
than silently changing the selected experiment. Missing MaskFeatureReject
defaults to enabled only in Mode 1 (legacy defaults remain unchanged).
Mode 2 forbids the four unimplemented processing switches: CoarseFilter,
UseCoarsePoseForB2, FinePolicy and MapAdmission. Fallback is reserved for the
later tracking split; its default does not introduce a new recovery policy.

As explicitly agreed, Mode 3 runtime acceptance is deferred, not counted as
a successfully executing action frontend. Modes 0/1/2 can run with YAML
changes only. Mode 2 currently uses the final Track() pose for B2 and prints
this stage limitation; it does not yet implement coarse/fine tracking.

Full configs: `configs/frontend_modes/rgbd_tum3/mode0_original.yaml`,
`mode1_b1_hard_mask.yaml`, `mode2_evidence_only.yaml`, and
`mode3_action_reserved.yaml`. Camera/ORB and existing B4 thresholds remain
identical to the copied TUM3 balanced config. Existing YAML is not overwritten.

ORIGINAL refers to the semantic-disabled feature/SLAM algorithm path in the
modified build. Previously shared stability guards and diagnostics remain;
do not infer binary/timing/bitwise equivalence to the preserved original
ORB-SLAM3 executable merely from selecting Mode 0.

## Verification discipline

Build with `cmake --build build_yolo11 --target ablation_sequence yolo_seg_smoke -- -j2`.
`tools/run_frontend_stage_validation.py` serially completes all 100-frame
cases before any full runs; failures stop the stage. Results use new
directories, effective YAML, command, exit status, trajectories, CSV and
validation metadata. Later batches must not change B2/B3/B4 algorithms in
order to make these initial mode checks pass.

## Batch 1 completed verification

`results/frontend_stage_validation/config_checks_20261022_144930/` records
16 startup checks: modes 0/1/2, intentionally rejected Mode 3, missing-model
behavior (Mode 0 succeeds, Mode 2 fails), invalid mode/type, conflicting
flags, reserved processing flags and the legacy B4 config. Historical
B0/B1/B2/B3 configs also passed startup checks without a Mode key.

`results/frontend_stage_validation/batch1_modes_20261022_144941/` passed
all 12 runs (3 modes x 2 sequences x short/full). Each successful process
exported nonempty camera/keyframe trajectories. These are engineering
checks, not an accuracy or performance claim. Notably, Mode 0 walking
reported 737 OK / 88 RECENTLY_LOST / 1 LOST and 825 trajectory rows from
827 inputs. That behavior is retained, not hidden by a different parameter
or algorithm. Full-image features can include moving people.

## Batch 2 extraction routing and diagnostics

Only `Tracking::GrabImageRGBD()` is changed: an explicit Mode 1 passes the
B1 mask to the main Frame; explicit Mode 0/2 pass an empty `cv::Mat`.
Mode 3 cannot reach this path because startup rejects it. Legacy YAML
continues using its existing `MaskFeatureReject.Enable` branch. Stereo and
monocular entry points, `Tracking::Track()` and map logic are not edited.

Each RGB-D image emits one `[DYNAMIC FRONTEND]` line with `frame_id`,
`dynamic_frontend_mode` (-1 means legacy), `raw_fast`, `b1_mask_rejected`,
`final_orb` and `person_main_orb`. The last field counts unique main-Frame
ORB keypoints inside the union of the original, undilated YOLO instance
masks. It is read-only and does not add semantic labels or change feature
use yet. Raw FAST is counted before octree selection; it is NOT an ORB
descriptor count. Existing `[FRONTEND].person_mask_ratio` still measures
the extraction exclusion mask, so it is zero in Mode 2 despite detections.

Builds use `-j2` and isolated `build_yolo11/bin` / `build_yolo11/lib` outputs.
Compatibility targets `rgbd_tum`, `stereo_euroc` and `mono_tum` compile.
`frontend_mask_test`, `b2_candidate_test` and `b4_state_machine_test` pass.
The old B2 test required fixture-only maintenance: replace removed generic
INSUFFICIENT_EVIDENCE with the actual INSUFFICIENT_MATCHES gate, provide
shared IDs from InstanceTrackManager instead of expecting B2 to create IDs,
and check B2 retains those IDs. No production B2/B3/B4 rule is changed.
Initial failed build/test logs are preserved alongside successful retries
in `results/frontend_stage_validation/batch2_unit_20261022_run01/`.

Data validation runs in
`results/frontend_stage_validation/batch2_mask_routing_20261022_150651/`.
Its `summary.json` is incremental: only a complete 12-case summary with all
`passed=true` constitutes sequence acceptance. After completion run
`python3 tools/audit_frontend_stage_scope.py <that directory>` to verify
frozen algorithm hashes, unchanged Tracking code outside GrabImageRGBD,
Mode 1 person exclusion and identical Mode 0/2 raw/final extraction counts.

The following stages remain UNIMPLEMENTED: coarse/fine Tracking split,
semantic/history annotation and feature-use policy, current-frame B4 action,
map-point admission. Evidence-only diagnostic success must not be described
as having implemented the full two-stage dynamic frontend.

## Batch 2 completed results

All 12 cases completed successfully, exit 0, with camera/keyframe trajectories.
The six full runs all reported zero RECENTLY_LOST and zero LOST:

| Mode | Sequence | Inputs / camera poses | Keyframes | Rejected FAST sum | Main ORB inside raw person masks |
|---|---|---|---|---|---|
| 0 | sitting_static | 680 / 680 | 14 | 0 | not measured (YOLO off) |
| 0 | walking_xyz | 827 / 827 | 459 | 0 | not measured (YOLO off) |
| 1 | sitting_static | 680 / 680 | 13 | 2446496 | 0 |
| 1 | walking_xyz | 827 / 827 | 127 | 1290575 | 0 |
| 2 | sitting_static | 680 / 680 | 15 | 0 | 215249 |
| 2 | walking_xyz | 827 / 827 | 270 | 0 | 131685 |

The person-ORB column is a sum over frames, not unique physical features.
`scope_audit.json` passed all 34 checks: unchanged frozen algorithms and
old YAML, unchanged Tracking outside GrabImageRGBD, Mode 1 person exclusion,
and exact per-frame equality of raw FAST / final ORB counts between Modes
0 and 2 for both short and full sequences. This verifies extraction counts,
not descriptor equality or trajectory accuracy.

Runs used viewer=0 and pace=0, not a controlled real-time speed benchmark.
Different YOLO overhead and scheduling of LocalMapping can substantially
change keyframe counts and trajectories (the two Mode 0 walking repeats
already differ). Do not attribute every such change to evidence decisions,
and do not claim Mode 0 bitwise equivalence to the independent original
binary. GUI display and new-mode ATE/RPE were not evaluated in this batch.

Current changes were not committed and no branch was switched. The stage
directory includes `implemented_sources.tar.gz` and `source_changes.patch`
alongside runtime hashes so the recorded commit is not mistaken for a
clean-worktree snapshot. The archive's document is the pre-completion copy;
this completed-results section is the later final report.

## Batch 9 update: partial Mode 3, coarse filtering only

The Mode 3 rejection above describes the original batches 1/2 scope. Batch 9
adds an explicit, narrowly validated coarse-only ACTION combination; the
old reserved Mode 3 YAML remains rejected. Mode 2 remains strictly read-only.
Fine filtering and map admission are still unavailable. See
[COARSE_FEATURE_FILTER.md](COARSE_FEATURE_FILTER.md) for the effective flags,
transactional rollback, tests, result paths and current limitations.

## Batch 10 update: current-state fine pose filtering

Mode 3 now also supports FinePolicy=1 paired with B4.EvidenceOnly=0; the
batch-9 coarse-only configuration remains available. Current-state filtering
runs before TrackLocalMap and again after SearchLocalPoints, with final
optimizer-input guards and no P/D restoration on fine failure. Map admission
is still disabled. See [FINE_FEATURE_POLICY.md](FINE_FEATURE_POLICY.md) for
the implementation, validated configuration, complete-sequence results and limitations.

## Batch 11 update: indexed persistent-map admission

Mode 3 can now enable `TwoStage.MapAdmission.Enable=1` together with fine
filtering and B4 action. KeyFrames carry index-preserving semantic/policy
snapshots. RGB-D bootstrap/depth creation, new observations, triangulation,
neighbor/Sim3 fusion and direct loop observations reject P/D/boundary/unknown.
The generic MapPoint observation API, historical dynamic-point cleanup and BA
objectives are unchanged. Disabled configurations remain available.
See [MAP_ADMISSION.md](MAP_ADMISSION.md) for implementation, archive compatibility,
reproduction commands, audits and complete-sequence results.

## Batch 12 update: keyframe decision provenance

Fine-action keyframes now require a reliable non-fallback coarse stage, current
semantic policy, completed fine optimization and sufficient final static support.
NeedNewKeyFrame explicitly passes the final local-map inlier snapshot into the
unchanged original heuristic; CreateNewKeyFrame rechecks approval before any
construction or map mutation. Bootstrap initialization is logged separately.
No new YAML switches or threshold changes were introduced. See
[KEYFRAME_DECISION_ORDER.md](KEYFRAME_DECISION_ORDER.md) for implementation,
stage CSVs, tests, compatibility and limitations.
