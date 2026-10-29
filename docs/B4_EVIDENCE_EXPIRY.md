# B4 evidence expiry (acquisition-time validity)

## Scope

This change bounds the lifetime of B4 reliable-evidence counters and historical
STATIC states. It does not change B2 residuals, B3 fusion, the 3/6/10 state
thresholds, BA, identity matching, or historical MapPoints. It is not a solution
to missed person detections: without a current person mask, image features can
still be labeled background by the existing annotation path.

The implementation is now ENABLED by default. YAML without the new keys uses
acquisition-time expiry with the limits below. This deliberately changes the
behavior of old B4-enabled YAML that previously omitted these settings. To
reproduce the old unlimited freeze behavior, explicitly set Enable to 0, or
use the existing off configuration. B4.Enable itself is unchanged: this
default does not turn on B4 in ORIGINAL/B1 modes.

```yaml
B4.EvidenceExpiry.Enable: 1
B4.EvidenceExpiry.MaxGapSeconds: 0.20
B4.EvidenceExpiry.MaxStaticAgeSeconds: 0.30
```

Limits must be finite, positive and `MaxGapSeconds <= MaxStaticAgeSeconds`,
even when the module is disabled. These are debugging initial values, not
optimized or validated accuracy thresholds. The parser uses double precision
for these seconds without changing the existing B2/B3 numeric reader.

## Clock, state and expiry order

Only RGB-D image acquisition timestamps are used, never inference duration,
wall-clock time or a frame-count approximation. The stored history consists of
`hasReliableEvidence` and `lastReliableTimestamp`. A new track has no reliable
history; its timestamp and age are NaN internally, not a fabricated zero.

Expiry is strictly `timestamp > lastReliableTimestamp + limit`, with no epsilon.
Equality at the stored timestamp deadline is valid. Timestamp arithmetic is
double precision; tests cover equality and the next representable later value.

For EVERY retained track, including missing instances, Process does this once:

1. Snapshot the historical state and age before this image's evidence.
2. If the reliable gap exceeds MaxGapSeconds, clear Ns/Nd/Nr.
3. If historical state S exceeds MaxStaticAgeSeconds, change S to P and clear
   Ns/Nd/Nr. P stays P. D stays D; timeout never restores D to use.
4. For a detected instance, apply the unchanged evidence transition rules.
5. Only reliable person static/moving evidence refreshes lastReliableTimestamp.
6. Preserve the original single missed-frame/TTL advancement. Identity expires
   at missed_frames > TrackTTL, independently of evidence expiry.

Example: P with Nd=2, then a gap over 0.20 s, then reliable MOVING gives Nd=1,
not D. Expired S followed by one reliable STATIC gives P with Ns=1, not S.
After a long gap the counter deadline remains exceeded until fresh reliable
evidence arrives; `counter_expired` may consequently repeat on multiple rows.
`counters_reset` distinguishes an actual nonzero counter reset from checking
already-zero expired counters.

## Coarse/fine/map consistency

`GetUsableState(trackId, imageTimestamp, state)` applies the SAME expiry helper
to a private track copy. It does not update counters, timestamps, track age,
missed_frames or frame guards. An expired S is returned as P before coarse
matching/optimization. `GetState` remains a raw-history diagnostic API and
must not be used to authorize coarse pose features.

Tracking uses GetUsableState when it captures previous feature states before
Track. The single Process call at the B2/B3/B4 boundary commits the expiry and
then current evidence. Current feature states are propagated through the
existing fine policy and KeyFrame map-admission path; no second expiry/TTL
updater is introduced. EvidenceOnly still only produces proposals; actions
depend on the existing frontend mode/policy switches.

The existing explicit coarse-filter failure fallback is retained: it may
restore the original associations for the original pose optimizer when static
support is insufficient. This is logged as a fallback, not a fresh STATIC
confirmation; B4 stays unreliable for evidence advancement and the existing
fine/keyframe gates still apply. Expiry does not silently replace that prior
fallback design with a new tracking algorithm.

## Sequence/time discontinuities

With expiry enabled, backward/nonadvancing acquisition timestamps and backward
frame IDs are rejected before Process mutates any history. Duplicate
frame/timestamp and duplicate track guards remain. Nonfinite timestamps are
always rejected. A preflight check at GrabImageRGBD rejects backward timestamps
before YOLO/instance association. Querying the same timestamp for display is
allowed; processing a second image at that timestamp is not.

`Reset()` clears B4 tracks and input guards. Tracking calls it at full reset,
active-map reset and CreateMapInAtlas when expiry is enabled. A new process or
new System constructs fresh B4 history. Restarted sequences in the same B4
object require an explicit reset; timestamp rollback is not silently repaired.
This does not add a second identity tracker or change InstanceTrackManager TTL.

## Logs and Viewer

B4 summary CSV and unified frontend INSTANCE rows include:

- `has_reliable_history`: whether reliable history existed BEFORE this image.
- `evidence_age_seconds`: age of that pre-update history; blank if absent;
  terminal uses `NA`. It is not the age after a current reliable refresh.
- `counter_expired`, `counters_reset`, `static_state_expired`, `expiry_reason`.
- `expiry_previous_state`: raw historical state before expiry.
- `expiry_current_state`: after expiry, before current evidence.
- `current_update_reason`: independent evidence/TTL reason; expiry cannot be
  overwritten by UNRELIABLE_EVIDENCE_FREEZE/P_ACCUMULATE_STATIC/etc.

B4 CSV keeps the old `previous_state/current_state` as raw-history/final-state.
Unified CSV `previous_b4_state` denotes the timestamp-usable coarse state;
`historical_b4_state` preserves its raw pre-expiry counterpart. The current B4
state, policy actions, counters and expiry event are shown by the Viewer; no
Viewer-side decision is made. A TTL-lost row records missing evidence rather
than fabricating a current reliable result.

## Paired configurations

Complete new camera configurations, copied from mode3_diagnostics_validation:

- `configs/frontend_modes/rgbd_tum3/mode3_b4_evidence_expiry_off.yaml`
- `configs/frontend_modes/rgbd_tum3/mode3_b4_evidence_expiry_on.yaml`

Only EvidenceExpiry.Enable differs between these files; B2 thresholds, B3,
B4 3/6/10/15, camera, ORB, fine policy and map admission remain identical.
Old YAML files are not overwritten. These are full action-mode ablations,
not merely EvidenceOnly visualization.

## Build and validation

An independent build directory avoids replacing build_yolo11/original binaries:

```bash
cd /home/aaa/ORB-SLAM3_original0
cmake -S . -B build_b4_expiry -DCMAKE_BUILD_TYPE=Release
cmake --build build_b4_expiry -j2
python3 tools/validate_b4_evidence_expiry.py \
  --build-dir build_b4_expiry \
  --output results/B4_evidence_expiry/validation_YOUR_UNIQUE_RUN \
  --short-sequences
```

The validator refuses existing output directories, runs the expiry test and
12 existing regressions, checks YAML defaults/invalid values/paired configs,
and optionally runs both switches on sitting_static and walking_xyz for 100
frames each. It records logs, commands, exit codes, configuration snapshots,
configuration hashes, commit, trajectories, B2/B3/B4 CSV and unified CSV.
The expiry test explicitly uses 1/30-second intervals, except deliberately
long gaps and legacy-disabled scenarios. It exercises synthetic fine-policy
and KeyFrame map-admission rejection of expired P features, without deleting
MapPoints or modifying the optimizer.

Historical stage-specific audit scripts with unconditional "unreliable means
all state/counters unchanged" assertions target expiry-disabled configurations.
For expiry-enabled results use this validator and the explicit expiry columns:
an expiry-induced S->P must not be counted as a reliable-moving S_TO_P event.

## Manual full-sequence experiment

Run from a desktop terminal for visualization. Change SWITCH to off/on and
SEQ to sitting_static/walking_xyz. The mktemp directory is unique, and the
working directory keeps outputs out of the dataset/repository source folders.

```bash
ROOT=/home/aaa/ORB-SLAM3_original0
SWITCH=on
SEQ=walking_xyz
mkdir -p "$ROOT/results/B4_evidence_expiry"
RUN=$(mktemp -d "$ROOT/results/B4_evidence_expiry/${SEQ}_${SWITCH}_XXXXXX")
cp "$ROOT/configs/frontend_modes/rgbd_tum3/mode3_b4_evidence_expiry_${SWITCH}.yaml" "$RUN/effective_config.yaml"
git -C "$ROOT" rev-parse HEAD > "$RUN/commit.txt"
sha256sum "$RUN/effective_config.yaml" > "$RUN/config.sha256"
cd "$RUN"
export LD_LIBRARY_PATH="$ROOT/build_b4_expiry/lib:$ROOT/Thirdparty/DBoW2/lib:$ROOT/Thirdparty/g2o/lib:$ROOT/third_party_ort/onnxruntime-linux-x64-1.17.3/lib:${LD_LIBRARY_PATH:-}"
set -o pipefail
"$ROOT/build_b4_expiry/bin/ablation_sequence" rgbd \
  "$ROOT/Vocabulary/ORBvoc.txt" "$RUN/effective_config.yaml" \
  "/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_${SEQ}" \
  "/home/aaa/dynamic_slam_experiments/associations/fr3_${SEQ}_associations.txt" \
  0 1 1 2>&1 | tee "$RUN/run.log"
SLAM_EXIT=${PIPESTATUS[0]}
printf '%s\n' "$SLAM_EXIT" > "$RUN/exit_code.txt"
```

Use viewer argument `0` instead of `1` for headless runs. Avoid image saving,
overlays and per-frame terminal logging equally in both groups for timing
comparisons, using new config snapshots rather than editing old results.

This patch makes no ATE improvement claim. Full-sequence repeated experiments,
visual motion labels, static retention, dynamic leakage, recovery delay,
tracking coverage and ATE remain necessary before selecting expiry limits.

## Validation completed in this workspace (2026-10-29)

The following results were obtained before the default changed from off to
on. Both dataset ablations explicitly specify Enable, so their interpretation
is unchanged. The later default-on regression is recorded separately below.

The independent Release build and incremental rebuild both completed with
`-j2`. The optional RealSense SDK was not found; configured RGB-D/stereo/mono
and test targets built successfully. Existing upstream/compiler warnings were
not treated as new algorithm failures. Build logs are in
`build_b4_expiry/{optimizer_build.log,build.log,rebuild.log}`.

Results: `results/B4_evidence_expiry/validation_20261029_01/`.
All 13 unit/regression tests and 12 configuration cases met their expected
exit codes. The seven invalid YAML cases correctly exited with code 2.

| Sequence | Expiry | Frames / camera poses | LOST | Actual counter-reset rows | S-expiry rows |
|---|---|---|---|---|---|
| sitting_static | off | 100 / 100 | 0 | 0 | 0 |
| walking_xyz | off | 100 / 100 | 0 | 0 | 0 |
| sitting_static | on | 100 / 100 | 0 | 0 | 0 |
| walking_xyz | on | 100 / 100 | 0 | 8 | 1 |

The walking-on run has 48 counter-expired rows, not 48 distinct nonzero reset
events. At frame 76, track 1, age 0.33315086364746094 s, raw S expired to P;
the coarse action was REJECT, fine REJECT and map DENY. The independent
current-update reason remained UNRELIABLE_EVIDENCE_FREEZE. This verifies the
intended separation between expiry and evidence updates in an actual run.

All four runs generated nonempty camera/keyframe trajectories, had one B4
row per frame/track, and passed final fine-input and indexed-map observation
audits with zero policy-forbidden entries/observations and zero unaligned
keyframes. Summary files: `summary.json`, `short_sequence_summary.json` and
`post_run_audit.json` in the validation directory.

Not performed: full-sequence repeated experiments, ATE/RPE comparison,
interactive GUI observation, visual motion-ground-truth evaluation, and live
System reset/sequence-switch scenarios. Reset/backward-time behavior was
covered by unit tests and its Tracking call sites were inspected. Real D
expiry/recovery was tested synthetically, not claimed from these short runs.
No original build_yolo11 binaries, old YAML or old experiment directories were
overwritten, and no Git commit was made.

## Default-on change

Both B4Config's C++ initializer and the shared YAML reader now default to
Enable=1. The explicit off/on YAML files are unchanged. The original legacy
state/identity tests explicitly disable expiry because their frameId-as-time
fixtures intentionally test unlimited freeze; enabled expiry tests continue
to use 1/30-second intervals. The validator checks the omitted-key default,
explicit off override, and a real B4-enabled Mode 3 YAML with expiry keys
omitted. MaxGapSeconds=0.20 and MaxStaticAgeSeconds=0.30 are unchanged.

On this 6.2 GiB RAM machine, Optimizer.cc was built separately before the
remaining `-j2` build, retaining the original -O3/-march=native options and
using GCC compile-time garbage collection parameters
`--param=ggc-min-expand=10 --param=ggc-min-heapsize=32768` to reduce compiler
memory pressure. These are compiler heap-management settings, not SLAM
runtime parameters or changes to the optimizer source/BA objective. The
completed heavy-object build log is
`build_b4_expiry/optimizer_default_on_lowmem.log`.

Default-on validation completed successfully: the remaining full build used
`cmake --build build_b4_expiry -j2` (log: `default_on_build.log`). All 13
unit/regression tests and 14 configuration checks met their expected exit
codes. In particular, omitted fields enabled expiry, explicit Enable=0
disabled it, and B4-enabled Mode 3 without any EvidenceExpiry fields used
Enable=1 / 0.20 / 0.30. Results are in
`results/B4_evidence_expiry/default_on_validation_20261029_01/summary.json`.
No dataset sequence or ATE/RPE experiment was rerun for this default change.
