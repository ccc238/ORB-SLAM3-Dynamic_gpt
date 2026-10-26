# Step 5: shared instance identity and once-only temporal updates

## Ownership and call order

RGB-D uses the existing synchronous flow:

```
YOLO -> InstanceTrackManager::Associate -> validate IDs -> main Frame
     -> complete Track -> B2::Process -> B3::Fuse
     -> RunB4TemporalStateMachine -> B4::Process -> UpdateB4State
     -> logs / CSV / Viewer
```

`InstanceTrackManager` is the only ID allocator. B2 copies the ID from the
raw semantic instance. B3 joins B2 by the same ID and rejects mismatches.
B4 keys its state table by that ID; it does not allocate a second identity.
New identity records become PENDING when first consumed by B4, before
applying the first B3 observation. There is no second mutable S/P/D copy
inside the association manager.

Unmatched detections receive new monotonically allocated IDs. Unmatched
history increments missed_frames once per association call. The existing
TTL behavior is retained: <= TTL stays active; > TTL terminates. Reappearance
after 15 missing frames may recover the old ID; reappearance after 16
missing frames cannot. Recovery still requires passing the configured
mask/box/center association gates, not just satisfying TTL. Visible but
unreliable geometry does not count as a missed detection.

## Added safety checks

- Before Frame construction, all current person IDs must be nonnegative
  and unique. With B4.Debug.PrintFrameSummary=1 and B4 enabled, one
  `[INSTANCE TRACK]` line records timestamp, count and ordered IDs.
- `B4TemporalStateMachine::Process` is the once-per-input-frame dispatcher;
  `UpdateB4State` is the existing UpdateTrackState implementation. The
  latter has a single call site and stays private.
- Re-dispatch of the most recent (frame_id, timestamp) is an explicit
  error. Timestamp distinguishes later input if the SLAM frame counter
  restarts. This guard assumes the existing synchronous, ordered Tracking
  thread; it is not a concurrent/out-of-order replay API.
- Duplicate or negative B3 track IDs are rejected before any mutation.
  Rejected input does not change evidence counters, ages or TTL.

S/P/D equations, thresholds, B2 residuals and B3 fusion rules are unchanged.
EvidenceOnly still applies NONE_EVIDENCE_ONLY. No feature, pose, keyframe
or map-point policy is introduced here.

## Tests and reproduction

Builds use the isolated build tree and `-j2`:

```
cmake --build build_yolo11 --target ablation_sequence b4_state_machine_test instance_track_lifecycle_test b2_candidate_test -- -j2
build_yolo11/bin/instance_track_lifecycle_test
build_yolo11/bin/b4_state_machine_test
```

The new lifecycle test covers new P state, one-to-one IDs, visible/unreliable
freeze, recovery after 15 missing frames with counters intact, expiry at
16, new identity without old state inheritance, and duplicate-frame/ID
rejection without double counting.

Sequence verification uses a new result directory:

```
python3 tools/run_frontend_stage_validation.py --stage step5_shared_track --mode-config 2=configs/frontend_modes/rgbd_tum3/mode2_evidence_only.yaml
python3 tools/audit_shared_track_updates.py <printed-batch-directory>
```

The audit joins pre-Frame ID logs to B2/B3/B4 CSV, checks one row per
frame/track, visible-instance missed_frames=0, age advancing once per
observation, TTL recoveries and evidence-only applied policies.

Build attempt logs are retained in
`results/frontend_stage_validation/step5_track_build_20261022_01/`.
The initial `-j2` build failed when cc1plus compiling Optimizer.cc was
externally killed (suspected memory pressure, not a reported syntax error).
An isolated Optimizer object build uses the same flags and `-j2` to avoid
competing with another heavy translation unit. No optimization setting or
SLAM algorithm is changed to work around compilation resources.

## Completed acceptance (2026-10-22)

Build completed successfully after the isolated Optimizer build. Lifecycle,
B4 state-machine and B2 candidate tests all exited 0. No Git commit was made.

Result directory:
`results/frontend_stage_validation/step5_shared_track_20261022_153908/`

Both 100-frame runs passed before the complete runs:

| Sequence | Input / camera poses | Keyframes | Exit | RECENTLY_LOST / LOST | Visible instance frames |
|---|---|---|---|---|---|
| sitting_static | 680 / 680 | 12 | 0 | 0 / 0 | 1360 |
| walking_xyz | 827 / 827 | 276 | 0 | 0 / 0 | 1020 |

`shared_track_audit.json` passed all four cases. Every pre-Frame ID agreed
with its B2/B3/B4 records. Maximum updates per frame/track was exactly 1;
all visible observations had missed_frames=0. Complete sequences included
981 unreliable observations (250 sitting + 731 walking); none changed
S/P/D or Ns/Nd/Nr. All applied policies remained NONE_EVIDENCE_ONLY.

Walking recorded 14 gap/recovery events retaining the same ID. For example,
track 3 recovered at frame 63 after 14 missing frames; tracks 5 and 9 each
had a seven-frame gap with their IDs retained. These are consistency checks
of the association/state records, not independently labeled physical-ID
ground truth. The synthetic boundary test additionally confirms recovery
after exactly 15 missing frames and fresh-ID/P creation after 16.

This is a functional lifecycle test, not new ATE/RPE or GUI validation.
The source archive saved at run start predates the audit script's final
parameterized TTL/freeze checks; `audit_shared_track_updates_final.py` in
the result directory preserves the exact script used for the final audit.
