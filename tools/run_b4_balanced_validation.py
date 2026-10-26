#!/usr/bin/env python3
"""Run both complete TUM RGB-D association lists with no interactive Viewer."""
import csv
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import time
from datetime import datetime


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "Examples/RGB-D/TUM3_B4_balanced_validation.yaml"
EXECUTABLE = ROOT / "build_yolo11/bin/ablation_sequence"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stamp():
    return datetime.now().astimezone().isoformat(timespec="seconds")


def record(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def csv_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def run_one(parent, sequence):
    directory = parent / ("fr3_" + sequence)
    directory.mkdir(exist_ok=False)
    dataset = Path("/home/aaa/Datasets/TUM") / ("rgbd_dataset_freiburg3_" + sequence)
    associations = Path("/home/aaa/dynamic_slam_experiments/associations") / ("fr3_" + sequence + "_associations.txt")
    pairs = [line.split() for line in associations.read_text().splitlines()
             if line.strip() and not line.lstrip().startswith("#")]
    for pair in pairs:
        if len(pair) != 4 or not (dataset / pair[1]).is_file() or not (dataset / pair[3]).is_file():
            raise RuntimeError("Missing or invalid RGB-depth pair: " + repr(pair))
    snapshot = directory / CONFIG.name
    shutil.copy2(CONFIG, snapshot)
    for name, command in (
        ("commit.txt", ["git", "rev-parse", "HEAD"]),
        ("git_status.txt", ["git", "status", "--short"]),
        ("tracked_changes.patch", ["git", "diff", "--binary", "HEAD"]),
    ):
        content = subprocess.check_output(command, cwd=ROOT)
        (directory / name).write_bytes(content)
    sources = sorted(list((ROOT / "include").rglob("*.h")) + list((ROOT / "src").rglob("*.cc")) +
                     list((ROOT / "src").rglob("*.cpp")) + [ROOT / "tools/ablation_sequence.cc", Path(__file__).resolve()])
    record(directory / "source_sha256.json", {str(p.relative_to(ROOT)): sha256(p) for p in sources})
    command = [str(EXECUTABLE), "rgbd", str(ROOT / "Vocabulary/ORBvoc.txt"),
               str(snapshot), str(dataset), str(associations), "0", "0", "1"]
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = ":".join([str(ROOT / "build_yolo11/lib"),
        str(ROOT / "Thirdparty/DBoW2/lib"), str(ROOT / "Thirdparty/g2o/lib"),
        str(ROOT / "third_party_ort/onnxruntime-linux-x64-1.17.3/lib"), env.get("LD_LIBRARY_PATH", "")])
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["QT_QPA_PLATFORM"] = "offscreen"
    script = "#!/usr/bin/env bash\nset -euo pipefail\ncd " + shlex.quote(str(directory)) + "\n"
    for key in ("LD_LIBRARY_PATH", "LIBGL_ALWAYS_SOFTWARE", "QT_QPA_PLATFORM"):
        script += "export " + key + "=" + shlex.quote(env[key]) + "\n"
    script += shlex.join(command) + "\n"
    (directory / "command.sh").write_text(script)
    metadata = {"sequence": sequence, "dataset": str(dataset), "association_file": str(associations),
                "association_sha256": sha256(associations), "expected_frames": len(pairs),
                "config_source": str(CONFIG), "config_snapshot": str(snapshot), "config_sha256": sha256(snapshot),
                "executable": str(EXECUTABLE), "executable_sha256": sha256(EXECUTABLE),
                "library_sha256": sha256(ROOT / "build_yolo11/lib/libORB_SLAM3.so"),
                "viewer": False, "max_frames": 0, "pace": True, "started_at": stamp(), "command": command}
    record(directory / "run_metadata.json", metadata)
    print("START " + sequence + " expected=" + str(len(pairs)) + " output=" + str(directory), flush=True)
    start = time.monotonic()
    with (directory / "run.log").open("wb") as log:
        process = subprocess.Popen(command, cwd=directory, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            code = process.wait(timeout=900)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            code = 124
            metadata["timeout_seconds"] = 900
    metadata.update(ended_at=stamp(), duration_seconds=time.monotonic() - start, exit_code=code)
    (directory / "exit_code.txt").write_text(str(code) + "\n")
    (directory / "duration.txt").write_text(str(metadata["duration_seconds"]) + " seconds\n")
    line_counts = {}
    for name in ("CameraTrajectory.txt", "KeyFrameTrajectory.txt"):
        path = directory / name
        line_counts[name] = sum(1 for line in path.read_text().splitlines() if line.strip() and not line.startswith("#")) if path.is_file() else 0
    (directory / "trajectory_line_count.txt").write_text("".join(str(n) + " " + name + "\n" for name, n in line_counts.items()))
    metadata["trajectory_line_counts"] = line_counts
    streams = {"b2": list(directory.glob("b2_evidence_*/b2_instance_evidence.csv")),
               "b3": list(directory.glob("b3_fusion_summary_*.csv")),
               "b4": list(directory.glob("b4_state_summary_*.csv"))}
    metadata["csvs"] = {key: [str(p) for p in paths] for key, paths in streams.items()}
    coverage = {}
    for key, paths in streams.items():
        records = csv_rows(paths[0]) if len(paths) == 1 else []
        coverage[key] = {"instance_rows": len(records), "unique_observed_frame_ids": len({r["frame_id"] for r in records})}
    metadata["csv_coverage"] = coverage
    timing = directory / "tracking_timing.yaml"
    statistics = {}
    if timing.is_file():
        for line in timing.read_text().splitlines():
            if ":" in line:
                key, value = line.split(":", 1)
                statistics[key.strip()] = value.strip()
    metadata["tracking_statistics"] = statistics
    metadata["complete"] = (code == 0 and all(line_counts.values()) and
                            int(statistics.get("frames", "0")) == len(pairs) and
                            statistics.get("preview_user_stopped") == "0" and
                            all(len(paths) == 1 and paths[0].stat().st_size > 0 for paths in streams.values()))
    record(directory / "run_metadata.json", metadata)
    print("END " + sequence + " " + json.dumps(metadata, ensure_ascii=False), flush=True)
    if not metadata["complete"]:
        raise RuntimeError("Run incomplete; preserved all artifacts in " + str(directory))
    return metadata


def main():
    parent = ROOT / "results/B4_balanced_validation" / datetime.now().strftime("%Y%m%d_%H%M%S")
    parent.mkdir(parents=True, exist_ok=False)
    print("BATCH " + str(parent), flush=True)
    results = []
    for name in ("sitting_static", "walking_xyz"):
        results.append(run_one(parent, name))
        record(parent / "batch_summary.json", results)
    print("COMPLETE " + str(parent), flush=True)


if __name__ == "__main__":
    main()
