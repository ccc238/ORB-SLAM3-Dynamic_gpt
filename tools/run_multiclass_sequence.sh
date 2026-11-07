#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 3 || $# -gt 4 ]]; then
    echo "Usage: bash tools/run_multiclass_sequence.sh person|multi|evidence DATASET ASSOCIATIONS [MAX_FRAMES=0]" >&2
    exit 2
fi
task_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
task_build="${task_repo}/build_multiclass"
case "$1" in
    person) task_config=person_only_action ;;
    multi) task_config=selected_classes_action ;;
    evidence) task_config=selected_classes_evidence ;;
    *) echo "Unknown group: $1" >&2; exit 2 ;;
esac
task_dataset=$(realpath -- "$2")
task_association=$(realpath -- "$3")
task_limit=${4:-0}
[[ "$task_limit" =~ ^[0-9]+$ ]] || { echo "MAX_FRAMES must be >= 0" >&2; exit 2; }
task_yaml="${task_repo}/configs/multiclass/rgbd_tum3/${task_config}.yaml"
for task_file in "$task_build/bin/ablation_sequence" "$task_yaml" "$task_association" "$task_repo/Vocabulary/ORBvoc.txt"; do
    [[ -f "$task_file" ]] || { echo "Missing file: $task_file" >&2; exit 2; }
done
[[ -d "$task_dataset" ]] || { echo "Missing dataset: $task_dataset" >&2; exit 2; }
mkdir -p -- "$task_build/runs"
task_run=$(mktemp -d "$task_build/runs/${task_config}_$(basename -- "$task_dataset")_XXXXXX")
cp -- "$task_yaml" "$task_run/effective.yaml"
git -C "$task_repo" rev-parse HEAD > "$task_run/commit.txt"
git -C "$task_repo" status --short > "$task_run/git_status.txt"
git -C "$task_repo" diff --binary > "$task_run/source_changes.patch"
sha256sum "$task_run/effective.yaml" "$task_build/bin/ablation_sequence" "$task_build/lib/libORB_SLAM3.so" > "$task_run/sha256.txt"
export LD_LIBRARY_PATH="$task_build/lib:$task_repo/third_party_ort/onnxruntime-linux-x64-1.17.3/lib:$task_repo/Thirdparty/DBoW2/lib:$task_repo/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"
export QT_QPA_PLATFORM=offscreen LIBGL_ALWAYS_SOFTWARE=1
task_command=("$task_build/bin/ablation_sequence" rgbd "$task_repo/Vocabulary/ORBvoc.txt" "$task_run/effective.yaml" "$task_dataset" "$task_association" "$task_limit" 0 0)
printf '%q ' "${task_command[@]}" > "$task_run/command.txt"
printf '\n' >> "$task_run/command.txt"
echo "Result directory: $task_run"
cd -- "$task_run"
set +e
"${task_command[@]}" 2>&1 | tee run.log
task_status=${PIPESTATUS[0]}
set -e
printf '%s\n' "$task_status" > exit_code.txt
echo "Result directory: $task_run (exit=$task_status)"
exit "$task_status"
