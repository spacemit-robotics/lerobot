#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/thirdparty/lerobot/examples/onnx_inference/${SROBOTIS_TEST_NAME:-smolvla-k3-performance}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/smolvla_k3_performance.log"
summary_file="$log_dir/smolvla_k3_performance_summary.txt"
download_dir="${SMOLVLA_MODEL_CACHE:-/tmp/smolvla_models}"

smolvla_base_url="https://archive.spacemit.com/spacemit-ai/model_zoo/vla/smolvla"
pytorch_archive="$download_dir/so101_smolvla_pick_green_cube_2cam.tar.gz"
fp32_archive="$download_dir/so101_smolvla_pick_green_cube_2cam_100k_fp32.tar.gz"
fp16_archive="$download_dir/so101_smolvla_pick_green_cube_2cam_100k_fp16_surgeried.tar.gz"
ort_archive="$download_dir/spacemit-ort.riscv64.2.0.4_yyx.tar.gz"
ort_dir="${SPACEMIT_ORT_DIR:-$HOME/spacemit-ort.riscv64.2.0.4_yyx}"
calibration="${SMOLVLA_CALIBRATION:-$HOME/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json}"
task="${SMOLVLA_TASK:-Place the green cube into the box}"
iters="${SMOLVLA_PERF_ITERS:-3}"
warmup="${SMOLVLA_PERF_WARMUP:-1}"
max_avg_ms="${SMOLVLA_PERF_MAX_AVG_MS:-1600}"

mkdir -p "$log_dir" "$download_dir"

download_if_missing() {
    local url="$1"
    local out="$2"
    if [[ -s "$out" ]]; then
        echo "[info] reuse archive: $out"
        return
    fi
    echo "[info] download: $url"
    curl -fL --retry 3 --continue-at - "$url" -o "$out"
}

ensure_models() {
    cd "$module_root"
    mkdir -p models/pytorch models/onnx

    download_if_missing \
        "$smolvla_base_url/models/pytorch/so101_smolvla_pick_green_cube_2cam.tar.gz" \
        "$pytorch_archive"
    download_if_missing \
        "$smolvla_base_url/models/onnx/so101_smolvla_pick_green_cube_2cam_100k_fp32.tar.gz" \
        "$fp32_archive"
    download_if_missing \
        "$smolvla_base_url/models/onnx/so101_smolvla_pick_green_cube_2cam_100k_fp16_surgeried.tar.gz" \
        "$fp16_archive"
    download_if_missing \
        "$smolvla_base_url/spacemit-ort-sdk/spacemit-ort.riscv64.2.0.4_yyx.tar.gz" \
        "$ort_archive"

    [[ -d models/pytorch/so101_smolvla_pick_green_cube_2cam ]] || \
        tar -xzf "$pytorch_archive" -C models/pytorch
    [[ -d models/onnx/so101_smolvla_pick_green_cube_2cam_100k_fp32 ]] || \
        tar -xzf "$fp32_archive" -C models/onnx
    [[ -d models/onnx/so101_smolvla_pick_green_cube_2cam_100k_fp16_surgeried ]] || \
        tar -xzf "$fp16_archive" -C models/onnx
    [[ -d "$ort_dir" ]] || tar -xzf "$ort_archive" -C "$HOME"

    ln -sfn so101_smolvla_pick_green_cube_2cam_100k_fp32 models/onnx/smolvla-fp32
    ln -sfn so101_smolvla_pick_green_cube_2cam_100k_fp16_surgeried models/onnx/smolvla-fp16-surgeried
    mkdir -p models/pytorch/smolvla/checkpoints/100000
    ln -sfn ../../../so101_smolvla_pick_green_cube_2cam/checkpoints/100000/pretrained_model \
        models/pytorch/smolvla/checkpoints/100000/pretrained_model
}

ensure_runtime() {
    cd "$module_root"
    if [[ -s models/onnx/smolvla_runtime.txt ]]; then
        echo "[info] reuse runtime metadata: models/onnx/smolvla_runtime.txt"
        return
    fi
    if [[ ! -f "$calibration" ]]; then
        echo "[error] missing calibration file: $calibration"
        echo "[error] set SMOLVLA_CALIBRATION or run lerobot-calibrate first"
        exit 1
    fi
    python3 tools/export_smolvla_runtime.py \
        --checkpoint models/pytorch/smolvla/checkpoints/100000/pretrained_model \
        --output models/onnx/smolvla_runtime.txt \
        --task "$task" \
        --num-cameras 2 \
        --calibration "$calibration"
}

parse_average() {
    python3 - "$log_file" "$summary_file" "$max_avg_ms" "$warmup" <<'PY'
from __future__ import annotations

import re
import sys
from pathlib import Path

log_path = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
max_avg_ms = float(sys.argv[3])
warmup = int(sys.argv[4])
text = log_path.read_text(encoding="utf-8", errors="replace")
samples = [
    float(m.group(1))
    for m in re.finditer(r"\[(?:iter|warmup)\s+\d+\]\s+infer=([0-9.]+)\s+ms", text)
]
if not samples:
    print("[error] no infer samples found", file=sys.stderr)
    raise SystemExit(1)
if len(samples) <= warmup:
    print(f"[error] only {len(samples)} infer samples found; warmup={warmup}", file=sys.stderr)
    raise SystemExit(1)
samples = samples[warmup:]
avg = sum(samples) / len(samples)
summary = (
    f"samples={len(samples)}\n"
    f"avg_ms={avg:.3f}\n"
    f"min_ms={min(samples):.3f}\n"
    f"max_ms={max(samples):.3f}\n"
    f"threshold_ms={max_avg_ms:.3f}\n"
)
summary_path.write_text(summary, encoding="utf-8")
print(summary, end="")
if avg > max_avg_ms:
    print(f"[error] avg_ms {avg:.3f} exceeds threshold {max_avg_ms:.3f}", file=sys.stderr)
    raise SystemExit(1)
PY
}

run_benchmark() {
    echo "[info] module_root=$module_root"
    echo "[info] artifact_dir=$artifact_dir"
    echo "[info] SMOLVLA_PERF_ITERS=$iters"
    echo "[info] SMOLVLA_PERF_WARMUP=$warmup"
    echo "[info] SMOLVLA_PERF_MAX_AVG_MS=$max_avg_ms"
    echo "[info] SPACEMIT_ORT_DIR=$ort_dir"
    echo "[info] benchmark_mode=synthetic-inputs"

    ensure_models
    ensure_runtime

    cd "$module_root/cpp"
    ./build_smolvla_robot_cpp.sh EP204

    local total_iters=$((warmup + iters))
    DRY_RUN=1 \
    SPACEMIT_ORT_DIR="$ort_dir" \
    WARMUP="$total_iters" \
    ./run_smolvla_robot_pipeline.sh \
        --warmup-only \
        --n-action-steps 50
}

run_benchmark 2>&1 | tee "$log_file"
parse_average 2>&1 | tee -a "$log_file"
echo "PERF_OK" | tee -a "$log_file"

grep -q "PERF_OK" "$log_file"
