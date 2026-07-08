#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/thirdparty/lerobot/examples/onnx_inference/${SROBOTIS_TEST_NAME:-act-k3-performance}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/act_k3_performance.log"
summary_file="$log_dir/act_k3_performance_summary.txt"
download_dir="${ACT_MODEL_CACHE:-/tmp/act_models}"
extract_dir="$download_dir/extract"
archive="$download_dir/act_models.tar.gz"
ort_dir="${SPACEMIT_ORT_DIR:-$HOME/spacemit-ort.riscv64.2.0.4_yyx}"
calibration="${ACT_CALIBRATION:-$HOME/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json}"
iters="${ACT_PERF_ITERS:-20}"
warmup="${ACT_PERF_WARMUP:-3}"
max_avg_ms="${ACT_PERF_MAX_AVG_MS:-500}"

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
        "https://archive.spacemit.com/spacemit-ai/model_zoo/vla/act/onnx/models.tar.gz" \
        "$archive"

    if [[ ! -d models/pytorch/act || ! -d models/onnx/act-fp32 || ! -d models/onnx/act-int8 ]]; then
        rm -rf "$extract_dir"
        mkdir -p "$extract_dir"
        tar -xzf "$archive" -C "$extract_dir"
        [[ -d models/pytorch/act ]] || cp -a "$extract_dir/models/pytorch/act" models/pytorch/
        [[ -d models/onnx/act-fp32 ]] || cp -a "$extract_dir/models/onnx/act-fp32" models/onnx/
        [[ -d models/onnx/act-int8 ]] || cp -a "$extract_dir/models/onnx/act-int8" models/onnx/
    fi
}

ensure_act_inputs() {
    cd "$module_root"
    local stats="models/onnx/act-fp32/act_norm_stats.txt"
    if [[ ! -s "$stats" ]]; then
        python3 tools/export_norm_stats.py \
            --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
            --calibration "$calibration" \
            --output "$stats"
    fi

    python3 - "$stats" cpp/inputs <<'PY'
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

stats_path = Path(sys.argv[1])
out_dir = Path(sys.argv[2])
out_dir.mkdir(parents=True, exist_ok=True)

cam_names: list[str] = []
img_c, img_h, img_w = 3, 480, 640
state_dim = 6
state_mean = None
state_std = None
image_mean: dict[str, np.ndarray] = {}
image_std: dict[str, np.ndarray] = {}

for line in stats_path.read_text(encoding="utf-8").splitlines():
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    tokens = line.split()
    key, values = tokens[0], tokens[1:]
    if key == "cam_names":
        cam_names = values
    elif key == "img_chw":
        img_c, img_h, img_w = [int(v) for v in values]
    elif key == "state_dim":
        state_dim = int(values[0])
    elif key.startswith("image_mean."):
        image_mean[key.removeprefix("image_mean.")] = np.array([float(v) for v in values], dtype=np.float32).reshape(3, 1, 1)
    elif key.startswith("image_std."):
        image_std[key.removeprefix("image_std.")] = np.array([float(v) for v in values], dtype=np.float32).reshape(3, 1, 1)
    elif key == "state_mean":
        state_mean = np.array([float(v) for v in values], dtype=np.float32)
    elif key == "state_std":
        state_std = np.array([float(v) for v in values], dtype=np.float32)

if not cam_names:
    raise SystemExit("missing cam_names in act_norm_stats.txt")
if state_mean is None or state_std is None:
    raise SystemExit("missing state stats in act_norm_stats.txt")

rng = np.random.default_rng(0)
images = []
for name in cam_names:
    raw = rng.integers(0, 256, size=(img_h, img_w, img_c), dtype=np.uint8)
    chw = np.transpose(raw.astype(np.float32) / 255.0, (2, 0, 1))
    images.append((chw - image_mean[name]) / image_std[name])

state_deg = np.array([5.0, -10.0, 15.0, -5.0, 30.0, 50.0], dtype=np.float32)[:state_dim]

np.save(out_dir / "images.npy", np.stack(images, axis=0).astype(np.float32))
np.save(out_dir / "state_deg.npy", state_deg.astype(np.float32))
print(f"[act-inputs] images.npy cams={len(cam_names)} chw=({img_c},{img_h},{img_w}) state_dim={state_dim}")
PY
}

parse_average() {
    python3 - "$log_file" "$summary_file" "$max_avg_ms" <<'PY'
from __future__ import annotations

import re
import sys
from pathlib import Path

log_path = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
max_avg_ms = float(sys.argv[3])
text = log_path.read_text(encoding="utf-8", errors="replace")
match = re.search(
    r"\[act\]\s+latency mean=([0-9.]+)ms median=([0-9.]+)ms min=([0-9.]+)ms max=([0-9.]+)(?:ms)?",
    text,
)
if match is None:
    print("[error] no ACT latency summary found", file=sys.stderr)
    raise SystemExit(1)
mean, median, min_v, max_v = [float(v) for v in match.groups()]
summary = (
    f"mean_ms={mean:.3f}\n"
    f"median_ms={median:.3f}\n"
    f"min_ms={min_v:.3f}\n"
    f"max_ms={max_v:.3f}\n"
    f"threshold_ms={max_avg_ms:.3f}\n"
)
summary_path.write_text(summary, encoding="utf-8")
print(summary, end="")
if mean > max_avg_ms:
    print(f"[error] mean_ms {mean:.3f} exceeds threshold {max_avg_ms:.3f}", file=sys.stderr)
    raise SystemExit(1)
PY
}

run_benchmark() {
    echo "[info] module_root=$module_root"
    echo "[info] artifact_dir=$artifact_dir"
    echo "[info] ACT_PERF_ITERS=$iters"
    echo "[info] ACT_PERF_WARMUP=$warmup"
    echo "[info] ACT_PERF_MAX_AVG_MS=$max_avg_ms"
    echo "[info] SPACEMIT_ORT_DIR=$ort_dir"

    ensure_models
    ensure_act_inputs

    cd "$module_root/cpp"
    rm -rf build
    mkdir -p build
    cmake -S . -B build -DSPACEMIT_ORT_DIR="$ort_dir"
    cmake --build build -j"$(nproc)" --target act_benchmark

    LD_LIBRARY_PATH="$ort_dir/lib:$ort_dir:${LD_LIBRARY_PATH:-}" \
    ./build/act_benchmark ../models/onnx/act-int8/act.q.onnx \
        --images-npy inputs/images.npy \
        --state-npy inputs/state_deg.npy \
        -s -t 8 -a "8;9;10;11;12;13;14;15" \
        -n "$iters" -w "$warmup"
}

run_benchmark 2>&1 | tee "$log_file"
parse_average 2>&1 | tee -a "$log_file"
echo "PERF_OK" | tee -a "$log_file"

grep -q "PERF_OK" "$log_file"
