#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  cpp/run_smolvla_robot_pipeline.sh [args passed to smolvla_robot_pipeline]

Default configuration:
  --model-dir models/onnx/smolvla-fp16-surgeried
  --runtime models/onnx/smolvla_runtime.txt
  --port /dev/ttyACM0
  --camera top=15 --camera wrist=13
  --use-spacemit-ep --ep-threads 8 --ep-affinity "8;9;10;11;12;13;14;15"
  --denoise-steps 10 --episode-time 60 --print-actions --global-ep-pool
  --spacemit-ort-dir ~/spacemit-ort.riscv64.2.0.4_yyx

Camera mapping examples:
  --camera top=15 --camera wrist=13
  --camera camera1=15 --camera camera2=13 --camera camera3=5

Examples:
  cpp/run_smolvla_robot_pipeline.sh --max-iters 1
  cpp/run_smolvla_robot_pipeline.sh --camera top=3 --camera wrist=1 --dry-run --max-iters 1
  cpp/run_smolvla_robot_pipeline.sh --camera camera1=3 --camera camera2=1 --camera camera3=5 --dry-run --max-iters 1
  cpp/run_smolvla_robot_pipeline.sh --dry-run --no-motors --max-iters 1
EOF
}

model_dir_arg="${MODEL_DIR:-models/onnx/smolvla-fp16-surgeried}"
runtime_arg="${RUNTIME:-models/onnx/smolvla_runtime.txt}"
port_arg="${PORT:-/dev/ttyACM0}"
spacemit_ort_dir_arg="${SPACEMIT_ORT_DIR:-$HOME/spacemit-ort.riscv64.2.0.4_yyx}"
ep_threads_arg="${EP_THREADS:-8}"
ep_affinity_arg="${EP_AFFINITY:-8;9;10;11;12;13;14;15}"
denoise_steps_arg="${DENOISE_STEPS:-10}"
episode_time_arg="${EPISODE_TIME:-60}"
n_action_steps_arg="${N_ACTION_STEPS:-}"
warmup_arg="${WARMUP:-}"
camera_arg="${camera:-${CAMERA:-}}"
camera_specs=()
passthrough_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --model-dir)
      if [[ $# -lt 2 ]]; then
        echo "--model-dir requires a value" >&2
        exit 1
      fi
      model_dir_arg="$2"
      shift 2
      ;;
    --model-dir=*)
      model_dir_arg="${1#--model-dir=}"
      shift
      ;;
    --runtime)
      if [[ $# -lt 2 ]]; then
        echo "--runtime requires a value" >&2
        exit 1
      fi
      runtime_arg="$2"
      shift 2
      ;;
    --runtime=*)
      runtime_arg="${1#--runtime=}"
      shift
      ;;
    --port)
      if [[ $# -lt 2 ]]; then
        echo "--port requires a value" >&2
        exit 1
      fi
      port_arg="$2"
      shift 2
      ;;
    --port=*)
      port_arg="${1#--port=}"
      shift
      ;;
    --spacemit-ort-dir)
      if [[ $# -lt 2 ]]; then
        echo "--spacemit-ort-dir requires a value" >&2
        exit 1
      fi
      spacemit_ort_dir_arg="$2"
      shift 2
      ;;
    --spacemit-ort-dir=*)
      spacemit_ort_dir_arg="${1#--spacemit-ort-dir=}"
      shift
      ;;
    --ep-threads)
      if [[ $# -lt 2 ]]; then
        echo "--ep-threads requires a value" >&2
        exit 1
      fi
      ep_threads_arg="$2"
      shift 2
      ;;
    --ep-threads=*)
      ep_threads_arg="${1#--ep-threads=}"
      shift
      ;;
    --ep-affinity)
      if [[ $# -lt 2 ]]; then
        echo "--ep-affinity requires a value" >&2
        exit 1
      fi
      ep_affinity_arg="$2"
      shift 2
      ;;
    --ep-affinity=*)
      ep_affinity_arg="${1#--ep-affinity=}"
      shift
      ;;
    --denoise-steps)
      if [[ $# -lt 2 ]]; then
        echo "--denoise-steps requires a value" >&2
        exit 1
      fi
      denoise_steps_arg="$2"
      shift 2
      ;;
    --denoise-steps=*)
      denoise_steps_arg="${1#--denoise-steps=}"
      shift
      ;;
    --episode-time)
      if [[ $# -lt 2 ]]; then
        echo "--episode-time requires a value" >&2
        exit 1
      fi
      episode_time_arg="$2"
      shift 2
      ;;
    --episode-time=*)
      episode_time_arg="${1#--episode-time=}"
      shift
      ;;
    --n-action-steps)
      if [[ $# -lt 2 ]]; then
        echo "--n-action-steps requires a value" >&2
        exit 1
      fi
      n_action_steps_arg="$2"
      shift 2
      ;;
    --n-action-steps=*)
      n_action_steps_arg="${1#--n-action-steps=}"
      shift
      ;;
    --warmup)
      if [[ $# -lt 2 ]]; then
        echo "--warmup requires a value" >&2
        exit 1
      fi
      warmup_arg="$2"
      shift 2
      ;;
    --warmup=*)
      warmup_arg="${1#--warmup=}"
      shift
      ;;
    --camera)
      if [[ $# -lt 2 ]]; then
        echo "--camera requires a value" >&2
        exit 1
      fi
      camera_specs+=("$2")
      shift 2
      ;;
    --camera=*)
      camera_specs+=("${1#--camera=}")
      shift
      ;;
    *)
      passthrough_args+=("$1")
      shift
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BIN="$SCRIPT_DIR/build_smolvla_robot/smolvla_robot_pipeline"

if [[ ! -x "$BIN" ]]; then
  echo "binary not found: $BIN" >&2
  echo "run: cpp/build_smolvla_robot_cpp.sh EP204" >&2
  exit 1
fi

export SPACEMIT_ORT_DIR="$spacemit_ort_dir_arg"
if [[ -d "$SPACEMIT_ORT_DIR/lib" ]]; then
  export LD_LIBRARY_PATH="$SPACEMIT_ORT_DIR/lib:$SPACEMIT_ORT_DIR:${LD_LIBRARY_PATH:-}"
fi

if command -v spacemit-tcm-smi >/dev/null 2>&1; then
  spacemit-tcm-smi -c 2>/dev/null || true
fi

# Use the verified fp16_surgeried SpaceMIT EP environment for SmolVLA.
# The C++ source has its own defaults, so disable them and set this group here.
export SMOLVLA_SKIP_EP_ENV_DEFAULTS=1
export SPACEMIT_EP_PWCONV_INT8_USE=1
export SPACEMIT_EP_CONVTRANSPOSE_4X4_FP16_USE=1
export SPACEMIT_EP_CONVTRANSPOSE_4X4_FP16_LOG=1
export SPACEMIT_EP_CONVTRANSPOSE_3X3_FP16_USE=1
export ORT_CPU_EP_DIV_FP32_RVV_USE=1
export SPACEMIT_EP_CONCAT_FP16_RVV_USE=1
export SPACEMIT_EP_GATHER_FP16_RVV_USE=1
export SPACEMIT_EP_MUL_FP16_RVV_USE=1
export SPACEMIT_EP_ERF_FP16_RVV_USE=1
export SPACEMIT_EP_SIN_FP16_RVV_USE=1
export SPACEMIT_EP_COS_FP16_RVV_USE=1
export SPACEMIT_EP_WHERE_FP16_USE=1
export SPACEMIT_EP_POW_FP16_RVV_USE=0
export SPACEMIT_EP_REDUCEMEAN_FP16_RVV_USE=1
export SPACEMIT_EP_SOFTMAX_FP16_USE=0
export SPACEMIT_EP_CONV3D_RVV_USE=1
export SPACEMIT_EP_REDUCESUM_FP32_USE=1
export SPACEMIT_EP_SEPDWCONV_USE=1
export SPACEMIT_EP_DWCONV_3X3_FP32_USE=1
export SPACEMIT_EP_DWCONV_3X3_S2_FP32_USE=1
export SPACEMIT_EP_ADD_QDQ_RVV_USE=1
export SPACEMIT_EP_REDUCEMEAN_QDQ_RVV_USE=1
export SPACEMIT_EP_SOFTMAX_QDQ_INT8_USE=1
export SPACEMIT_EP_GELU_QDQ_INT8_USE=1
export SPACEMIT_EP_LAYERNORM_QDQ_INT8_USE=1
export SPACEMIT_EP_CONVTRANSPOSE_3X3_USE=1
export SPACEMIT_EP_CONVTRANSPOSE_4X4_USE=1
export SPACEMIT_EP_POW2_REDUCEMEAN_USE=0

cd "$ROOT_DIR"

extra_args=()
if [[ -n "$n_action_steps_arg" ]]; then
  extra_args+=(--n-action-steps "$n_action_steps_arg")
fi
if [[ "${dry_run:-${DRY_RUN:-0}}" == "1" ]]; then
  extra_args+=(--dry-run --no-motors)
fi
if [[ -n "$warmup_arg" && "$warmup_arg" != "0" ]]; then
  extra_args+=(--warmup "$warmup_arg")
fi
if [[ "${per_camera_vision:-${PER_CAMERA_VISION:-1}}" == "1" ]]; then
  extra_args+=(--per-camera-vision)
fi
camera_args=()
if [[ ${#camera_specs[@]} -gt 0 ]]; then
  for camera_spec in "${camera_specs[@]}"; do
    camera_args+=(--camera "$camera_spec")
  done
elif [[ -n "$camera_arg" ]]; then
  env_camera_specs="${camera_arg//,/ }"
  for camera_spec in $env_camera_specs; do
    camera_args+=(--camera "$camera_spec")
  done
else
  camera_args+=(
    --camera "top=${top_cam:-${TOP_CAM:-15}}"
    --camera "wrist=${wrist_cam:-${WRIST_CAM:-13}}"
  )
fi

exec "$BIN" \
  --model-dir "$model_dir_arg" \
  --runtime "$runtime_arg" \
  --port "$port_arg" \
  "${camera_args[@]}" \
  --use-spacemit-ep \
  --ep-threads "$ep_threads_arg" \
  --ep-affinity "$ep_affinity_arg" \
  --denoise-steps "$denoise_steps_arg" \
  --episode-time "$episode_time_arg" \
  --print-actions \
  --global-ep-pool \
  "${extra_args[@]}" \
  "${passthrough_args[@]}"
