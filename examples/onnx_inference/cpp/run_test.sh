#!/usr/bin/env bash
# Run onnxruntime_perf_test with the SpaceMIT EP knobs used by ACT models.
set -euo pipefail

usage() {
    echo "Usage:"
    echo "  $0 <use_self_ep:true|false> <model_path> <intra_threads>"
}

if [ $# -ne 3 ]; then
    usage
    exit 1
fi

USE_SELF_EP="$1"
MODEL_PATH="$2"
INTRA_THREADS="$3"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ORT_DIR="${SPACEMIT_ORT_DIR:-}"
if [ -n "$ORT_DIR" ]; then
    PERF_BIN="${ORT_DIR}/bin/onnxruntime_perf_test"
else
    PERF_BIN="$(command -v onnxruntime_perf_test || true)"
fi
PERF_ARGS=("$MODEL_PATH" -x "$INTRA_THREADS" -S 1 -s -I -c 1 -r 5 -e spacemit)

case "$USE_SELF_EP" in
    true|1) USE_SELF_EP=true ;;
    false|0) USE_SELF_EP=false ;;
    *)
        usage
        echo "Error: use_self_ep must be true, false, 1, or 0." >&2
        exit 1
        ;;
esac

if [ ! -x "$PERF_BIN" ]; then
    echo "Error: onnxruntime_perf_test not found." >&2
    echo "Install spacemit-onnxruntime or set SPACEMIT_ORT_DIR=/path/to/sdk." >&2
    exit 1
fi

if [ ! -f "$MODEL_PATH" ]; then
    echo "Error: model file not found: $MODEL_PATH" >&2
    exit 1
fi

if ! [[ "$INTRA_THREADS" =~ ^[0-9]+$ ]] || [ "$INTRA_THREADS" -lt 1 ]; then
    echo "Error: intra_threads must be a positive integer." >&2
    exit 1
fi

if [ -n "$ORT_DIR" ]; then
    export LD_LIBRARY_PATH="${ORT_DIR}/lib:${LD_LIBRARY_PATH:-}"
fi

if [ "$USE_SELF_EP" = true ]; then
    env SPACEMIT_EP_PWCONV_INT8_USE=1 \
    SPACEMIT_EP_CONVTRANSPOSE_4X4_FP16_USE=1 \
    SPACEMIT_EP_CONVTRANSPOSE_4X4_FP16_LOG=1 \
    SPACEMIT_EP_CONVTRANSPOSE_3X3_FP16_USE=1 \
    ORT_CPU_EP_DIV_FP32_RVV_USE=1 \
    SPACEMIT_EP_CONCAT_FP16_RVV_USE=1 \
    SPACEMIT_EP_GATHER_FP16_RVV_USE=1 \
    SPACEMIT_EP_MUL_FP16_RVV_USE=1 \
    SPACEMIT_EP_ERF_FP16_RVV_USE=1 \
    SPACEMIT_EP_SIN_FP16_RVV_USE=1 \
    SPACEMIT_EP_COS_FP16_RVV_USE=1 \
    SPACEMIT_EP_WHERE_FP16_USE=1 \
    SPACEMIT_EP_POW_FP16_RVV_USE=1 \
    SPACEMIT_EP_REDUCEMEAN_FP16_RVV_USE=1 \
    SPACEMIT_EP_SOFTMAX_FP16_USE=1 \
    SPACEMIT_EP_CONV3D_RVV_USE=1 \
    SPACEMIT_EP_SOFTMAX_FP32_USE=1 \
    SPACEMIT_EP_REDUCESUM_FP32_USE=1 \
    SPACEMIT_EP_SEPDWCONV_USE=1 \
    SPACEMIT_EP_DWCONV_3X3_FP32_USE=1 \
    SPACEMIT_EP_DWCONV_3X3_S2_FP32_USE=1 \
    SPACEMIT_EP_ADD_QDQ_RVV_USE=1 \
    SPACEMIT_EP_REDUCEMEAN_QDQ_RVV_USE=1 \
    SPACEMIT_EP_SOFTMAX_QDQ_INT8_USE=1 \
    SPACEMIT_EP_GELU_QDQ_INT8_USE=1 \
    SPACEMIT_EP_LAYERNORM_QDQ_INT8_USE=1 \
    SPACEMIT_EP_CONVTRANSPOSE_3X3_USE=1 \
    SPACEMIT_EP_CONVTRANSPOSE_4X4_USE=1 \
    SPACEMIT_EP_POW2_REDUCEMEAN_USE=1 \
    "$PERF_BIN" "${PERF_ARGS[@]}"
else
    export SPACEMIT_EP_DEBUG_PROFILE="ref_ep"
    "$PERF_BIN" "${PERF_ARGS[@]}" -p "ref_ep"
fi
