#!/usr/bin/env bash

set -euo pipefail

# Example:
# SPINETORCH_SITE_PACKAGES=/path/to/spinetorch/site-packages \
# ./examples/spine-torch-deploy/run_lerobot_fp16_a100x8.sh lerobot-record ... \
#     --policy.path=/path/to/pretrained_model_fp16

if [[ $# -eq 0 ]]; then
    echo "Usage: $0 lerobot-record [arguments...]" >&2
    exit 2
fi

if [[ -n "${SPINETORCH_SITE_PACKAGES:-}" ]]; then
    export PYTHONPATH="${SPINETORCH_SITE_PACKAGES}:${PYTHONPATH:-}"
fi

# Use eight OpenMP worker threads for model inference.
export OMP_NUM_THREADS=8

# Pin SpineDNN intra-op workers to K3 AI Core logical CPUs 8 through 15.
export SPINEDNN_INTRA_THREAD_AFFINITY="8;9;10;11;12;13;14;15"

# Keep this launcher on the AI Core path instead of enabling common CPU cores.
unset USE_COMMON_CORE

# Set the convolution tiling optimization strength used by SpineDNN.
export SPINEDNN_CONV_TILING_STRENGTH=900

# Enable the cached SpineDNN execution context for the ACT vision backbone.
export SPINEDNN_TORCH_PLUGIN_VISUAL_CONTEXT=1

# Apply the visual context optimization to the complete vision backbone.
export SPINEDNN_TORCH_PLUGIN_VISUAL_SCOPE=full

# Use the NHWC layout in the optimized visual execution path.
export SPINEDNN_TORCH_PLUGIN_VISUAL_LAYOUT=nhwc

if [[ "$(basename -- "$1")" == "lerobot-record" ]]; then
    shift
    SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
    exec python "${SCRIPT_DIR}/lerobot_record_fp16.py" "$@"
fi

exec "$@"
