#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/thirdparty/lerobot/examples/onnx_inference/${SROBOTIS_TEST_NAME:-onnx-inference-static-functional}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/onnx_inference_static_functional.log"

mkdir -p "$log_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] artifact_dir=$artifact_dir"

    required_files=(
        README.md
        tools/README.md
        cpp/README.md
        cpp/build_smolvla_robot_cpp.sh
        cpp/run_smolvla_robot_pipeline.sh
        tools/surgery_smolvla_fp16.py
        tools/surgery_vision_self_attn_nhwc.py
        tools/compare_smolvla_onnx.py
        test/test_act_k3_performance.sh
        test/test_smolvla_k3_performance.sh
    )
    for rel in "${required_files[@]}"; do
        test -f "$module_root/$rel"
        echo "[check] exists: $rel"
    done

    bash -n "$module_root/cpp/build_smolvla_robot_cpp.sh"
    bash -n "$module_root/cpp/run_smolvla_robot_pipeline.sh"
    bash -n "$module_root/test/test_act_k3_performance.sh"
    bash -n "$module_root/test/test_smolvla_k3_performance.sh"

    python3 -m py_compile \
        "$module_root/tools/surgery_smolvla_fp16.py" \
        "$module_root/tools/surgery_vision_self_attn_nhwc.py" \
        "$module_root/tools/compare_smolvla_onnx.py" \
        "$module_root/tools/export_smolvla_runtime.py" \
        "$module_root/python/smolvla_evaluate.py"

    grep -R "spacemit-ort.riscv64.2.0.4_yyx" \
        "$module_root/README.md" \
        "$module_root/tools/README.md" \
        "$module_root/cpp/README.md" \
        "$module_root/cpp/build_smolvla_robot_cpp.sh" \
        "$module_root/cpp/run_smolvla_robot_pipeline.sh" \
        "$module_root/cpp/smolvla_robot/CMakeLists.txt" >/dev/null
    grep -R "VisionSelfAttnNHWC" \
        "$module_root/tools/README.md" \
        "$module_root/tools/surgery_vision_self_attn_nhwc.py" >/dev/null

    if grep -R "EP203\|EP204_REF\|spacemit-ort.riscv64.2.0.3_yyx" \
        "$module_root/README.md" \
        "$module_root/tools/README.md" \
        "$module_root/cpp/README.md" \
        "$module_root/cpp/build_smolvla_robot_cpp.sh" \
        "$module_root/cpp/run_smolvla_robot_pipeline.sh" \
        "$module_root/cpp/smolvla_robot/CMakeLists.txt"; then
        echo "[error] stale EP203 or 2.0.3 reference found"
        exit 1
    fi

    echo "STATIC_OK"
} | tee "$log_file"

grep -q "STATIC_OK" "$log_file"
