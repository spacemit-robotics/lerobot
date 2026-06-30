#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: cpp/build_smolvla_robot_cpp.sh [EP]

EP:
  EP203      custom SpaceMIT ORT 2.0.3 under $SPACEMIT_ORT_DIR or ~/spacemit-ort.riscv64.2.0.3_yyx
  EP203_REF  custom reference SpaceMIT ORT under ~/spacemit-ort.riscv64.2.0.3_yyx_ref
  SYSTEM     system onnxruntime under /usr
  clean      remove build directory

Output:
  cpp/build_smolvla_robot/smolvla_robot_pipeline
EOF
}

EP="${1:-EP203}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/cpp/build_smolvla_robot"

case "$EP" in
  help|-h|--help) usage; exit 0 ;;
  clean)
    rm -rf "$BUILD_DIR"
    echo "cleaned: $BUILD_DIR"
    exit 0
    ;;
  EP203|EP203_REF|SYSTEM) ;;
  *)
    echo "unknown EP: $EP" >&2
    usage >&2
    exit 1
    ;;
esac

CMAKE_ARGS=(-DSPACEMIT_EP="$EP")
if [[ -n "${SPACEMIT_ORT_DIR:-}" ]]; then
  CMAKE_ARGS+=("-DSPACEMIT_ORT_DIR=$SPACEMIT_ORT_DIR")
fi

cmake -S "$ROOT_DIR/cpp/smolvla_robot" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" --target smolvla_robot_pipeline -j2

echo
echo "built: $BUILD_DIR/smolvla_robot_pipeline"
