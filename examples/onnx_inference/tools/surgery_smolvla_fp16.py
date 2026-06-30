#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Create the K3-ready SmolVLA fp16_surgeried model directory."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def _run(script: str, src: Path, dst: Path, no_smoke: bool) -> None:
    cmd = [sys.executable, str(SCRIPT_DIR / script), "--in", str(src), "--out", str(dst)]
    if no_smoke:
        cmd.append("--no-smoke")
    print("[surgery] " + " ".join(cmd))
    subprocess.run(cmd, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True, help="fp16 ONNX directory")
    parser.add_argument("--output-dir", type=Path, required=True, help="fp16_surgeried output directory")
    parser.add_argument(
        "--no-smoke", action="store_true", help="Skip ONNX Runtime smoke tests in surgery scripts"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    for name in ["vision_encoder.onnx", "connector.onnx"]:
        src = args.input_dir / name
        if not src.exists():
            raise FileNotFoundError(src)
        shutil.copy2(src, args.output_dir / name)

    prefill_tmp = args.output_dir / "prefill_lm.rms_fp32.onnx"
    _run("surgery_rms_fp32.py", args.input_dir / "prefill_lm.onnx", prefill_tmp, args.no_smoke)
    _run("surgery_scatternd.py", prefill_tmp, args.output_dir / "prefill_lm.onnx", args.no_smoke)
    prefill_tmp.unlink(missing_ok=True)

    denoise_tmp = args.output_dir / "denoise_step.rms_fp32.onnx"
    _run("surgery_rms_fp32.py", args.input_dir / "denoise_step.onnx", denoise_tmp, args.no_smoke)
    _run("surgery_scatternd.py", denoise_tmp, args.output_dir / "denoise_step.onnx", args.no_smoke)
    denoise_tmp.unlink(missing_ok=True)

    export_config = args.input_dir / "export_config.json"
    if export_config.exists():
        shutil.copy2(export_config, args.output_dir / "export_config.json")
    print(f"[surgery] wrote {args.output_dir}")


if __name__ == "__main__":
    main()
