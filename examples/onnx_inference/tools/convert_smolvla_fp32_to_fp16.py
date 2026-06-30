#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Convert a SmolVLA 4-model fp32 ONNX directory to fp16 with xslim."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

MODEL_NAMES = ["vision_encoder", "connector", "prefill_lm", "denoise_step"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True, help="fp32 ONNX directory")
    parser.add_argument("--output-dir", type=Path, required=True, help="fp16 ONNX output directory")
    parser.add_argument("--xslim-module", default="xslim", help="Python module name for xslim")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    for name in MODEL_NAMES:
        src = args.input_dir / f"{name}.onnx"
        dst = args.output_dir / f"{name}.onnx"
        if not src.exists():
            raise FileNotFoundError(src)
        cmd = [sys.executable, "-m", args.xslim_module, "-i", str(src), "-o", str(dst), "--fp16"]
        print("[fp16] " + " ".join(cmd))
        subprocess.run(cmd, check=True)

    export_config = args.input_dir / "export_config.json"
    if export_config.exists():
        shutil.copy2(export_config, args.output_dir / "export_config.json")
    print(f"[fp16] wrote {args.output_dir}")


if __name__ == "__main__":
    main()
