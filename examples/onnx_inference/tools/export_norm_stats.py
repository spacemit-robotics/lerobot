#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Export ACT normalization stats + SO-101 calibration for the C++ runtime.

The ACT ONNX graph does NOT contain input normalization, output
unnormalization, or the per-motor lerobot calibration (raw encoder step <->
degrees). The Python script reads those from the checkpoint safetensors and the
lerobot calibration JSON directly; this tool dumps the exact same numbers into a
flat, trivially-parseable text file so the C++ twin (cpp/act_evaluate.cpp)
stays byte-for-byte semantically identical.

Output format (act_norm_stats.txt) — one record per line, first token is a key:
  cam_names top wrist
  image_keys observation.images.top observation.images.wrist
  img_chw 3 480 640
  state_dim 6
  action_dim 6
  chunk_size 100
  n_action_steps 100
  motor_order shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper
  state_mean <6 floats>
  state_std  <6 floats>
  action_mean <6 floats>
  action_std  <6 floats>
  image_mean.top <3 floats>
  image_std.top  <3 floats>
  ...
  calib shoulder_pan id 1 drive_mode 0 range_min 662 range_max 3246 norm_mode DEGREES
  calib gripper      id 6 drive_mode 0 range_min 2020 range_max 3487 norm_mode RANGE_0_100

Usage:
  python tools/export_norm_stats.py \
      --checkpoint models/pytorch/<run>/checkpoints/100000/pretrained_model \
            --calibration ~/.cache/huggingface/lerobot/calibration/robots/so_follower/robot.json \
            --output models/onnx/act-fp32/act_norm_stats.txt
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

# Same fixed motor order as act_evaluate.py (observation.state / action).
MOTOR_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
# norm mode per motor: body joints use DEGREES (use_degrees=True, the lerobot
# SO-101 default), the gripper uses RANGE_0_100. See so_follower.py.
NORM_MODE = dict.fromkeys(MOTOR_ORDER, "DEGREES")
NORM_MODE["gripper"] = "RANGE_0_100"


def _load_safetensors_np(path: Path) -> dict[str, np.ndarray]:
    from safetensors import safe_open

    out: dict[str, np.ndarray] = {}
    with safe_open(str(path), framework="np") as sf:
        for k in sf:
            out[k] = sf.get_tensor(k)
    return out


def _fmt(arr: np.ndarray) -> str:
    return " ".join(f"{float(v):.8g}" for v in np.asarray(arr).flatten())


def read_meta(checkpoint: Path) -> dict:
    cfg = json.loads((checkpoint / "config.json").read_text(encoding="utf-8"))
    image_keys = [k for k, v in cfg["input_features"].items() if v["type"] == "VISUAL"]
    cam_names = [k.split("observation.images.")[-1] for k in image_keys]
    c, h, w = cfg["input_features"][image_keys[0]]["shape"]
    return {
        "image_keys": image_keys,
        "cam_names": cam_names,
        "state_dim": cfg["input_features"]["observation.state"]["shape"][0],
        "action_dim": cfg["output_features"]["action"]["shape"][0],
        "chunk_size": cfg["chunk_size"],
        "n_action_steps": cfg["n_action_steps"],
        "img_c": c,
        "img_h": h,
        "img_w": w,
    }


def build_lines(checkpoint: Path, calibration: Path | None) -> list[str]:
    meta = read_meta(checkpoint)
    pre = _load_safetensors_np(checkpoint / "policy_preprocessor_step_3_normalizer_processor.safetensors")
    post = _load_safetensors_np(checkpoint / "policy_postprocessor_step_0_unnormalizer_processor.safetensors")

    lines: list[str] = ["# ACT norm stats + SO-101 calibration (generated)"]
    lines.append("cam_names " + " ".join(meta["cam_names"]))
    lines.append("image_keys " + " ".join(meta["image_keys"]))
    lines.append(f"img_chw {meta['img_c']} {meta['img_h']} {meta['img_w']}")
    lines.append(f"state_dim {meta['state_dim']}")
    lines.append(f"action_dim {meta['action_dim']}")
    lines.append(f"chunk_size {meta['chunk_size']}")
    lines.append(f"n_action_steps {meta['n_action_steps']}")
    lines.append("motor_order " + " ".join(MOTOR_ORDER))

    lines.append("state_mean " + _fmt(pre["observation.state.mean"]))
    lines.append("state_std " + _fmt(pre["observation.state.std"]))
    lines.append("action_mean " + _fmt(post["action.mean"]))
    lines.append("action_std " + _fmt(post["action.std"]))
    for key, name in zip(meta["image_keys"], meta["cam_names"], strict=True):
        lines.append(f"image_mean.{name} " + _fmt(pre[f"{key}.mean"]))
        lines.append(f"image_std.{name} " + _fmt(pre[f"{key}.std"]))

    if calibration is not None:
        calib = json.loads(calibration.read_text(encoding="utf-8"))
        for motor in MOTOR_ORDER:
            if motor not in calib:
                raise SystemExit(f"calibration missing motor '{motor}' in {calibration}")
            c = calib[motor]
            lines.append(
                f"calib {motor} id {c['id']} drive_mode {c['drive_mode']} "
                f"range_min {c['range_min']} range_max {c['range_max']} "
                f"norm_mode {NORM_MODE[motor]}"
            )
    else:
        lines.append("# (no calibration provided; C++ direct motor control needs it)")
    return lines


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--checkpoint",
        type=Path,
        required=True,
        help="pretrained_model dir (config.json + normalizer safetensors)",
    )
    default_cal = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json"
    p.add_argument(
        "--calibration",
        type=Path,
        default=default_cal,
        help=f"lerobot SO-101 calibration JSON (default: {default_cal})",
    )
    p.add_argument("--output", type=Path, required=True, help="Output .txt path")
    p.add_argument(
        "--no-calibration",
        action="store_true",
        help="Skip calibration (stats only; C++ direct motor control will refuse to run)",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    calibration = None if args.no_calibration else args.calibration
    if calibration is not None and not calibration.exists():
        print(
            f"[export_norm_stats] WARNING: calibration not found: {calibration}\n"
            "  writing stats only; pass --no-calibration to silence, or fix --calibration."
        )
        calibration = None

    lines = build_lines(args.checkpoint, calibration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(f"[export_norm_stats] wrote {args.output} ({len(lines)} lines, calibration={'yes' if calibration else 'NO'})")


if __name__ == "__main__":
    main()
