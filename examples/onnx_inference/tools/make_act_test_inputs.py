#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Generate deterministic ACT inputs + Python reference output for the C++ twin.

Self-contained: reads everything (cam order, dims, MEAN_STD stats, SO-101
calibration) from act_norm_stats.txt -- the same single source of truth the
C++ twin uses -- so no checkpoint / torch / lerobot import is required.

Writes, into <out-dir>:
  images.npy     (n_cam, 3, H, W) float32  -- ALREADY normalized (MEAN_STD), the
                 exact tensor act_evaluate.cpp --images-npy expects.
  state_deg.npy  (state_dim,)      float32  -- state in lerobot-norm space
                 (degrees / 0..100), the exact tensor act_evaluate.cpp --state-npy
                 expects (the C++ then applies MEAN_STD itself).
  ref_action0.txt                           -- Python ONNX action[0] in lerobot
                 space + its raw-step round-trip, to diff against C++ offline.

Run on any host with onnxruntime (CPU is fine):
  python tools/make_act_test_inputs.py \
        --onnx models/onnx/act-fp32/act.onnx \
            --stats models/onnx/act-fp32/act_norm_stats.txt \
      --out-dir inputs/
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

MAX_STEP = 4095.0


# --------------------------------------------------------------------------- #
# Parse act_norm_stats.txt (mirrors act_evaluate.cpp parse_stats).
# --------------------------------------------------------------------------- #
def parse_stats(path: Path) -> dict:
    s = {
        "cam_names": [],
        "img_c": 3,
        "img_h": 480,
        "img_w": 640,
        "state_dim": 6,
        "action_dim": 6,
        "motor_order": [],
        "image_mean": {},
        "image_std": {},
        "calib": {},
    }
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        toks = line.split()
        key, vals = toks[0], toks[1:]
        if key == "cam_names":
            s["cam_names"] = vals
        elif key == "img_chw":
            s["img_c"], s["img_h"], s["img_w"] = (int(x) for x in vals)
        elif key == "state_dim":
            s["state_dim"] = int(vals[0])
        elif key == "action_dim":
            s["action_dim"] = int(vals[0])
        elif key == "motor_order":
            s["motor_order"] = vals
        elif key in ("state_mean", "state_std", "action_mean", "action_std"):
            s[key] = np.array([float(x) for x in vals], dtype=np.float32)
        elif key.startswith("image_mean."):
            s["image_mean"][key[len("image_mean.") :]] = np.array([float(x) for x in vals], dtype=np.float32).reshape(
                3, 1, 1
            )
        elif key.startswith("image_std."):
            s["image_std"][key[len("image_std.") :]] = np.array([float(x) for x in vals], dtype=np.float32).reshape(
                3, 1, 1
            )
        elif key == "calib":
            name = vals[0]
            d, i = {}, 1
            while i + 1 < len(vals):
                d[vals[i]] = vals[i + 1]
                i += 2
            s["calib"][name] = {
                "drive_mode": int(d["drive_mode"]),
                "range_min": int(d["range_min"]),
                "range_max": int(d["range_max"]),
                "norm_mode": d["norm_mode"],
            }
    return s


def norm_to_raw(c, val, clamp=True):
    if c["norm_mode"] == "RANGE_0_100":
        if c["drive_mode"] == 1:
            val = 100.0 - val
        val = min(100.0, max(0.0, val))
        raw = val / 100.0 * (c["range_max"] - c["range_min"]) + c["range_min"]
    else:  # DEGREES
        if c["drive_mode"] == 1:
            val = -val
        mid = (c["range_min"] + c["range_max"]) / 2.0
        raw = val * MAX_STEP / 360.0 + mid
    iraw = int(round(raw))
    if clamp:
        iraw = min(c["range_max"], max(c["range_min"], iraw))
    return min(4095, max(0, iraw))


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--onnx", required=True, type=Path)
    p.add_argument("--stats", required=True, type=Path)
    p.add_argument("--out-dir", required=True, type=Path)
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args()


def main():
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    st = parse_stats(args.stats)

    img_h, img_w = st["img_h"], st["img_w"]
    state_dim = st["state_dim"]
    rng = np.random.default_rng(args.seed)

    raw_imgs = {name: rng.integers(0, 256, size=(img_h, img_w, 3), dtype=np.uint8) for name in st["cam_names"]}
    state_deg = np.array([5.0, -10.0, 15.0, -5.0, 30.0, 50.0], dtype=np.float32)[:state_dim]

    # images: (x/255 - mean)/std, HWC->CHW, stacked in model cam order
    norm_imgs = np.stack(
        [
            (np.transpose(raw_imgs[name].astype(np.float32) / 255.0, (2, 0, 1)) - st["image_mean"][name])
            / st["image_std"][name]
            for name in st["cam_names"]
        ],
        axis=0,
    ).astype(np.float32)

    np.save(args.out_dir / "images.npy", norm_imgs)
    np.save(args.out_dir / "state_deg.npy", state_deg)

    # state MEAN_STD then ONNX
    state_n = ((state_deg - st["state_mean"]) / st["state_std"]).astype(np.float32)

    import onnxruntime as ort

    sess = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
    in_names = [i.name for i in sess.get_inputs()]
    out_names = [o.name for o in sess.get_outputs()]
    feeds = {}
    for n in in_names:
        if n == "images":
            feeds[n] = norm_imgs[None].astype(np.float32)
        elif n == "state":
            feeds[n] = state_n[None].astype(np.float32)
    raw = sess.run(out_names, feeds)[0]  # (1, chunk, action_dim)
    a0 = (raw[0, 0] * st["action_std"] + st["action_mean"]).astype(np.float32)

    steps = [norm_to_raw(st["calib"][m], float(a0[i])) for i, m in enumerate(st["motor_order"])]
    lines = [
        "action0_lerobot: " + ", ".join(f"{v:.8g}" for v in a0.tolist()),
        "action0_rawstep: " + ", ".join(str(v) for v in steps),
    ]
    (args.out_dir / "ref_action0.txt").write_text("\n".join(lines) + "\n")
    print("[make-inputs] images.npy", norm_imgs.shape, "state_deg.npy", state_deg.shape)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
