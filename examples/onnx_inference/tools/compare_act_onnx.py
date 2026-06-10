#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Compare ACT ONNX output against the PyTorch reference (K3-friendly).

The comparison is done in the *normalized* space on the raw action chunk
(B, chunk_size, action_dim), which is exactly what the ONNX graph emits. The
PyTorch reference is ``ACT.forward`` inference branch (latent=0, no VAE encoder),
matching ``tools/act_pytorch_to_onnx.py``.

Inputs are generated from a fixed seed so the ONNX-only run on K3 and a
PyTorch run elsewhere use identical tensors.

Usage:
  # ONNX only (works on K3 without torch)
    python compare_act_onnx.py --onnx models/onnx/act-fp32/act.onnx \
      --checkpoint models/pytorch/<run>/checkpoints/100000/pretrained_model

  # Force SpaceMIT EP on K3
    python compare_act_onnx.py --onnx models/onnx/act-fp32/act.onnx \
      --checkpoint <ckpt> --use-spacemit-ep

  # Also run the PyTorch reference and compare (needs torch)
    python compare_act_onnx.py --onnx models/onnx/act-fp32/act.onnx \
      --checkpoint <ckpt> --with-torch
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort

# This script lives in ``examples/onnx_inference/tools/`` alongside
# ``act_pytorch_to_onnx.py`` (which provides ``ACTInferenceModule``).
TOOLS_DIR = Path(__file__).resolve().parent
EXAMPLE_DIR = TOOLS_DIR.parent


def read_act_config(checkpoint: Path) -> dict:
    """Read shapes from a checkpoint's config.json without importing torch."""
    with open(checkpoint / "config.json", encoding="utf-8") as f:
        cfg = json.load(f)
    image_features = []
    state_dim = 0
    for name, feat in cfg["input_features"].items():
        if feat["type"] == "VISUAL":
            image_features.append((name, tuple(feat["shape"])))
        elif feat["type"] == "STATE":
            state_dim = int(feat["shape"][0])
    action_dim = int(cfg["output_features"]["action"]["shape"][0])
    return {
        "image_features": image_features,
        "state_dim": state_dim,
        "action_dim": action_dim,
        "chunk_size": int(cfg["chunk_size"]),
    }


def make_fixed_inputs(meta: dict, seed: int = 0, batch_size: int = 1) -> dict[str, np.ndarray]:
    """Build deterministic normalized inputs matching the ONNX graph signature."""
    rng = np.random.default_rng(seed)
    inputs: dict[str, np.ndarray] = {}
    if meta["image_features"]:
        n_cam = len(meta["image_features"])
        _, (c, h, w) = meta["image_features"][0]
        inputs["images"] = rng.standard_normal((batch_size, n_cam, c, h, w), dtype=np.float32)
    if meta["state_dim"]:
        inputs["state"] = rng.standard_normal((batch_size, meta["state_dim"]), dtype=np.float32)
    return inputs


def build_session(
    onnx_path: Path, use_spacemit_ep: bool, ep_threads: int = 4, ep_affinity: str = "", cpu_threads: int = 0
) -> ort.InferenceSession:
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    if cpu_threads > 0:
        so.intra_op_num_threads = cpu_threads
    providers: list = ["CPUExecutionProvider"]
    provider_options = None
    if use_spacemit_ep:
        try:
            import spacemit_ort  # noqa: F401  # registers SpaceMITExecutionProvider
        except ImportError:
            print("[warn] spacemit_ort not importable; falling back to CPU")
        else:
            providers = ["SpaceMITExecutionProvider", "CPUExecutionProvider"]
            ep_opt = {"SPACEMIT_EP_INTRA_THREAD_NUM": str(ep_threads)}
            if ep_affinity:
                ep_opt["SPACEMIT_EP_INTRA_THREAD_AFFINITY"] = ep_affinity
            provider_options = [ep_opt, {}]
    sess = ort.InferenceSession(
        str(onnx_path), sess_options=so, providers=providers, provider_options=provider_options
    )
    print(f"[onnx] providers={sess.get_providers()}")
    return sess


def run_onnx(sess: ort.InferenceSession, inputs: dict[str, np.ndarray], warmup: int, repeat: int):
    feed = {i.name: inputs[i.name] for i in sess.get_inputs()}
    for _ in range(warmup):
        sess.run(None, feed)
    times = []
    out = None
    for _ in range(repeat):
        t0 = time.perf_counter()
        out = sess.run(None, feed)
        times.append((time.perf_counter() - t0) * 1000.0)
    actions = np.asarray(out[0], dtype=np.float32)
    print(
        f"[onnx] output shape={actions.shape} latency: mean={np.mean(times):.2f}ms min={np.min(times):.2f}ms"
    )
    return actions


def run_torch_reference(checkpoint: Path, inputs: dict[str, np.ndarray]) -> np.ndarray:
    """Run the PyTorch ACTInferenceModule on identical inputs (needs torch)."""
    import sys

    import torch

    # Prepend local lerobot/src so it takes priority over any pip-installed
    # version (which may have broken groot_n1.py @dataclass field ordering).
    local_src = EXAMPLE_DIR.parents[2] / "src"
    if local_src.exists() and str(local_src) not in sys.path:
        sys.path.insert(0, str(local_src))
    if str(TOOLS_DIR) not in sys.path:
        sys.path.insert(0, str(TOOLS_DIR))
    from act_pytorch_to_onnx import ACTInferenceModule  # noqa: E402

    from lerobot.policies.act.modeling_act import ACTPolicy  # noqa: E402

    policy = ACTPolicy.from_pretrained(str(checkpoint))
    policy.eval()
    module = ACTInferenceModule(policy).eval()
    kwargs = {k: torch.from_numpy(v) for k, v in inputs.items()}
    with torch.no_grad():
        actions = module(**kwargs)
    return actions.cpu().numpy().astype(np.float32)


def report_diff(onnx_out: np.ndarray, ref: np.ndarray, atol: float, rtol: float) -> bool:
    if onnx_out.shape != ref.shape:
        print(f"[diff] SHAPE MISMATCH onnx={onnx_out.shape} ref={ref.shape}")
        return False
    diff = np.abs(onnx_out - ref)
    max_abs = float(diff.max())
    denom = np.abs(ref) + 1e-9
    max_rel = float((diff / denom).max())
    ok = bool(np.allclose(onnx_out, ref, atol=atol, rtol=rtol))
    print(
        f"[diff] max|abs|={max_abs:.3e} max|rel|={max_rel:.3e} "
        f"atol={atol} rtol={rtol} -> {'OK' if ok else 'MISMATCH'}"
    )
    return ok


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--onnx", type=Path, required=True, help="Path to act.onnx")
    p.add_argument(
        "--checkpoint",
        type=Path,
        required=True,
        help="ACT checkpoint dir (for config.json)",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Fixed input seed (must match on both sides)",
    )
    p.add_argument("--batch-size", type=int, default=1)
    p.add_argument("--warmup", type=int, default=2)
    p.add_argument("--repeat", type=int, default=10)
    p.add_argument("--use-spacemit-ep", action="store_true", help="Prefer SpaceMIT EP on K3")
    p.add_argument("--ep-threads", type=int, default=4, help="SpaceMIT EP intra thread count")
    p.add_argument("--ep-affinity", default="", help="SpaceMIT EP core affinity, e.g. '8;9;10;11'")
    p.add_argument(
        "--cpu-threads",
        type=int,
        default=0,
        help="CPU EP intra_op_num_threads (0=ORT default)",
    )
    p.add_argument(
        "--with-torch",
        action="store_true",
        help="Run PyTorch reference and compare (needs torch)",
    )
    p.add_argument(
        "--save-ref",
        type=Path,
        default=None,
        help="Save PyTorch reference output to .npy",
    )
    p.add_argument(
        "--ref-npy",
        type=Path,
        default=None,
        help="Load reference output .npy and compare",
    )
    p.add_argument(
        "--save-inputs",
        type=Path,
        default=None,
        help="Dir to dump fixed inputs as <name>.npy (for the C++ benchmark)",
    )
    p.add_argument("--atol", type=float, default=1e-3)
    p.add_argument("--rtol", type=float, default=1e-3)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    meta = read_act_config(args.checkpoint)
    print(
        f"[config] cameras={len(meta['image_features'])} state={meta['state_dim']} "
        f"action_dim={meta['action_dim']} chunk={meta['chunk_size']}"
    )

    inputs = make_fixed_inputs(meta, seed=args.seed, batch_size=args.batch_size)
    for name, arr in inputs.items():
        print(f"[input] {name}: shape={arr.shape} dtype={arr.dtype}")

    if args.save_inputs:
        args.save_inputs.mkdir(parents=True, exist_ok=True)
        for name, arr in inputs.items():
            out_path = args.save_inputs / f"{name}.npy"
            np.save(out_path, np.ascontiguousarray(arr, dtype=np.float32))
            print(f"[input] saved -> {out_path}")

    sess = build_session(
        args.onnx,
        args.use_spacemit_ep,
        args.ep_threads,
        args.ep_affinity,
        args.cpu_threads,
    )
    onnx_out = run_onnx(sess, inputs, args.warmup, args.repeat)

    ref = None
    if args.with_torch:
        ref = run_torch_reference(args.checkpoint, inputs)
        if args.save_ref:
            args.save_ref.parent.mkdir(parents=True, exist_ok=True)
            np.save(args.save_ref, ref)
            print(f"[ref] saved PyTorch reference -> {args.save_ref}")
    elif args.ref_npy:
        ref = np.load(args.ref_npy).astype(np.float32)
        print(f"[ref] loaded reference -> {args.ref_npy} shape={ref.shape}")

    if ref is None:
        print("[done] ONNX-only run (no reference to compare). Use --with-torch or --ref-npy to compare.")
        return 0

    ok = report_diff(onnx_out, ref, args.atol, args.rtol)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
