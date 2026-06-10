#!/usr/bin/env python3
"""ACT ONNX dummy benchmark script.

Use this to measure ACT ONNX inference latency on CPU or SpaceMIT EP.

Example:
  python python/act_benchmark.py \
      --onnx models/onnx/act-fp32/act.onnx \
      --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
      --use-spacemit-ep \
      --ep-threads 8 \
      --ep-affinity "8;9;10;11;12;13;14;15" \
      --warmup 5 --iters 20
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort

try:
    import safetensors  # noqa: F401
except ImportError:
    safetensors = None

from utils.act_runtime import build_session as build_ort_session, random_float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark ACT ONNX inference latency")
    parser.add_argument("--onnx", type=Path, required=True, help="Path to ACT ONNX model")
    parser.add_argument(
        "--checkpoint",
        type=Path,
        required=True,
        help="Path to pretrained_model dir for config and normalization stats",
    )
    parser.add_argument("--use-spacemit-ep", action="store_true", help="Use SpaceMITExecutionProvider if available")
    parser.add_argument("--ep-threads", type=int, default=8, help="SpaceMIT EP intra thread count")
    parser.add_argument(
        "--ep-affinity", default="8;9;10;11;12;13;14;15", help="SpaceMIT EP intra thread affinity string"
    )
    parser.add_argument("--cpu-threads", type=int, default=0, help="ORT CPU intra-op thread count (0 = default)")
    parser.add_argument("--warmup", type=int, default=5, help="Warmup iterations")
    parser.add_argument("--iters", type=int, default=20, help="Measurement iterations")
    parser.add_argument("--batch", type=int, default=1, help="Batch size for random inputs")
    return parser.parse_args()


def read_checkpoint_meta(checkpoint_dir: Path) -> dict:
    config_path = checkpoint_dir / "config.json"
    if not config_path.exists():
        raise FileNotFoundError(f"Checkpoint config.json not found: {config_path}")
    cfg = json.loads(config_path.read_text(encoding="utf-8"))

    image_keys = [k for k, v in cfg["input_features"].items() if v["type"] == "VISUAL"]
    state_dim = cfg["input_features"]["observation.state"]["shape"][0]
    return {
        "image_keys": image_keys,
        "cam_names": [key.split("observation.images.")[-1] for key in image_keys],
        "state_dim": state_dim,
    }


def load_norm_stats(checkpoint_dir: Path, meta: dict) -> dict:
    if safetensors is None:
        raise RuntimeError("safetensors is required to read checkpoint normalization stats")
    from safetensors import safe_open

    pre_path = checkpoint_dir / "policy_preprocessor_step_3_normalizer_processor.safetensors"
    post_path = checkpoint_dir / "policy_postprocessor_step_0_unnormalizer_processor.safetensors"
    if not pre_path.exists() or not post_path.exists():
        raise FileNotFoundError("Missing safetensors files in checkpoint directory")

    def load_tensor(path: Path, key: str) -> np.ndarray:
        with safe_open(str(path), framework="np") as sf:
            return np.asarray(sf.get_tensor(key), dtype=np.float32)

    stats = {
        "state_mean": load_tensor(pre_path, "observation.state.mean"),
        "state_std": load_tensor(pre_path, "observation.state.std"),
        "action_mean": load_tensor(post_path, "action.mean"),
        "action_std": load_tensor(post_path, "action.std"),
        "image_mean": {},
        "image_std": {},
    }
    for key, name in zip(meta["image_keys"], meta["cam_names"], strict=True):
        stats["image_mean"][name] = load_tensor(pre_path, f"{key}.mean").reshape(3, 1, 1)
        stats["image_std"][name] = load_tensor(pre_path, f"{key}.std").reshape(3, 1, 1)
    return stats


def build_random_inputs(session: ort.InferenceSession, batch: int):
    feeds = {}
    for inp in session.get_inputs():
        shape = [batch if (s is None or s == "batch" or s == "?" or s == -1) else s for s in inp.shape]
        dtype = np.float16 if inp.type == "tensor(float16)" else np.float32
        if inp.type in {"tensor(float)", "tensor(float32)", "tensor(float16)"}:
            feeds[inp.name] = random_float(tuple(shape), dtype)
        elif inp.type == "tensor(int64)":
            feeds[inp.name] = np.zeros(shape, dtype=np.int64)
        else:
            raise RuntimeError(f"Unsupported input type: {inp.type}")
    return feeds


def run_benchmark(session: ort.InferenceSession, feeds: dict, warmup: int, iters: int):
    output_names = [o.name for o in session.get_outputs()]
    for _ in range(warmup):
        session.run(output_names, feeds)

    latencies = []
    for i in range(iters):
        t0 = time.perf_counter()
        session.run(output_names, feeds)
        latencies.append((time.perf_counter() - t0) * 1000.0)
        print(f"iter {i + 1}/{iters}: {latencies[-1]:.1f} ms")

    arr = np.array(latencies)
    return {
        "avg_ms": arr.mean(),
        "p50_ms": np.percentile(arr, 50),
        "p95_ms": np.percentile(arr, 95),
        "min_ms": arr.min(),
        "max_ms": arr.max(),
    }


def main() -> None:
    args = parse_args()
    print("ACT ONNX benchmark")
    print("model:", args.onnx)
    print("checkpoint:", args.checkpoint)
    print("use_spacemit_ep:", args.use_spacemit_ep)
    print("ep_threads:", args.ep_threads)
    print("ep_affinity:", args.ep_affinity)
    print("cpu_threads:", args.cpu_threads)
    print("warmup:", args.warmup)
    print("iters:", args.iters)
    print("batch:", args.batch)

    meta = read_checkpoint_meta(args.checkpoint)
    load_norm_stats(args.checkpoint, meta)
    print("Checkpoint meta:", meta)
    print("Loaded normalization stats")

    session, load_ms = build_ort_session(
        args.onnx,
        args.use_spacemit_ep,
        args.ep_threads,
        args.ep_affinity,
        args.cpu_threads,
    )
    print("Loaded", args.onnx.name, f"in {load_ms:.0f} ms", "providers=", session.get_providers())
    feeds = build_random_inputs(session, args.batch)
    print("Input tensors:", {k: v.shape for k, v in feeds.items()})

    results = run_benchmark(session, feeds, args.warmup, args.iters)
    print("\n=== Results ===")
    for k, v in results.items():
        print(f"{k}: {v:.2f} ms")


if __name__ == "__main__":
    main()
