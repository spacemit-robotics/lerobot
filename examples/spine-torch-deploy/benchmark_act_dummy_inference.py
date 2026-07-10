#!/usr/bin/env python3
"""Benchmark ACT inference with synthetic observations.

FP32 example::

    python examples/spine-torch-deploy/benchmark_act_dummy_inference.py \
        --model-path /path/to/pretrained_model \
        --device cpu

K3 SpineTorch FP16 example::

    python examples/spine-torch-deploy/benchmark_act_dummy_inference.py \
        --model-path /path/to/pretrained_model_fp16 \
        --device cpu \
        --use-half \
        --keep-mkldnn \
        --use-spinednn
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import time
from pathlib import Path
from typing import Any

import torch
from spine_runtime import load_fp16_policy
from torch.profiler import ProfilerActivity, profile

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.utils.constants import OBS_IMAGES, OBS_STATE


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark ACT inference with dummy inputs.")
    parser.add_argument(
        "--model-path",
        type=str,
        required=True,
        help="Path to pretrained_model directory.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Device to run on, e.g. cuda, cuda:0, cpu. Defaults to model config device.",
    )
    parser.add_argument("--warmup", type=int, default=10, help="Number of warmup iterations.")
    parser.add_argument("--iters", type=int, default=50, help="Number of timed iterations.")
    parser.add_argument(
        "--use-half",
        action="store_true",
        help="Use float16 for model, inputs, bias and outputs.",
    )
    parser.add_argument(
        "--input-bhwc",
        action="store_true",
        help="Convert image inputs to BHWC layout for debugging/layout validation.",
    )
    parser.add_argument(
        "--print-json-config",
        action="store_true",
        help="Print loaded model config json summary.",
    )
    parser.add_argument(
        "--keep-mkldnn",
        action="store_true",
        help="Keep mkldnn enabled instead of forcing it off.",
    )
    parser.add_argument(
        "--use-spinednn",
        action="store_true",
        help="Enable the optional SpineDNN plugin used by the K3 FP16 path.",
    )
    return parser.parse_args()


def configure_spinednn(use_spinednn: bool) -> None:
    if not use_spinednn:
        return

    try:
        import spinednn_torch_plugin as plugin
    except ImportError as exc:
        raise RuntimeError("Install spinednn_torch_plugin before using --use-spinednn.") from exc

    plugin.only_enable(["addmm", "bmm", "softmax", "layer_norm", "relu", "add"])
    print("spinednn plugin enabled:", plugin.is_enabled())


def configure_mkldnn(keep_mkldnn: bool) -> None:
    print("mkldnn before:", torch.backends.mkldnn.enabled)
    if not keep_mkldnn:
        torch.backends.mkldnn.enabled = False
    print("mkldnn after:", torch.backends.mkldnn.enabled)


def infer_tensor_dtype(device: torch.device, use_half: bool) -> torch.dtype:
    if use_half:
        return torch.float16
    return torch.float32


def maybe_to_bhwc_tensor(tensor: torch.Tensor, input_bhwc: bool) -> torch.Tensor:
    if input_bhwc and tensor.ndim == 4:
        return tensor.permute(0, 2, 3, 1).contiguous()
    return tensor


def maybe_to_bhwc_batch(batch: dict[str, Any], input_bhwc: bool) -> dict[str, Any]:
    converted: dict[str, Any] = {}
    for key, value in batch.items():
        if isinstance(value, torch.Tensor):
            converted[key] = maybe_to_bhwc_tensor(value, input_bhwc)
        elif isinstance(value, list):
            converted[key] = [
                maybe_to_bhwc_tensor(item, input_bhwc) if isinstance(item, torch.Tensor) else item
                for item in value
            ]
        else:
            converted[key] = value
    return converted


def tensor_layout_str(tensor: torch.Tensor) -> str:
    if tensor.ndim == 4 and tensor.shape[-1] in {1, 3}:
        return "bhwc"
    if tensor.is_contiguous():
        return "contiguous"
    return "non_contiguous"


def make_dummy_batch(policy: ACTPolicy, device: torch.device, dtype: torch.dtype) -> dict[str, Any]:
    cfg = policy.config
    batch: dict[str, Any] = {}

    if cfg.robot_state_feature is not None:
        state_shape = cfg.robot_state_feature.shape
        batch[OBS_STATE] = torch.randn((1, *state_shape), device=device, dtype=dtype)

    if cfg.image_features:
        for image_key, feature in cfg.image_features.items():
            shape = feature.shape
            batch[image_key] = torch.randn((1, *shape), device=device, dtype=dtype)

    if not batch:
        raise ValueError("Could not build dummy batch: no robot state or image features found in config.")

    return batch


def make_display_batch(policy: ACTPolicy, batch: dict[str, Any]) -> dict[str, Any]:
    display_batch = dict(batch)
    if policy.config.image_features:
        display_batch[OBS_IMAGES] = [batch[key] for key in policy.config.image_features]
    return display_batch


def describe_value(name: str, value: Any, indent: int = 0) -> None:
    prefix = " " * indent
    if isinstance(value, torch.Tensor):
        tensor_description = (
            f"Tensor(shape={tuple(value.shape)}, dtype={value.dtype}, device={value.device}, "
            f"layout={tensor_layout_str(value)}, type={type(value).__name__})"
        )
        print(f"{prefix}{name}: {tensor_description}")
    elif isinstance(value, list):
        print(f"{prefix}{name}: list(len={len(value)}, type={type(value).__name__})")
        for i, item in enumerate(value):
            describe_value(f"[{i}]", item, indent + 2)
    elif isinstance(value, tuple):
        print(f"{prefix}{name}: tuple(len={len(value)}, type={type(value).__name__})")
        for i, item in enumerate(value):
            describe_value(f"[{i}]", item, indent + 2)
    elif isinstance(value, dict):
        print(f"{prefix}{name}: dict(type={type(value).__name__})")
        for k, v in value.items():
            describe_value(str(k), v, indent + 2)
    else:
        print(f"{prefix}{name}: type={type(value).__name__}, value={value}")


def sync_if_needed(device: torch.device) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def benchmark_select_action(policy: ACTPolicy, batch: dict[str, Any], warmup: int, iters: int) -> list[float]:
    device = next(policy.parameters()).device
    for _ in range(warmup):
        policy.reset()
        sync_if_needed(device)
        _ = policy.select_action(batch)
        sync_if_needed(device)

    latencies_ms = []
    for _ in range(iters):
        policy.reset()
        sync_if_needed(device)
        start = time.perf_counter()
        _ = policy.select_action(batch)
        sync_if_needed(device)
        end = time.perf_counter()
        latencies_ms.append((end - start) * 1000.0)
    return latencies_ms


def profile_top_ops(policy: ACTPolicy, batch: dict[str, Any]) -> None:
    activities = [ProfilerActivity.CPU]
    device = next(policy.parameters()).device
    if device.type == "cuda":
        activities.append(ProfilerActivity.CUDA)

    policy.reset()
    with profile(
        activities=activities,
        record_shapes=True,
        profile_memory=True,
        with_stack=False,
    ) as prof:
        _ = policy.select_action(batch)
        sync_if_needed(device)

    sort_key = "self_cuda_time_total" if device.type == "cuda" else "self_cpu_time_total"
    print("\nTop 20 torch ops by self time:")
    print(prof.key_averages().table(sort_by=sort_key, row_limit=20))


def main() -> None:
    args = parse_args()
    configure_spinednn(args.use_spinednn)
    configure_mkldnn(args.keep_mkldnn)
    model_path = Path(args.model_path).expanduser().resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"Model path does not exist: {model_path}")

    effective_use_half = args.use_half

    if effective_use_half:
        policy = load_fp16_policy(str(model_path), device=args.device or "cpu")
    else:
        policy = ACTPolicy.from_pretrained(str(model_path))
    target_device = torch.device(args.device) if args.device else next(policy.parameters()).device
    policy.to(target_device)
    policy.eval()
    policy.reset()

    dtype = infer_tensor_dtype(target_device, effective_use_half)
    batch = make_dummy_batch(policy, target_device, dtype)
    batch = maybe_to_bhwc_batch(batch, args.input_bhwc)

    if args.print_json_config:
        print("Loaded config summary:")
        print(json.dumps(dataclasses.asdict(policy.config), indent=2, default=str))

    first_param = next(policy.parameters())
    print("Model parameter dtype:", first_param.dtype)
    print("Effective use_half:", effective_use_half)
    print("Input image layout request:", "BHWC" if args.input_bhwc else "BCHW")

    print("=== Model input types ===")
    describe_value("batch", make_display_batch(policy, batch))

    with torch.no_grad():
        policy.reset()
        output = policy.select_action(batch)

    print("\n=== Model output types ===")
    describe_value("action", output)

    latencies_ms = benchmark_select_action(policy, batch, warmup=args.warmup, iters=args.iters)
    avg_ms = sum(latencies_ms) / len(latencies_ms)
    p50_ms = sorted(latencies_ms)[len(latencies_ms) // 2]
    p95_ms = sorted(latencies_ms)[min(len(latencies_ms) - 1, int(len(latencies_ms) * 0.95))]

    print("\n=== Inference latency (select_action, reset per iter) ===")
    print(f"device: {target_device}")
    print(f"dtype: {dtype}")
    print(f"warmup: {args.warmup}, iters: {args.iters}")
    print(f"avg: {avg_ms:.3f} ms")
    print(f"p50: {p50_ms:.3f} ms")
    print(f"p95: {p95_ms:.3f} ms")
    print(f"min: {min(latencies_ms):.3f} ms")
    print(f"max: {max(latencies_ms):.3f} ms")

    profile_top_ops(policy, batch)


if __name__ == "__main__":
    main()
