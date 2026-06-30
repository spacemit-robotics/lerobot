#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Compare SmolVLA fp32 and fp16_surgeried 4-model ONNX outputs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np
import onnxruntime as ort

try:
    import spacemit_ort  # noqa: F401  # Registers SpaceMITExecutionProvider.
except ImportError:
    spacemit_ort = None

SPACEMIT_EP_ENV_DEFAULTS = {
    "SPACEMIT_EP_PWCONV_INT8_USE": "1",
    "SPACEMIT_EP_CONVTRANSPOSE_4X4_FP16_USE": "1",
    "SPACEMIT_EP_CONVTRANSPOSE_4X4_FP16_LOG": "1",
    "SPACEMIT_EP_CONVTRANSPOSE_3X3_FP16_USE": "1",
    "ORT_CPU_EP_DIV_FP32_RVV_USE": "1",
    "SPACEMIT_EP_CONCAT_FP16_RVV_USE": "1",
    "SPACEMIT_EP_GATHER_FP16_RVV_USE": "1",
    "SPACEMIT_EP_MUL_FP16_RVV_USE": "1",
    "SPACEMIT_EP_ERF_FP16_RVV_USE": "1",
    "SPACEMIT_EP_SIN_FP16_RVV_USE": "1",
    "SPACEMIT_EP_COS_FP16_RVV_USE": "1",
    "SPACEMIT_EP_WHERE_FP16_USE": "1",
    "SPACEMIT_EP_POW_FP16_RVV_USE": "0",
    "SPACEMIT_EP_REDUCEMEAN_FP16_RVV_USE": "1",
    "SPACEMIT_EP_SOFTMAX_FP16_USE": "0",
    "SPACEMIT_EP_CONV3D_RVV_USE": "1",
    "SPACEMIT_EP_REDUCESUM_FP32_USE": "1",
    "SPACEMIT_EP_SEPDWCONV_USE": "1",
    "SPACEMIT_EP_DWCONV_3X3_FP32_USE": "1",
    "SPACEMIT_EP_DWCONV_3X3_S2_FP32_USE": "1",
    "SPACEMIT_EP_ADD_QDQ_RVV_USE": "1",
    "SPACEMIT_EP_REDUCEMEAN_QDQ_RVV_USE": "1",
    "SPACEMIT_EP_SOFTMAX_QDQ_INT8_USE": "1",
    "SPACEMIT_EP_GELU_QDQ_INT8_USE": "1",
    "SPACEMIT_EP_LAYERNORM_QDQ_INT8_USE": "1",
    "SPACEMIT_EP_CONVTRANSPOSE_3X3_USE": "1",
    "SPACEMIT_EP_CONVTRANSPOSE_4X4_USE": "1",
    "SPACEMIT_EP_POW2_REDUCEMEAN_USE": "0",
}

MODEL_NAMES = ["vision_encoder", "connector", "prefill_lm", "denoise_step"]


def _resolve_spacemit_ep_library(spacemit_ort_dir: Path | None) -> Path | None:
    if spacemit_ort_dir is None:
        return None

    ort_dir = spacemit_ort_dir.expanduser()
    candidates = (ort_dir / "lib" / "libspacemit_ep.so", ort_dir / "libspacemit_ep.so")
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"cannot find libspacemit_ep.so under {ort_dir}")


def _install_spacemit_ep_shared_library(ep_lib: Path) -> None:
    import onnxruntime.capi._pybind_state as pybind_state

    provider_name = "SpaceMITExecutionProvider"
    entry_point = "GetSpaceMITSharedProviderFactory"
    ep_lib = ep_lib.expanduser()
    if not ep_lib.exists():
        raise FileNotFoundError(ep_lib)

    if getattr(pybind_state.InferenceSession.initialize_session, "_spacemit_ep_patched", False):
        return

    original_initialize_session = pybind_state.InferenceSession.initialize_session

    def initialize_session_with_spacemit(*args, **kwargs):
        sess, providers, provider_options, disabled_optimizers = args[:4]
        if provider_name in providers:
            ep_index = providers.index(provider_name)
            provider_options[ep_index]["shared_lib_path"] = str(ep_lib)
            provider_options[ep_index]["provider_factory_entry_point"] = entry_point
        if "CPUExecutionProvider" not in providers:
            providers.append("CPUExecutionProvider")
            provider_options.append({})
        return original_initialize_session(*args, **kwargs)

    initialize_session_with_spacemit._spacemit_ep_patched = True
    pybind_state.InferenceSession.initialize_session = initialize_session_with_spacemit


def _providers(use_ep: bool, ep_threads: int, ep_affinity: str, ep_lib: Path | None):
    if not use_ep:
        return ["CPUExecutionProvider"], None
    if spacemit_ort is None:
        raise RuntimeError("spacemit_ort is not installed; cannot use SpaceMITExecutionProvider")
    for key, value in SPACEMIT_EP_ENV_DEFAULTS.items():
        os.environ.setdefault(key, value)
    if ep_lib is not None:
        _install_spacemit_ep_shared_library(ep_lib)
    options = {
        "SPACEMIT_EP_INTRA_THREAD_NUM": str(ep_threads),
        "SPACEMIT_EP_INTRA_THREAD_AFFINITY": ep_affinity,
    }
    if ep_lib is not None:
        options["shared_lib_path"] = str(ep_lib.expanduser())
        options["provider_factory_entry_point"] = "GetSpaceMITSharedProviderFactory"
    return ["SpaceMITExecutionProvider", "CPUExecutionProvider"], [options, {}]


def _make_sessions(
    model_dir: Path,
    use_ep: bool,
    args: argparse.Namespace,
    spacemit_ep_library: Path | None,
) -> dict[str, ort.InferenceSession]:
    providers, provider_options = _providers(use_ep, args.ep_threads, args.ep_affinity, spacemit_ep_library)
    opts = ort.SessionOptions()
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    sessions = {}
    for name in MODEL_NAMES:
        path = model_dir / f"{name}.onnx"
        if not path.exists():
            raise FileNotFoundError(path)
        sessions[name] = ort.InferenceSession(
            str(path),
            opts,
            providers=providers,
            provider_options=provider_options,
        )
        print(f"  {model_dir.name}/{name:<14} providers={sessions[name].get_providers()}")
    return sessions


def _dtype(session: ort.InferenceSession, input_index: int) -> np.dtype:
    return np.float16 if session.get_inputs()[input_index].type == "tensor(float16)" else np.float32


def _camera_count_from_export_config(model_dir: Path) -> int | None:
    config_path = model_dir / "export_config.json"
    if not config_path.exists():
        return None
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if "num_cameras" in config:
        return int(config["num_cameras"])
    if "model_camera_count" in config:
        return int(config["model_camera_count"])
    image_keys = config.get("image_keys")
    if image_keys is not None:
        return len(image_keys) + int(config.get("empty_cameras", 0))
    return None


def _camera_count(model_dir: Path, sessions: dict[str, ort.InferenceSession], override: int | None) -> int:
    if override is not None:
        if override <= 0:
            raise ValueError("--num-cameras must be positive")
        return override

    config_count = _camera_count_from_export_config(model_dir)
    if config_count is not None:
        return config_count

    connector_tokens = sessions["connector"].get_outputs()[0].shape[1]
    prefill_tokens = sessions["prefill_lm"].get_inputs()[0].shape[1]
    if not isinstance(connector_tokens, int) or not isinstance(prefill_tokens, int):
        raise ValueError("Cannot infer camera count from dynamic ONNX shapes; pass --num-cameras")
    return prefill_tokens // connector_tokens


def _cast(session: ort.InferenceSession, input_index: int, value: np.ndarray) -> np.ndarray:
    target = _dtype(session, input_index)
    return value.astype(target, copy=False)


def run_pipeline(
    sessions: dict[str, ort.InferenceSession],
    images: np.ndarray,
    lang_tokens: np.ndarray,
    lang_masks: np.ndarray,
    state: np.ndarray,
    noise: np.ndarray,
    denoise_steps: int,
) -> np.ndarray:
    image_embs = []
    vision = sessions["vision_encoder"]
    connector = sessions["connector"]
    for camera_index in range(images.shape[1]):
        image = _cast(vision, 0, images[:, camera_index])
        hidden = vision.run(None, {vision.get_inputs()[0].name: image})[0]
        image_embs.append(
            connector.run(None, {connector.get_inputs()[0].name: _cast(connector, 0, hidden)})[0]
        )

    prefill = sessions["prefill_lm"]
    image_embs_np = np.concatenate(image_embs, axis=1)
    prefill_outputs = prefill.run(
        None,
        {
            prefill.get_inputs()[0].name: _cast(prefill, 0, image_embs_np),
            prefill.get_inputs()[1].name: lang_tokens,
            prefill.get_inputs()[2].name: lang_masks,
            prefill.get_inputs()[3].name: _cast(prefill, 3, state),
        },
    )
    if len(prefill_outputs) != 3:
        raise ValueError(f"Unsupported prefill output count: {len(prefill_outputs)}")
    past_keys, past_values, prefix_pad_masks = prefill_outputs

    denoise = sessions["denoise_step"]
    x_t = _cast(denoise, 0, noise)
    dt = np.float32(-1.0 / denoise_steps)
    for step in range(denoise_steps):
        timestep = np.asarray([1.0 + step * float(dt)], dtype=_dtype(denoise, 1))
        v_t = denoise.run(
            None,
            {
                denoise.get_inputs()[0].name: x_t,
                denoise.get_inputs()[1].name: timestep,
                denoise.get_inputs()[2].name: prefix_pad_masks,
                denoise.get_inputs()[3].name: past_keys,
                denoise.get_inputs()[4].name: past_values,
            },
        )[0]
        x_t = x_t + dt * v_t.astype(x_t.dtype, copy=False)
    return x_t.astype(np.float32)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fp32-dir", type=Path, required=True, help="Reference fp32 4-model ONNX directory")
    parser.add_argument(
        "--fp16-dir", type=Path, required=True, help="Candidate fp16_surgeried ONNX directory"
    )
    parser.set_defaults(use_spacemit_ep=True)
    parser.add_argument(
        "--use-spacemit-ep", dest="use_spacemit_ep", action="store_true", help="Use SpaceMIT EP (default)"
    )
    parser.add_argument(
        "--cpu", dest="use_spacemit_ep", action="store_false", help="Run ONNX models with CPU EP"
    )
    parser.add_argument("--ep-threads", type=int, default=8)
    parser.add_argument("--ep-affinity", default="8;9;10;11;12;13;14;15")
    parser.add_argument(
        "--spacemit-ort-dir",
        type=Path,
        default=None,
        help="SpaceMIT ORT SDK directory; omit to use the system SpaceMIT EP",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--denoise-steps", type=int, default=10)
    parser.add_argument("--num-cameras", type=int, default=None, help="Override model camera count")
    parser.add_argument("--save-fp32", type=Path, default=None)
    parser.add_argument("--save-fp16", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    spacemit_ep_library = _resolve_spacemit_ep_library(args.spacemit_ort_dir)
    fp32_sessions = _make_sessions(args.fp32_dir, args.use_spacemit_ep, args, spacemit_ep_library)
    fp16_sessions = _make_sessions(args.fp16_dir, args.use_spacemit_ep, args, spacemit_ep_library)
    num_cameras = _camera_count(args.fp32_dir, fp32_sessions, args.num_cameras)
    if _camera_count(args.fp16_dir, fp16_sessions, args.num_cameras) != num_cameras:
        raise ValueError("fp32/fp16 model camera counts differ")

    rng = np.random.default_rng(args.seed)
    images = rng.normal(size=(1, num_cameras, 3, 512, 512)).astype(np.float32)
    lang_tokens = rng.integers(0, 1000, size=(1, 48), dtype=np.int64)
    lang_masks = np.ones((1, 48), dtype=np.int64)
    state = rng.normal(size=(1, 32)).astype(np.float32)
    noise = rng.normal(size=(1, 50, 32)).astype(np.float32)

    fp32_actions = run_pipeline(
        fp32_sessions, images, lang_tokens, lang_masks, state, noise, args.denoise_steps
    )
    fp16_actions = run_pipeline(
        fp16_sessions, images, lang_tokens, lang_masks, state, noise, args.denoise_steps
    )
    if args.save_fp32 is not None:
        args.save_fp32.parent.mkdir(parents=True, exist_ok=True)
        np.save(args.save_fp32, fp32_actions)
    if args.save_fp16 is not None:
        args.save_fp16.parent.mkdir(parents=True, exist_ok=True)
        np.save(args.save_fp16, fp16_actions)

    diff = np.abs(fp16_actions[:, :, :6] - fp32_actions[:, :, :6])
    print(f"camera_count={num_cameras} denoise_steps={args.denoise_steps}")
    print("fp32 first_action6:", fp32_actions[0, 0, :6].tolist())
    print("fp16 first_action6:", fp16_actions[0, 0, :6].tolist())
    print("diff first_action6:", (fp16_actions[0, 0, :6] - fp32_actions[0, 0, :6]).tolist())
    print(
        "diff action6: "
        f"max={float(np.max(diff)):.8g} mean={float(np.mean(diff)):.8g} "
        f"p95={float(np.percentile(diff, 95)):.8g} p99={float(np.percentile(diff, 99)):.8g}"
    )
    print(f"finite fp32={bool(np.isfinite(fp32_actions).all())} fp16={bool(np.isfinite(fp16_actions).all())}")


if __name__ == "__main__":
    main()
