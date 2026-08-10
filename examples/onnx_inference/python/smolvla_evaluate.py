#!/usr/bin/env python3
"""SO101 online grasp pipeline with the SmolVLA 4-model ONNX export.

The script connects an SO101 follower arm and OpenCV cameras, preprocesses the
real observation with the training checkpoint processors, runs the 4-model ONNX
pipeline (vision_encoder -> connector -> prefill_lm -> denoise_step), unnormalizes
the first 6 action dimensions, and streams actions to the robot.

Use --no-robot for a dry-run smoke test without USB hardware.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch
import torch.nn.functional as functional
from safetensors.torch import load_file
from tokenizers import Tokenizer

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

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))

from lerobot.utils.constants import OBS_STATE  # noqa: E402

EXAMPLE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_DIR = EXAMPLE_DIR / "models" / "onnx" / "smolvla-fp32"
DEFAULT_CHECKPOINT = (
    EXAMPLE_DIR / "models" / "pytorch" / "smolvla" / "checkpoints" / "100000" / "pretrained_model"
)
DEFAULT_TASK = "Place the green cube into the box"
DEFAULT_FPS = 30
DEFAULT_CAMERA_FPS = 25
DEFAULT_CAMERA_FOURCC = "MJPG"
DEFAULT_DENOISE_STEPS = 10
DEFAULT_EPISODE_TIME_S = 60.0
TOKENIZER_NAME = "HuggingFaceTB/SmolVLM2-500M-Video-Instruct"

SO101_MOTORS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
SO101_STATE_KEYS = tuple(f"{motor}.pos" for motor in SO101_MOTORS)
SO101_ACTION_KEYS = SO101_STATE_KEYS


def _tokenizer_files(tokenizer_name: str) -> tuple[Path, Path]:
    local_path = Path(tokenizer_name).expanduser()
    if local_path.exists():
        tokenizer_json = local_path / "tokenizer.json" if local_path.is_dir() else local_path
        tokenizer_config = tokenizer_json.parent / "tokenizer_config.json"
    else:
        from huggingface_hub import hf_hub_download

        tokenizer_json = Path(hf_hub_download(tokenizer_name, "tokenizer.json"))
        tokenizer_config = Path(hf_hub_download(tokenizer_name, "tokenizer_config.json"))

    if not tokenizer_json.is_file():
        raise FileNotFoundError(tokenizer_json)
    if not tokenizer_config.is_file():
        raise FileNotFoundError(tokenizer_config)
    return tokenizer_json, tokenizer_config


def _special_token_text(value: str | dict, name: str) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, dict) and isinstance(value.get("content"), str):
        return value["content"]
    raise ValueError(f"tokenizer config has no usable {name}")


def _tokenize(task: str, tokenizer_name: str, max_length: int) -> tuple[torch.Tensor, torch.Tensor]:
    tokenizer_json, tokenizer_config_path = _tokenizer_files(tokenizer_name)
    tokenizer_config = json.loads(tokenizer_config_path.read_text(encoding="utf-8"))
    pad_token = _special_token_text(tokenizer_config.get("pad_token"), "pad_token")

    tokenizer = Tokenizer.from_file(str(tokenizer_json))
    pad_token_id = tokenizer.token_to_id(pad_token)
    if pad_token_id is None:
        raise ValueError(f"pad token {pad_token!r} is missing from {tokenizer_json}")
    tokenizer.enable_truncation(
        max_length=max_length,
        direction=tokenizer_config.get("truncation_side", "right"),
    )
    tokenizer.enable_padding(
        length=max_length,
        direction=tokenizer_config.get("padding_side", "right"),
        pad_id=pad_token_id,
        pad_token=pad_token,
    )
    prompt = task if task.endswith("\n") else f"{task}\n"
    encoded = tokenizer.encode(prompt, add_special_tokens=True)
    input_ids = torch.tensor(encoded.ids, dtype=torch.int64).unsqueeze(0)
    attention_mask = torch.tensor(encoded.attention_mask, dtype=torch.bool).unsqueeze(0)
    return input_ids, attention_mask


class PortableNormalRng:
    """Small deterministic normal RNG shared with the C++ runner."""

    _MASK = (1 << 64) - 1
    _INV_53 = 1.0 / float(1 << 53)

    def __init__(self, seed: int):
        self._state = seed & self._MASK
        self._has_spare = False
        self._spare = 0.0

    def _next_u64(self) -> int:
        self._state = (self._state + 0x9E3779B97F4A7C15) & self._MASK
        value = self._state
        value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & self._MASK
        value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & self._MASK
        return (value ^ (value >> 31)) & self._MASK

    def _uniform_open(self) -> float:
        return ((self._next_u64() >> 11) + 0.5) * self._INV_53

    def _normal(self) -> float:
        if self._has_spare:
            self._has_spare = False
            return self._spare
        u1 = self._uniform_open()
        u2 = self._uniform_open()
        radius = math.sqrt(-2.0 * math.log(u1))
        theta = 2.0 * math.pi * u2
        self._spare = radius * math.sin(theta)
        self._has_spare = True
        return radius * math.cos(theta)

    def standard_normal(self, shape: tuple[int, ...]) -> np.ndarray:
        out = np.empty(int(np.prod(shape)), dtype=np.float32)
        for index in range(out.size):
            out[index] = self._normal()
        return out.reshape(shape)


class LightweightSmolVLARuntime:
    def __init__(self, checkpoint: Path, task: str):
        with (checkpoint / "config.json").open("r", encoding="utf-8") as file:
            cfg = json.load(file)

        self.image_keys = tuple(key for key in cfg["input_features"] if key.startswith("observation.images."))
        self.action_dim = int(cfg["output_features"]["action"]["shape"][0])
        self.chunk_size = int(cfg["chunk_size"])
        self.n_action_steps = int(cfg["n_action_steps"])
        self.max_state_dim = int(cfg["max_state_dim"])
        self.max_action_dim = int(cfg["max_action_dim"])
        self.empty_cameras = int(cfg.get("empty_cameras", 0))
        self.resize_hw = tuple(cfg["resize_imgs_with_padding"])
        self.tokenizer_max_length = int(cfg["tokenizer_max_length"])

        stats = load_file(str(checkpoint / "policy_preprocessor_step_5_normalizer_processor.safetensors"))
        self.state_mean = stats["observation.state.mean"].float()
        self.state_std = stats["observation.state.std"].float()
        self.action_mean = stats["action.mean"].float()
        self.action_std = stats["action.std"].float()

        self.task = task if task.endswith("\n") else f"{task}\n"
        self.lang_tokens, self.lang_masks = _tokenize(self.task, TOKENIZER_NAME, self.tokenizer_max_length)


def _resize_with_pad(img: torch.Tensor, width: int, height: int, pad_value: float = 0.0) -> torch.Tensor:
    if img.ndim != 4:
        raise ValueError(f"(b,c,h,w) expected, got {tuple(img.shape)}")
    cur_height, cur_width = img.shape[2:]
    ratio = max(cur_width / width, cur_height / height)
    resized_height = int(cur_height / ratio)
    resized_width = int(cur_width / ratio)
    resized_img = functional.interpolate(
        img, size=(resized_height, resized_width), mode="bilinear", align_corners=False
    )
    pad_height = max(0, int(height - resized_height))
    pad_width = max(0, int(width - resized_width))
    return functional.pad(resized_img, (pad_width, 0, pad_height, 0), value=pad_value)


def _pad_vector(vector: torch.Tensor, new_dim: int) -> torch.Tensor:
    if vector.shape[-1] == new_dim:
        return vector
    shape = list(vector.shape)
    shape[-1] = new_dim
    padded = torch.zeros(*shape, dtype=vector.dtype, device=vector.device)
    padded[..., : vector.shape[-1]] = vector
    return padded


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--model-dir",
        type=Path,
        default=DEFAULT_MODEL_DIR,
        help="Directory containing 4 ONNX files",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=DEFAULT_CHECKPOINT,
        help="SmolVLA pretrained_model directory",
    )
    parser.add_argument("--task", default=DEFAULT_TASK, help="Natural-language task prompt")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Control-loop frequency")
    parser.add_argument(
        "--episode-time",
        type=float,
        default=DEFAULT_EPISODE_TIME_S,
        help="Online episode length in seconds",
    )
    parser.add_argument("--iters", type=int, default=3, help="Dry-run iterations when --no-robot is used")
    parser.add_argument("--max-iters", type=int, default=None, help="Hard cap for online iterations")
    parser.add_argument("--seed", type=int, default=0)

    parser.add_argument(
        "--no-robot",
        action="store_true",
        help="Dry-run with synthetic observations; do not connect hardware",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Connect hardware and cameras, but do not send actions"
    )
    parser.add_argument("--port", default=None, help="SO101 serial port, e.g. /dev/ttyACM0")
    parser.add_argument(
        "--camera",
        action="append",
        default=[],
        metavar="NAME=INDEX_OR_PATH",
        help="Camera mapping for model image slots, e.g. --camera top=/dev/video3",
    )
    parser.add_argument("--camera-fps", type=int, default=DEFAULT_CAMERA_FPS)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument(
        "--camera-fourcc", default=DEFAULT_CAMERA_FOURCC, help="OpenCV FOURCC, e.g. MJPG or YUYV"
    )
    parser.add_argument("--robot-id", default="my_awesome_follower_arm")

    parser.set_defaults(use_spacemit_ep=True)
    parser.add_argument(
        "--use-spacemit-ep",
        dest="use_spacemit_ep",
        action="store_true",
        help="Use SpaceMITExecutionProvider on K3 (default)",
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
    parser.add_argument(
        "--cpu-vision",
        action="store_true",
        help="Run vision_encoder on CPU even with --use-spacemit-ep",
    )
    parser.add_argument(
        "--cpu-connector", action="store_true", help="Run connector on CPU even with --use-spacemit-ep"
    )
    parser.add_argument(
        "--cpu-prefill", action="store_true", help="Run prefill_lm on CPU even with --use-spacemit-ep"
    )
    parser.add_argument(
        "--cpu-denoise",
        action="store_true",
        help="Run denoise_step on CPU even with --use-spacemit-ep",
    )
    parser.add_argument("--print-actions", action="store_true")
    parser.add_argument("--infer-every-tick", action="store_true", help="Disable action chunk amortization")
    parser.add_argument(
        "--warmup", type=int, default=0, help="Number of synthetic warmup inferences before the loop"
    )
    parser.add_argument(
        "--denoise-steps",
        type=int,
        default=DEFAULT_DENOISE_STEPS,
        help="Number of denoise_step calls per inference",
    )
    parser.add_argument(
        "--n-action-steps",
        type=int,
        default=None,
        help="Number of actions from each inferred chunk to enqueue; default comes from checkpoint",
    )
    parser.add_argument(
        "--include-empty-cameras", action="store_true", help="Append configured empty camera image(s)"
    )
    return parser.parse_args()


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
    original_get_available_providers = pybind_state.get_available_providers
    original_ort_get_available_providers = ort.get_available_providers

    def initialize_session_with_spacemit(*args, **kwargs):
        sess, providers, provider_options, disabled_optimizers = args[:4]
        assert len(providers) == len(provider_options)
        if provider_name in providers:
            ep_index = providers.index(provider_name)
            provider_options[ep_index]["shared_lib_path"] = str(ep_lib)
            provider_options[ep_index]["provider_factory_entry_point"] = entry_point
        if "CPUExecutionProvider" not in providers:
            providers.append("CPUExecutionProvider")
            provider_options.append({})
        return original_initialize_session(*args, **kwargs)

    initialize_session_with_spacemit._spacemit_ep_patched = True

    def get_available_providers_with_spacemit():
        providers = original_get_available_providers()
        if provider_name not in providers:
            providers = [provider_name] + providers
        return providers

    def ort_get_available_providers_with_spacemit():
        providers = original_ort_get_available_providers()
        if provider_name not in providers:
            providers = [provider_name] + providers
        return providers

    pybind_state.InferenceSession.initialize_session = initialize_session_with_spacemit
    pybind_state.get_available_providers = get_available_providers_with_spacemit
    ort.get_available_providers = ort_get_available_providers_with_spacemit


def _resolve_spacemit_ep_library(spacemit_ort_dir: Path | None) -> Path | None:
    if spacemit_ort_dir is None:
        return None

    ort_dir = spacemit_ort_dir.expanduser()
    candidates = (ort_dir / "lib" / "libspacemit_ep.so", ort_dir / "libspacemit_ep.so")
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"cannot find libspacemit_ep.so under {ort_dir}")


def _resolve_checkpoint_path(model_dir: Path, checkpoint: Path) -> Path:
    checkpoint = checkpoint.expanduser()
    if (checkpoint / "config.json").exists():
        return checkpoint

    export_config_path = model_dir.expanduser() / "export_config.json"
    if not export_config_path.exists():
        return checkpoint

    with export_config_path.open("r", encoding="utf-8") as file:
        export_config = json.load(file)

    export_checkpoint = export_config.get("checkpoint")
    if not export_checkpoint:
        return checkpoint

    candidate = Path(export_checkpoint).expanduser()
    if not candidate.is_absolute():
        candidate = (export_config_path.parent / candidate).resolve()

    if (candidate / "config.json").exists():
        print(f"Resolved checkpoint from export_config.json: {candidate}")
        return candidate

    return checkpoint


def _resolve_camera_index(value: str | None) -> int | str | None:
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return value


def _parse_camera_spec(spec: str) -> tuple[str, int | str]:
    name, sep, value = spec.partition("=")
    if sep != "=" or not name or not value:
        raise ValueError(f"Invalid --camera value {spec!r}; expected NAME=INDEX_OR_PATH")
    return name, _resolve_camera_index(value)


def _build_robot(args: argparse.Namespace):
    from lerobot.cameras.opencv import OpenCVCameraConfig
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    if args.port is None:
        raise ValueError("--port is required unless --no-robot is set")

    cameras = {}
    for camera_spec in args.camera:
        camera_name, index_or_path = _parse_camera_spec(camera_spec)
        cameras[camera_name] = OpenCVCameraConfig(
            index_or_path=index_or_path,
            fps=args.camera_fps,
            width=args.camera_width,
            height=args.camera_height,
            fourcc=args.camera_fourcc,
        )
    if not cameras:
        raise ValueError("Provide --camera NAME=INDEX_OR_PATH")

    robot = SO101Follower(
        SO101FollowerConfig(port=args.port, cameras=cameras, id=args.robot_id, use_degrees=True)
    )
    robot.connect()
    return robot


def _build_synthetic_observation(seed: int) -> dict:
    rng = np.random.default_rng(seed)
    obs = {
        "observation.images.top": rng.integers(0, 256, size=(480, 640, 3), dtype=np.uint8),
        "observation.images.wrist": rng.integers(0, 256, size=(480, 640, 3), dtype=np.uint8),
        OBS_STATE: rng.normal(0, 1, size=(6,)).astype(np.float32),
    }
    for camera_index in range(1, 9):
        obs[f"observation.images.camera{camera_index}"] = rng.integers(
            0, 256, size=(480, 640, 3), dtype=np.uint8
        )
    return obs


def _hwc_uint8_to_chw_float(image: np.ndarray) -> np.ndarray:
    if image.dtype == np.uint8:
        image = image.astype(np.float32) / 255.0
    elif image.dtype != np.float32:
        image = image.astype(np.float32)
    if image.ndim == 3 and image.shape[-1] == 3:
        image = np.transpose(image, (2, 0, 1))
    return image


def _build_policy_observation_dict(raw_obs: dict) -> dict:
    if OBS_STATE in raw_obs:
        state = np.asarray(raw_obs[OBS_STATE], dtype=np.float32).reshape(-1)
    else:
        state = np.asarray([float(raw_obs[key]) for key in SO101_STATE_KEYS], dtype=np.float32)

    obs = {OBS_STATE: torch.from_numpy(state)}
    for key, value in raw_obs.items():
        if key == OBS_STATE or key in SO101_STATE_KEYS:
            continue
        image = np.asarray(value)
        if image.ndim < 2:
            continue
        dataset_key = key if key.startswith("observation.images.") else f"observation.images.{key}"
        obs[dataset_key] = torch.from_numpy(_hwc_uint8_to_chw_float(image))
    return obs


def _make_sessions(args: argparse.Namespace) -> dict[str, ort.InferenceSession]:
    spacemit_ep_library = _resolve_spacemit_ep_library(args.spacemit_ort_dir)
    opts = ort.SessionOptions()
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    opts.intra_op_num_threads = args.ep_threads
    if args.use_spacemit_ep:
        if spacemit_ep_library is not None:
            raise RuntimeError(
                "Python SmolVLA does not support loading the patched SpaceMIT EP with "
                "--spacemit-ort-dir. The EP is loaded too late in the Python process and may "
                "fall back to CPU or crash. Use the installed spacemit_ort package for fp32 "
                "Python tests, or use the C++ pipeline with --spacemit-ort-dir for fp16."
            )
        else:
            try:
                import spacemit_ort  # noqa: F401  registers SpaceMITExecutionProvider
            except ImportError as exc:
                raise RuntimeError(
                    "spacemit_ort is not installed and --spacemit-ort-dir was not provided"
                ) from exc
        for key, value in SPACEMIT_EP_ENV_DEFAULTS.items():
            os.environ.setdefault(key, value)
        ep_providers = ["SpaceMITExecutionProvider", "CPUExecutionProvider"]
        ep_provider_options = [
            {
                "SPACEMIT_EP_INTRA_THREAD_NUM": str(args.ep_threads),
                "SPACEMIT_EP_INTRA_THREAD_AFFINITY": args.ep_affinity,
                **(
                    {
                        "shared_lib_path": str(spacemit_ep_library),
                        "provider_factory_entry_point": "GetSpaceMITSharedProviderFactory",
                    }
                    if spacemit_ep_library is not None
                    else {}
                ),
            },
            {},
        ]
    else:
        ep_providers = ["CPUExecutionProvider"]
        ep_provider_options = None

    cpu_providers = ["CPUExecutionProvider"]
    cpu_provider_options = None
    force_cpu = {
        "vision_encoder": args.cpu_vision,
        "connector": args.cpu_connector,
        "prefill_lm": args.cpu_prefill,
        "denoise_step": args.cpu_denoise,
    }

    sessions = {}
    for name in ("vision_encoder", "connector", "prefill_lm", "denoise_step"):
        model_path = args.model_dir / f"{name}.onnx"
        if not model_path.exists():
            raise FileNotFoundError(model_path)
        if force_cpu[name]:
            providers = cpu_providers
            provider_options = cpu_provider_options
            assignment = "CPU"
        else:
            providers = ep_providers
            provider_options = ep_provider_options
            assignment = "EP" if args.use_spacemit_ep else "CPU"
        start = time.perf_counter()
        sessions[name] = ort.InferenceSession(
            str(model_path), opts, providers=providers, provider_options=provider_options
        )
        elapsed_ms = (time.perf_counter() - start) * 1000
        print(
            f"  {name:<14} loaded in {elapsed_ms:8.0f} ms  "
            f"assignment={assignment}  providers={sessions[name].get_providers()}"
        )
    return sessions


def _static_dim(dim: int | str | None) -> int | None:
    return dim if isinstance(dim, int) and dim > 0 else None


def _infer_model_camera_count(sessions: dict[str, ort.InferenceSession]) -> int | None:
    connector_shape = sessions["connector"].get_outputs()[0].shape
    prefill_shape = sessions["prefill_lm"].get_inputs()[0].shape
    tokens_per_camera = _static_dim(connector_shape[1] if len(connector_shape) > 1 else None)
    expected_tokens = _static_dim(prefill_shape[1] if len(prefill_shape) > 1 else None)
    if tokens_per_camera is None or expected_tokens is None:
        return None
    if expected_tokens % tokens_per_camera != 0:
        raise ValueError(
            f"Cannot derive camera count: prefill image tokens={expected_tokens}, connector tokens={tokens_per_camera}"
        )
    return expected_tokens // tokens_per_camera


def _run_4model_pipeline(
    sessions: dict[str, ort.InferenceSession],
    images: torch.Tensor,
    lang_tokens: torch.Tensor,
    lang_masks: torch.Tensor,
    state: torch.Tensor,
    noise: np.ndarray,
    cached_image_embs: dict[int, np.ndarray] | None = None,
    num_steps: int = 10,
) -> tuple[np.ndarray, dict[str, float]]:
    timings: dict[str, float] = {}

    vision = sessions["vision_encoder"]
    connector = sessions["connector"]
    prefill = sessions["prefill_lm"]
    denoise = sessions["denoise_step"]

    start = time.perf_counter()
    image_embs = []
    for camera_index in range(images.shape[1]):
        if cached_image_embs is not None and camera_index in cached_image_embs:
            image_embs.append(cached_image_embs[camera_index])
        else:
            image = images[:, camera_index].cpu().numpy().astype(np.float32, copy=False)
            hidden = vision.run(None, {vision.get_inputs()[0].name: image})[0]
            image_embs.append(connector.run(None, {connector.get_inputs()[0].name: hidden})[0])
    image_embs_np = np.concatenate(image_embs, axis=1).astype(np.float32, copy=False)
    timings["vision_connector"] = (time.perf_counter() - start) * 1000

    start = time.perf_counter()
    prefill_inputs = {
        prefill.get_inputs()[0].name: image_embs_np,
        prefill.get_inputs()[1].name: lang_tokens.cpu().numpy().astype(np.int64, copy=False),
        prefill.get_inputs()[2].name: lang_masks.cpu().numpy().astype(np.int64, copy=False),
        prefill.get_inputs()[3].name: state.cpu().numpy().astype(np.float32, copy=False),
    }
    prefill_outputs = prefill.run(None, prefill_inputs)
    if len(prefill_outputs) == 3:
        past_keys, past_values, prefix_pad_masks = prefill_outputs
    elif len(prefill_outputs) == 33:
        key_layers = [np.asarray(output, dtype=np.float32) for output in prefill_outputs[:16]]
        value_layers = [np.asarray(output, dtype=np.float32) for output in prefill_outputs[16:32]]

        def stack_kv(layers: list[np.ndarray]) -> np.ndarray:
            if layers[0].ndim == 4:
                return np.stack(layers, axis=0)
            if layers[0].ndim == 5 and layers[0].shape[0] == 1:
                return np.concatenate(layers, axis=0)
            raise ValueError(f"Unexpected split prefill KV rank: {layers[0].ndim}")

        past_keys = stack_kv(key_layers)
        past_values = stack_kv(value_layers)
        prefix_pad_masks = prefill_outputs[32]
    else:
        raise ValueError(f"Unsupported prefill output count: {len(prefill_outputs)}")
    timings["prefill"] = (time.perf_counter() - start) * 1000

    start = time.perf_counter()
    x_t = noise.astype(np.float32, copy=False)
    dt = np.float32(-1.0 / num_steps)
    for step in range(num_steps):
        timestep = np.asarray([1.0 + step * float(dt)], dtype=np.float32)
        denoise_inputs = {
            denoise.get_inputs()[0].name: x_t,
            denoise.get_inputs()[1].name: timestep,
            denoise.get_inputs()[2].name: prefix_pad_masks,
            denoise.get_inputs()[3].name: past_keys,
            denoise.get_inputs()[4].name: past_values,
        }
        v_t = np.asarray(denoise.run(None, denoise_inputs)[0], dtype=np.float32)
        x_t = x_t + dt * v_t
    timings["denoise"] = (time.perf_counter() - start) * 1000
    timings["inference"] = timings["vision_connector"] + timings["prefill"] + timings["denoise"]
    return x_t, timings


def _postprocess_actions(runtime: LightweightSmolVLARuntime, actions_normalized: np.ndarray) -> np.ndarray:
    actions_tensor = torch.from_numpy(
        actions_normalized[:, :, : runtime.action_dim].astype(np.float32, copy=False)
    )
    actions = actions_tensor * runtime.action_std + runtime.action_mean
    return actions.detach().cpu().numpy()


def _summarize_ms(name: str, values: list[float]) -> None:
    arr = np.asarray(values, dtype=np.float64)
    if arr.size > 1:
        arr = arr[1:]
    print(
        f"  {name:<16} mean={arr.mean():8.2f}  min={arr.min():8.2f}  "
        f"median={np.median(arr):8.2f}  max={arr.max():8.2f} ms"
    )


def main() -> int:  # noqa: C901
    args = parse_args()
    args.model_dir = args.model_dir.expanduser()
    args.checkpoint = _resolve_checkpoint_path(args.model_dir, args.checkpoint)
    if args.denoise_steps <= 0:
        raise ValueError("--denoise-steps must be positive")
    if args.n_action_steps is not None and args.n_action_steps <= 0:
        raise ValueError("--n-action-steps must be positive")
    os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    print(f"Loading lightweight checkpoint metadata: {args.checkpoint}")
    runtime = LightweightSmolVLARuntime(args.checkpoint, args.task)
    chunk_size = runtime.chunk_size
    n_action_steps = min(args.n_action_steps or runtime.n_action_steps, chunk_size)
    max_action_dim = runtime.max_action_dim
    print(
        f"  cameras={list(runtime.image_keys)}  action_dim={runtime.action_dim}  "
        f"chunk_size={chunk_size}  n_action_steps={n_action_steps}  denoise_steps={args.denoise_steps}"
    )

    print(f"Loading FP32 ONNX 4-model pipeline from: {args.model_dir}")
    sessions = _make_sessions(args)
    model_camera_count = _infer_model_camera_count(sessions)
    if model_camera_count is None:
        model_camera_count = 2 + (runtime.empty_cameras if args.include_empty_cameras else 0)
        print(f"  model camera_count could not be inferred, using {model_camera_count}")
    else:
        print(f"  model camera_count={model_camera_count}")

    robot = None
    if args.no_robot:
        print("Running in --no-robot dry-run mode.")
    else:
        print("Connecting SO101 follower ...")
        robot = _build_robot(args)
        print(f"  connected: {robot}")
        if args.dry_run:
            print("  dry-run: actions will not be sent to motors")

    noise_rng = PortableNormalRng(args.seed)
    action_queue: deque[np.ndarray] = deque()
    timing_keys = (
        "obs",
        "preprocess",
        "vision_connector",
        "prefill",
        "denoise",
        "inference",
        "postprocess",
        "send",
        "total",
    )
    timings = {key: [] for key in timing_keys}

    def prepare_feed(raw_obs: dict) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        obs_for_policy = _build_policy_observation_dict(raw_obs)
        with torch.no_grad():
            mapped_images = {
                key: value for key, value in obs_for_policy.items() if key.startswith("observation.images.")
            }
            legacy_aliases = {
                "observation.images.camera1": "observation.images.top",
                "observation.images.camera2": "observation.images.wrist",
                "observation.images.top": "observation.images.camera1",
                "observation.images.wrist": "observation.images.camera2",
            }
            for dst_key, src_key in legacy_aliases.items():
                if dst_key not in mapped_images and src_key in mapped_images:
                    mapped_images[dst_key] = mapped_images[src_key]

            images_list = []
            reference_img = None
            required_image_keys = runtime.image_keys[:model_camera_count]
            missing_image_keys = []
            for image_key in required_image_keys:
                img = mapped_images.get(image_key)
                if img is None:
                    missing_image_keys.append(image_key)
                    continue
                img = img.unsqueeze(0)
                img = _resize_with_pad(img, *runtime.resize_hw, pad_value=0)
                img = img * 2.0 - 1.0
                reference_img = img
                images_list.append(img)
            if missing_image_keys:
                available_image_keys = sorted(mapped_images)
                raise ValueError(
                    "Missing camera image(s): "
                    + ", ".join(missing_image_keys)
                    + f"; available={available_image_keys}"
                )
            if not images_list:
                raise ValueError("No camera images found in observation")
            while len(images_list) < model_camera_count:
                images_list.append(torch.ones_like(reference_img) * -1)
            images = torch.stack(images_list, dim=1)

            state_raw = obs_for_policy[OBS_STATE].unsqueeze(0)
            state_norm = (state_raw - runtime.state_mean) / (runtime.state_std + 1e-8)
            state = _pad_vector(state_norm, runtime.max_state_dim)

            lang_tokens = runtime.lang_tokens
            lang_masks = runtime.lang_masks
        return images, lang_tokens, lang_masks, state

    empty_image_embs: dict[int, np.ndarray] = {}
    empty_camera_start = min(len(runtime.image_keys), model_camera_count)
    if model_camera_count > empty_camera_start:
        print("Caching empty camera embedding(s) ...")
        images, _, _, _ = prepare_feed(_build_synthetic_observation(args.seed))
        vision = sessions["vision_encoder"]
        connector = sessions["connector"]
        for camera_index in range(empty_camera_start, images.shape[1]):
            image = images[:, camera_index].cpu().numpy().astype(np.float32, copy=False)
            hidden = vision.run(None, {vision.get_inputs()[0].name: image})[0]
            empty_image_embs[camera_index] = connector.run(None, {connector.get_inputs()[0].name: hidden})[0]

    if args.warmup > 0:
        print(f"Running {args.warmup} synthetic warmup inference(s) ...")
        for warmup_index in range(args.warmup):
            images, lang_tokens, lang_masks, state = prepare_feed(
                _build_synthetic_observation(args.seed + warmup_index)
            )
            noise = noise_rng.standard_normal((1, chunk_size, max_action_dim))
            _run_4model_pipeline(
                sessions, images, lang_tokens, lang_masks, state, noise, empty_image_embs, args.denoise_steps
            )

    max_iters = args.iters if args.no_robot else args.max_iters or int(args.fps * args.episode_time)
    period = 1.0 / args.fps if args.fps > 0 else 0.0
    print(f"\nStarting grasp loop: max_iters={max_iters}  fps={args.fps}  task={args.task!r}\n")

    try:
        for iteration in range(max_iters):
            tick_start = time.perf_counter()

            start = time.perf_counter()
            raw_obs = (
                robot.get_observation()
                if robot is not None
                else _build_synthetic_observation(args.seed + iteration)
            )
            timings["obs"].append((time.perf_counter() - start) * 1000)

            start = time.perf_counter()
            images, lang_tokens, lang_masks, state = prepare_feed(raw_obs)
            timings["preprocess"].append((time.perf_counter() - start) * 1000)

            if not action_queue or args.infer_every_tick:
                noise = noise_rng.standard_normal((1, chunk_size, max_action_dim))
                actions_normalized, infer_timings = _run_4model_pipeline(
                    sessions,
                    images,
                    lang_tokens,
                    lang_masks,
                    state,
                    noise,
                    empty_image_embs,
                    args.denoise_steps,
                )
                for key in ("vision_connector", "prefill", "denoise", "inference"):
                    timings[key].append(infer_timings[key])

                start = time.perf_counter()
                actions_unnorm = _postprocess_actions(runtime, actions_normalized)
                if args.infer_every_tick:
                    action_queue.clear()
                for step in range(min(n_action_steps, actions_unnorm.shape[1])):
                    action_queue.append(actions_unnorm[0, step])
                timings["postprocess"].append((time.perf_counter() - start) * 1000)
            else:
                for key in ("vision_connector", "prefill", "denoise", "inference", "postprocess"):
                    timings[key].append(0.0)

            current_action = action_queue.popleft()
            if not np.all(np.isfinite(current_action[: runtime.action_dim])):
                raise RuntimeError("non-finite SmolVLA action")
            action_dict = {
                key: float(value) for key, value in zip(SO101_ACTION_KEYS, current_action, strict=True)
            }

            start = time.perf_counter()
            if robot is not None and not args.dry_run:
                robot.send_action(action_dict)
            timings["send"].append((time.perf_counter() - start) * 1000)
            timings["total"].append((time.perf_counter() - tick_start) * 1000)

            action_text = ""
            if args.print_actions:
                action_text = f" action={[round(float(v), 3) for v in current_action.tolist()]}"
            print(
                f"[iter {iteration:3d}] total={timings['total'][-1]:8.1f} ms  "
                f"obs={timings['obs'][-1]:6.1f}  pre={timings['preprocess'][-1]:6.1f}  "
                f"vc={timings['vision_connector'][-1]:8.1f}  pf={timings['prefill'][-1]:8.1f}  "
                f"dn={timings['denoise'][-1]:8.1f}  post={timings['postprocess'][-1]:6.1f}  "
                f"send={timings['send'][-1]:6.1f}{action_text}",
                flush=True,
            )

            if period > 0 and not args.no_robot:
                slack = period - (time.perf_counter() - tick_start)
                if slack > 0:
                    time.sleep(slack)
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception as exc:  # noqa: BLE001
                logging.warning("robot.disconnect() failed: %s", exc)

    print("\n" + "=" * 78)
    print("4-model FP32 grasp pipeline latency summary")
    print("=" * 78)
    print(f"iters={len(timings['total'])}")
    for key in timing_keys:
        if timings[key]:
            _summarize_ms(key, timings[key])
    if len(timings["total"]) > 1:
        steady = np.asarray(timings["total"][1:], dtype=np.float64)
        print(f"\nSteady loop throughput: {1000.0 / steady.mean():.3f} FPS  mean={steady.mean():.1f} ms")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
