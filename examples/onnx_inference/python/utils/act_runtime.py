"""ONNX Runtime helpers shared by ACT inference and benchmark scripts."""

from __future__ import annotations

import ctypes
import glob
import os
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort

try:
    import spacemit_ort  # noqa: F401
except ImportError:
    spacemit_ort = None

ACT_MODEL_FILENAMES = ("act.onnx", "act.q.onnx")
SPACEMIT_PROVIDER_NAME = "SpaceMITExecutionProvider"
SPACEMIT_PROVIDER_FACTORY = "GetSpaceMITSharedProviderFactory"


def resolve_spacemit_ep_library(spacemit_ort_dir: Path | None) -> Path | None:
    if spacemit_ort_dir is None:
        return None

    ort_dir = spacemit_ort_dir.expanduser()
    candidates = (ort_dir / "lib" / "libspacemit_ep.so", ort_dir / "libspacemit_ep.so")
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"cannot find libspacemit_ep.so under {ort_dir}")


def resolve_act_model_path(model_dir: Path) -> Path:
    if model_dir is None:
        raise ValueError("pass --model-dir")

    model_dir = model_dir.expanduser()
    candidates = [model_dir / name for name in ACT_MODEL_FILENAMES]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    names = ", ".join(ACT_MODEL_FILENAMES)
    raise FileNotFoundError(f"cannot find ACT ONNX model in {model_dir}; expected one of: {names}")


def _preload_onnxruntime_shared() -> None:
    capi_dir = os.path.join(
        os.path.dirname(os.path.dirname(ort.__file__)),
        "onnxruntime",
        "capi",
    )
    for pattern in ("libonnxruntime.so.*", "libonnxruntime.so"):
        for lib in glob.glob(os.path.join(capi_dir, pattern)):
            try:
                ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
                return
            except OSError:
                continue


def install_spacemit_ep_shared_library(ep_lib: Path) -> None:
    import onnxruntime.capi._pybind_state as pybind_state

    ep_lib = ep_lib.expanduser()
    if not ep_lib.exists():
        raise FileNotFoundError(ep_lib)

    if getattr(pybind_state.InferenceSession.initialize_session, "_spacemit_ep_patched", False):
        return

    original_initialize_session = pybind_state.InferenceSession.initialize_session
    original_get_available_providers = pybind_state.get_available_providers
    original_ort_get_available_providers = ort.get_available_providers

    def initialize_session_with_spacemit(*args, **kwargs):
        _session, providers, provider_options, _disabled_optimizers = args[:4]
        if SPACEMIT_PROVIDER_NAME in providers:
            ep_index = providers.index(SPACEMIT_PROVIDER_NAME)
            provider_options[ep_index]["shared_lib_path"] = str(ep_lib)
            provider_options[ep_index]["provider_factory_entry_point"] = SPACEMIT_PROVIDER_FACTORY
        if "CPUExecutionProvider" not in providers:
            providers.append("CPUExecutionProvider")
            provider_options.append({})
        return original_initialize_session(*args, **kwargs)

    initialize_session_with_spacemit._spacemit_ep_patched = True

    def get_available_providers_with_spacemit():
        providers = original_get_available_providers()
        if SPACEMIT_PROVIDER_NAME not in providers:
            providers = [SPACEMIT_PROVIDER_NAME] + providers
        return providers

    def ort_get_available_providers_with_spacemit():
        providers = original_ort_get_available_providers()
        if SPACEMIT_PROVIDER_NAME not in providers:
            providers = [SPACEMIT_PROVIDER_NAME] + providers
        return providers

    pybind_state.InferenceSession.initialize_session = initialize_session_with_spacemit
    pybind_state.get_available_providers = get_available_providers_with_spacemit
    ort.get_available_providers = ort_get_available_providers_with_spacemit


def build_session(
    model_path: Path,
    use_spacemit_ep: bool,
    ep_threads: int,
    ep_affinity: str,
    cpu_threads: int = 0,
    spacemit_ep_library: Path | None = None,
) -> tuple[ort.InferenceSession, float]:
    _preload_onnxruntime_shared()

    sess_options = ort.SessionOptions()
    sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    if cpu_threads > 0:
        sess_options.intra_op_num_threads = cpu_threads

    providers = ["CPUExecutionProvider"]
    provider_options = [{}]
    if use_spacemit_ep:
        if spacemit_ort is None:
            raise RuntimeError("spacemit_ort is not installed; cannot use SpaceMITExecutionProvider")
        if spacemit_ep_library is not None:
            install_spacemit_ep_shared_library(spacemit_ep_library)
        ep_options = {
            "SPACEMIT_EP_INTRA_THREAD_NUM": str(ep_threads),
            "SPACEMIT_EP_INTRA_THREAD_AFFINITY": ep_affinity,
        }
        if spacemit_ep_library is not None:
            ep_options["shared_lib_path"] = str(spacemit_ep_library.expanduser())
            ep_options["provider_factory_entry_point"] = SPACEMIT_PROVIDER_FACTORY
        providers = [SPACEMIT_PROVIDER_NAME, "CPUExecutionProvider"]
        provider_options = [
            ep_options,
            {},
        ]

    start = time.perf_counter()
    session = ort.InferenceSession(
        str(model_path),
        sess_options,
        providers=providers,
        provider_options=provider_options,
    )
    elapsed_ms = (time.perf_counter() - start) * 1000
    return session, elapsed_ms


def random_float(shape: tuple[int, ...], dtype=np.float32) -> np.ndarray:
    return np.random.randn(*shape).astype(dtype)


def input_float_dtype(
    session: ort.InferenceSession,
    input_index: int,
) -> type[np.float16] | type[np.float32]:
    input_type = session.get_inputs()[input_index].type
    if input_type == "tensor(float16)":
        return np.float16
    return np.float32


def _hwc_uint8_to_chw_float(image: np.ndarray) -> np.ndarray:
    if image.dtype == np.uint8:
        image = image.astype(np.float32) / 255.0
    elif image.dtype != np.float32:
        image = image.astype(np.float32)
    if image.ndim == 3 and image.shape[-1] == 3:
        image = np.transpose(image, (2, 0, 1))
    return image
