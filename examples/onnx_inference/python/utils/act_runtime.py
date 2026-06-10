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


def build_session(
    model_path: Path,
    use_spacemit_ep: bool,
    ep_threads: int,
    ep_affinity: str,
    cpu_threads: int = 0,
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
        providers = ["SpaceMITExecutionProvider", "CPUExecutionProvider"]
        provider_options = [
            {
                "SPACEMIT_EP_INTRA_THREAD_NUM": str(ep_threads),
                "SPACEMIT_EP_INTRA_THREAD_AFFINITY": ep_affinity,
            },
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
