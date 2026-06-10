#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""ACT real-hardware evaluation entry (SO-101 follower).

Runs an exported ACT ONNX graph (see tools/act_pytorch_to_onnx.py) on a live
SO-101 follower arm + two OpenCV cameras, reproducing the parts that live
OUTSIDE the ONNX graph in pure numpy:
  - input normalization   (images MEAN_STD, state MEAN_STD)
  - output unnormalization (action MEAN_STD)
  - the action queue       (predict a chunk, then pop one action per step)

The normalization statistics are read directly from the training checkpoint's
safetensors (no torch / no lerobot processor needed for the math), so the same
stats can also be exported for the C++ twin (see tools/export_norm_stats.py).

This mirrors the reference command:
  lerobot-record --robot.type=so101_follower --robot.port=/dev/ttyACM0 \
      --robot.cameras="{top: {type: opencv, index_or_path: 13, ...}, \
                        wrist: {type: opencv, index_or_path: 15, ...}}" \
      --policy.path=<checkpoint> ...

Example:
    python act_evaluate.py \
        --onnx models/onnx/act-fp32/act.onnx \
        --checkpoint models/pytorch/act/checkpoints/100000/pretrained_model \
        --port /dev/ttyACM0 \
        --cam top=15 --cam wrist=13 \
        --task "Place the green cube into the box" \
        --episode-time 180 --fps 30

    # SpaceMIT EP on K3 (must run inside the venv that has spacemit_ort)
    python act_evaluate.py ... --use-spacemit-ep --ep-threads 8 \
        --ep-affinity "8;9;10;11;12;13;14;15"
"""

from __future__ import annotations

import argparse
import json
import time
from collections import deque
from pathlib import Path

import numpy as np
from utils import safe_disconnect, suspend_sigint

EXAMPLE_DIR = Path(__file__).resolve().parent

# Feature order is fixed by the training config (observation.state / action are
# the 6 SO-101 motors in this order). Cameras are stacked in config.image_features
# order, which is the JSON insertion order of input_features.
MOTOR_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


# --------------------------------------------------------------------------- #
# Checkpoint introspection (config + normalization stats), torch-free.
# --------------------------------------------------------------------------- #
def read_act_meta(checkpoint: Path) -> dict:
    """Read shapes / chunk_size / camera order from the checkpoint config.json."""
    cfg = json.loads((checkpoint / "config.json").read_text(encoding="utf-8"))
    image_keys = [k for k, v in cfg["input_features"].items() if v["type"] == "VISUAL"]
    state_dim = cfg["input_features"]["observation.state"]["shape"][0]
    action_dim = cfg["output_features"]["action"]["shape"][0]
    # camera short name = key without the "observation.images." prefix
    cam_names = [k.split("observation.images.")[-1] for k in image_keys]
    img_shape = cfg["input_features"][image_keys[0]]["shape"]  # [C, H, W]
    return {
        "image_keys": image_keys,
        "cam_names": cam_names,
        "state_dim": state_dim,
        "action_dim": action_dim,
        "chunk_size": cfg["chunk_size"],
        "n_action_steps": cfg["n_action_steps"],
        "img_c": img_shape[0],
        "img_h": img_shape[1],
        "img_w": img_shape[2],
    }


def _load_safetensors_np(path: Path) -> dict[str, np.ndarray]:
    """Minimal safetensors reader (numpy framework) without importing torch."""
    from safetensors import safe_open

    out: dict[str, np.ndarray] = {}
    with safe_open(str(path), framework="np") as sf:
        for k in sf:
            out[k] = sf.get_tensor(k)
    return out


def load_norm_stats(checkpoint: Path, meta: dict) -> dict:
    """Load MEAN_STD stats for state, each image, and the action unnormalizer.

    Returns float32 arrays:
      state_mean/std        (state_dim,)
      action_mean/std       (action_dim,)
      image_mean/std[name]  (3,1,1) per camera
    """
    pre = _load_safetensors_np(checkpoint / "policy_preprocessor_step_3_normalizer_processor.safetensors")
    post = _load_safetensors_np(checkpoint / "policy_postprocessor_step_0_unnormalizer_processor.safetensors")

    def f32(d, key):
        return np.asarray(d[key], dtype=np.float32)

    stats = {
        "state_mean": f32(pre, "observation.state.mean"),
        "state_std": f32(pre, "observation.state.std"),
        "action_mean": f32(post, "action.mean"),
        "action_std": f32(post, "action.std"),
        "image_mean": {},
        "image_std": {},
    }
    for key, name in zip(meta["image_keys"], meta["cam_names"], strict=True):
        stats["image_mean"][name] = f32(pre, f"{key}.mean").reshape(3, 1, 1)
        stats["image_std"][name] = f32(pre, f"{key}.std").reshape(3, 1, 1)
    return stats


# --------------------------------------------------------------------------- #
# Preprocessing (numpy) — must match what the ONNX graph expects as input.
# --------------------------------------------------------------------------- #
def preprocess_image(img_hwc_uint8: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    """HWC uint8 RGB -> normalized CHW float32: (x/255 - mean) / std."""
    x = img_hwc_uint8.astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))  # HWC -> CHW
    x = (x - mean) / std
    return x


def preprocess_state(state_vec: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    """(state - mean) / std."""
    return (state_vec.astype(np.float32) - mean) / std


def unnormalize_action(action: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    """action * std + mean."""
    return action.astype(np.float32) * std + mean


# --------------------------------------------------------------------------- #
# ONNX session
# --------------------------------------------------------------------------- #
def build_session(
    onnx_path: Path, use_spacemit_ep: bool, ep_threads: int, ep_affinity: str, cpu_threads: int
):
    import onnxruntime as ort

    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    if cpu_threads > 0:
        so.intra_op_num_threads = cpu_threads

    providers: list = ["CPUExecutionProvider"]
    provider_options: list = [{}]
    if use_spacemit_ep:
        # Registering the EP requires importing spacemit_ort first; it does not
        # show up in get_available_providers() until then.
        import spacemit_ort  # noqa: F401

        opts = {"SPACEMIT_EP_INTRA_THREAD_NUM": str(ep_threads)}
        if ep_affinity:
            opts["SPACEMIT_EP_INTRA_THREAD_AFFINITY"] = ep_affinity
        providers = ["SpaceMITExecutionProvider", "CPUExecutionProvider"]
        provider_options = [opts, {}]

    sess = ort.InferenceSession(
        str(onnx_path), sess_options=so, providers=providers, provider_options=provider_options
    )
    in_names = [i.name for i in sess.get_inputs()]
    out_names = [o.name for o in sess.get_outputs()]
    return sess, in_names, out_names


# --------------------------------------------------------------------------- #
# Robot
# --------------------------------------------------------------------------- #
def build_robot(
    port: str, robot_id: str, cam_map: dict[str, int], width: int, height: int, fps: int, fourcc: str
):
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
    from lerobot.robots.so_follower.so_follower import SO101Follower

    cameras = {
        name: OpenCVCameraConfig(index_or_path=index, fps=fps, width=width, height=height, fourcc=fourcc)
        for name, index in cam_map.items()
    }
    cfg = SO101FollowerConfig(port=port, id=robot_id, cameras=cameras)
    return SO101Follower(cfg)


def build_state_vector(obs: dict) -> np.ndarray:
    """Pull the 6 motor positions out of the observation dict, in MOTOR_ORDER."""
    return np.array([obs[f"{m}.pos"] for m in MOTOR_ORDER], dtype=np.float32)


def build_action_dict(action_vec: np.ndarray) -> dict:
    """Map the 6-D action vector back to {motor.pos: float} for send_action."""
    return {f"{m}.pos": float(action_vec[i]) for i, m in enumerate(MOTOR_ORDER)}


# --------------------------------------------------------------------------- #
# Inference (single chunk) + queue
# --------------------------------------------------------------------------- #
def predict_chunk(sess, in_names, out_names, images_nchw, state_vec, meta, stats):
    """Run one ONNX forward and return the unnormalized action chunk (T, A)."""
    feeds = {}
    if "images" in in_names:
        feeds["images"] = images_nchw[None]  # (1, n_cam, 3, H, W)
    if "state" in in_names:
        feeds["state"] = preprocess_state(state_vec, stats["state_mean"], stats["state_std"])[None]
    raw = sess.run(out_names, feeds)[0]  # (1, chunk, action_dim), normalized
    chunk = unnormalize_action(raw[0], stats["action_mean"], stats["action_std"])
    return chunk  # (chunk_size, action_dim)


def run_episode(robot, sess, in_names, out_names, meta, stats, args):
    n_action_steps = min(args.n_action_steps or meta["n_action_steps"], meta["chunk_size"])
    queue: deque[np.ndarray] = deque(maxlen=n_action_steps)

    inference_ms: list[float] = []
    loop_ms: list[float] = []
    t_start = time.perf_counter()
    step = 0

    print(
        f"[ACT ONNX] starting episode: fps={args.fps} "
        f"n_action_steps={n_action_steps} chunk_size={meta['chunk_size']}"
    )
    try:
        while time.perf_counter() - t_start < args.episode_time:
            loop_t0 = time.perf_counter()
            state_vec = None

            if not queue:
                obs = robot.get_observation()

                # Build inputs only when a new action chunk is needed.
                state_vec = build_state_vector(obs)
                images = np.stack(
                    [
                        preprocess_image(
                            obs[name],
                            stats["image_mean"][name],
                            stats["image_std"][name],
                        )
                        for _key, name in zip(
                            meta["image_keys"],
                            meta["cam_names"],
                            strict=True,
                        )
                    ],
                    axis=0,
                ).astype(np.float32)  # (n_cam, 3, H, W)

                inf_t0 = time.perf_counter()
                chunk = predict_chunk(sess, in_names, out_names, images, state_vec, meta, stats)
                inference_ms.append((time.perf_counter() - inf_t0) * 1e3)
                for a in chunk[:n_action_steps]:
                    queue.append(a)

            action_vec = queue.popleft()
            robot.send_action(build_action_dict(action_vec))

            if args.verbose:
                msg = f"[step {step:4d}] action={np.round(action_vec, 2).tolist()}"
                if state_vec is not None:
                    msg = (
                        f"[step {step:4d}] state={np.round(state_vec, 2).tolist()} "
                        f"action={np.round(action_vec, 2).tolist()}"
                    )
                print(msg, flush=True)

            loop_ms.append((time.perf_counter() - loop_t0) * 1e3)
            step += 1
            # Pace to the requested control frequency.
            sleep_s = 1.0 / args.fps - (time.perf_counter() - loop_t0)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("\n[ACT ONNX] interrupted by user.")

    _report(inference_ms, loop_ms, step)


def _report(inference_ms, loop_ms, steps):
    print(f"\n[ACT ONNX] episode done: {steps} steps")
    if inference_ms:
        a = np.array(inference_ms)
        print(
            f"  inference (chunk): n={len(a)} mean={a.mean():.1f}ms "
            f"median={np.median(a):.1f}ms min={a.min():.1f}ms max={a.max():.1f}ms"
        )
    if loop_ms:
        b = np.array(loop_ms)
        print(
            f"  loop: mean={b.mean():.1f}ms median={np.median(b):.1f}ms "
            f"(target {1000.0 / max(b.mean(), 1e-9):.1f} -> effective fps)"
        )


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _parse_cam(values: list[str]) -> dict[str, int]:
    """--cam name=index pairs -> {name: index}."""
    out: dict[str, int] = {}
    for v in values or []:
        if "=" not in v:
            raise argparse.ArgumentTypeError(f"--cam expects name=index, got '{v}'")
        name, idx = v.split("=", 1)
        out[name.strip()] = int(idx)
    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-robot ONNX ACT control (SO-101).")
    # model / stats
    p.add_argument("--onnx", type=Path, required=True, help="Path to act.onnx")
    p.add_argument(
        "--checkpoint",
        type=Path,
        required=True,
        help="pretrained_model dir (for config.json + norm stats safetensors)",
    )
    # robot
    p.add_argument("--port", default="/dev/ttyACM0", help="SO-101 serial port")
    p.add_argument(
        "--robot-id", default="my_awesome_follower_arm", help="Robot id (selects the calibration file)"
    )
    p.add_argument(
        "--cam",
        action="append",
        metavar="NAME=INDEX",
        help="Camera mapping, e.g. --cam top=13 --cam wrist=15 "
        "(must cover all VISUAL features; default top=13 wrist=15)",
    )
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fourcc", default="MJPG")
    p.add_argument(
        "--cam-fps",
        type=int,
        default=30,
        help="Camera capture fps. Keep this at a camera-supported value, "
        "e.g. 30; --fps controls action playback speed.",
    )
    # control loop
    p.add_argument(
        "--fps", type=float, default=30.0, help="Action playback/control frequency. Independent of --cam-fps."
    )
    p.add_argument("--episode-time", type=float, default=180.0, help="seconds")
    p.add_argument(
        "--n-action-steps",
        type=int,
        default=0,
        help="Actions consumed per predicted chunk (0 = config default)",
    )
    p.add_argument("--task", default="", help="Task description (logging only)")
    # ONNX runtime
    p.add_argument("--use-spacemit-ep", action="store_true")
    p.add_argument("--ep-threads", type=int, default=8)
    p.add_argument("--ep-affinity", default="8;9;10;11;12;13;14;15")
    p.add_argument("--cpu-threads", type=int, default=0, help="0 = ORT default")
    p.add_argument("--verbose", action="store_true", help="Print per-step state/action")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cam_map = _parse_cam(args.cam) if args.cam else {"top": 13, "wrist": 15}

    meta = read_act_meta(args.checkpoint)
    stats = load_norm_stats(args.checkpoint, meta)
    print(
        f"[ACT ONNX] cameras={meta['cam_names']} state_dim={meta['state_dim']} "
        f"action_dim={meta['action_dim']} img={meta['img_c']}x{meta['img_h']}x{meta['img_w']}"
    )
    missing = [c for c in meta["cam_names"] if c not in cam_map]
    if missing:
        raise SystemExit(f"Missing --cam mapping for camera(s): {missing}")

    sess, in_names, out_names = build_session(
        args.onnx, args.use_spacemit_ep, args.ep_threads, args.ep_affinity, args.cpu_threads
    )
    print(
        f"[ACT ONNX] ONNX inputs={in_names} outputs={out_names} "
        f"provider={'SpaceMIT EP' if args.use_spacemit_ep else 'CPU'}"
    )

    robot = build_robot(args.port, args.robot_id, cam_map, args.width, args.height, args.cam_fps, args.fourcc)
    print(f"[ACT ONNX] connecting robot on {args.port} (id={args.robot_id}) ...")
    try:
        robot.connect()
        if args.task:
            print(f"[ACT ONNX] task: {args.task}")
        run_episode(robot, sess, in_names, out_names, meta, stats, args)
    except KeyboardInterrupt:
        # Ctrl+C during connect() (before run_episode's own handler is active),
        # or anything that escaped the loop — fall through to guaranteed cleanup.
        print("\n[ACT ONNX] interrupted during setup.")
    finally:
        # Shield first: a second Ctrl+C here must not abort the release half-way.
        with suspend_sigint():
            safe_disconnect(robot, log=lambda m: print(f"[ACT ONNX] {m}"))
        print("[ACT ONNX] robot disconnected.")


if __name__ == "__main__":
    main()
