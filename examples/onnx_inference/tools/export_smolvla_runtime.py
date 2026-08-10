#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Export SmolVLA runtime metadata for the C++ SO-101 runner.

The 4-model SmolVLA ONNX graph only contains neural network computation. The
C++ runner still needs camera keys, language tokens, normalization statistics,
and motor calibration. This tool exports those values from the LeRobot
pretrained_model directory and the SO-101 calibration JSON into a simple text
file consumed by cpp/smolvla_robot_pipeline.cpp.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

MOTOR_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
NORM_MODE = dict.fromkeys(MOTOR_ORDER, "DEGREES")
NORM_MODE["gripper"] = "RANGE_0_100"
DEFAULT_TASK = "Place the green cube into the box"
DEFAULT_TOKENIZER = "HuggingFaceTB/SmolVLM2-500M-Video-Instruct"


def _load_safetensors_np(path: Path) -> dict[str, np.ndarray]:
    from safetensors import safe_open

    out: dict[str, np.ndarray] = {}
    with safe_open(str(path), framework="np") as sf:
        for key in sf.keys():  # noqa: SIM118 - safe_open is not directly iterable.
            out[key] = sf.get_tensor(key)
    return out


def _fmt(arr: np.ndarray) -> str:
    return " ".join(f"{float(v):.8g}" for v in np.asarray(arr).flatten())


def _read_config(checkpoint: Path) -> dict:
    return json.loads((checkpoint / "config.json").read_text(encoding="utf-8"))


def _image_keys(config: dict) -> list[str]:
    return [key for key in config["input_features"] if key.startswith("observation.images.")]


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


def _tokenize(task: str, tokenizer_name: str, max_length: int) -> tuple[np.ndarray, np.ndarray]:
    from tokenizers import Tokenizer

    prompt = task if task.endswith("\n") else f"{task}\n"
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
    encoded = tokenizer.encode(prompt, add_special_tokens=True)
    return np.asarray(encoded.ids, dtype=np.int64), np.asarray(encoded.attention_mask, dtype=np.int64)


def _calibration_lines(calibration: Path | None) -> list[str]:
    if calibration is None:
        return ["# (no calibration provided; C++ direct motor control needs it)"]

    data = json.loads(calibration.read_text(encoding="utf-8"))
    lines = []
    for motor in MOTOR_ORDER:
        if motor not in data:
            raise SystemExit(f"calibration missing motor '{motor}' in {calibration}")
        item = data[motor]
        lines.append(
            f"calib {motor} id {item['id']} drive_mode {item['drive_mode']} "
            f"range_min {item['range_min']} range_max {item['range_max']} "
            f"norm_mode {NORM_MODE[motor]}"
        )
    return lines


def build_lines(
    checkpoint: Path,
    task: str,
    calibration: Path | None,
    tokenizer: str,
    num_cameras: int | None,
) -> list[str]:
    config = _read_config(checkpoint)
    image_keys = _image_keys(config)
    inferred_cameras = len(image_keys) + int(config.get("empty_cameras", 0))
    camera_count = num_cameras if num_cameras is not None else inferred_cameras
    resize_hw = config.get("resize_imgs_with_padding", [512, 512])
    tokenizer_max_length = int(config["tokenizer_max_length"])
    stats = _load_safetensors_np(checkpoint / "policy_preprocessor_step_5_normalizer_processor.safetensors")
    lang_tokens, lang_masks = _tokenize(task, tokenizer, tokenizer_max_length)

    lines = ["# SmolVLA runtime metadata for SO-101 C++ runner (generated)"]
    lines.append(f"task {task}")
    lines.append("image_keys " + " ".join(image_keys))
    lines.append(f"resize_hw {int(resize_hw[0])} {int(resize_hw[1])}")
    lines.append(f"tokenizer_max_length {tokenizer_max_length}")
    lines.append(f"state_dim {int(config['input_features']['observation.state']['shape'][0])}")
    lines.append(f"action_dim {int(config['output_features']['action']['shape'][0])}")
    lines.append(f"max_state_dim {int(config['max_state_dim'])}")
    lines.append(f"max_action_dim {int(config['max_action_dim'])}")
    lines.append(f"chunk_size {int(config['chunk_size'])}")
    lines.append(f"n_action_steps {int(config['n_action_steps'])}")
    lines.append(f"empty_cameras {max(0, camera_count - len(image_keys))}")
    lines.append("motor_order " + " ".join(MOTOR_ORDER))
    lines.append("state_mean " + _fmt(stats["observation.state.mean"]))
    lines.append("state_std " + _fmt(stats["observation.state.std"]))
    lines.append("action_mean " + _fmt(stats["action.mean"]))
    lines.append("action_std " + _fmt(stats["action.std"]))
    lines.append("lang_tokens " + " ".join(str(int(value)) for value in lang_tokens))
    lines.append("lang_masks " + " ".join(str(int(value)) for value in lang_masks))
    lines.extend(_calibration_lines(calibration))
    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--checkpoint", type=Path, required=True, help="SmolVLA pretrained_model directory")
    parser.add_argument("--output", type=Path, required=True, help="Output runtime metadata txt")
    parser.add_argument("--task", default=DEFAULT_TASK, help="Natural-language task prompt")
    parser.add_argument(
        "--tokenizer", default=DEFAULT_TOKENIZER, help="HF tokenizer name or local tokenizer dir"
    )
    parser.add_argument("--num-cameras", type=int, default=None, help="Override ONNX camera count")
    default_calibration = (
        Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json"
    )
    parser.add_argument(
        "--calibration", type=Path, default=default_calibration, help="SO-101 calibration JSON"
    )
    parser.add_argument(
        "--no-calibration",
        action="store_true",
        help="Skip calibration. C++ direct motor control will refuse to send actions.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    calibration = None if args.no_calibration else args.calibration
    if calibration is not None and not calibration.exists():
        print(
            f"[export_smolvla_runtime] WARNING: calibration not found: {calibration}\n"
            "  writing stats only; pass --no-calibration to silence, or fix --calibration."
        )
        calibration = None

    lines = build_lines(args.checkpoint, args.task, calibration, args.tokenizer, args.num_cameras)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(
        f"[export_smolvla_runtime] wrote {args.output} "
        f"({len(lines)} lines, calibration={'yes' if calibration else 'NO'})"
    )


if __name__ == "__main__":
    main()
