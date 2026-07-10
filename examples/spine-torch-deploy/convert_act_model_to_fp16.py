#!/usr/bin/env python3

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Convert an ACT pretrained directory from FP32 to FP16."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import torch
from spine_runtime import load_policy_for_fp16_conversion


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-path", type=Path, required=True, help="Source ACT pretrained directory.")
    parser.add_argument("--output-path", type=Path, required=True, help="Destination FP16 directory.")
    return parser.parse_args()


def convert_act_model_to_fp16(input_path: Path, output_path: Path) -> None:
    input_path = input_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()

    if not input_path.is_dir():
        raise FileNotFoundError(f"ACT pretrained directory does not exist: {input_path}")
    if output_path.exists():
        raise FileExistsError(f"Output path already exists: {output_path}")

    policy = load_policy_for_fp16_conversion(input_path)
    policy.to(dtype=torch.float16)

    shutil.copytree(input_path, output_path)
    policy.save_pretrained(output_path)

    first_parameter = next(policy.parameters())
    print(f"Saved FP16 ACT model to {output_path}")
    print(f"Model parameter dtype: {first_parameter.dtype}")


def main() -> None:
    args = parse_args()
    convert_act_model_to_fp16(args.input_path, args.output_path)


if __name__ == "__main__":
    main()
