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

"""Run lerobot-record with process-local SpineTorch ACT FP16 extensions."""

from __future__ import annotations

import sys

from spine_runtime import install_record_runtime


def _remove_legacy_dtype_argument(arguments: list[str]) -> list[str]:
    filtered: list[str] = []
    skip_next = False
    for argument in arguments:
        if skip_next:
            skip_next = False
            continue
        if argument == "--policy.dtype":
            skip_next = True
            continue
        if argument.startswith("--policy.dtype="):
            continue
        filtered.append(argument)
    return filtered


def main() -> None:
    sys.argv[1:] = _remove_legacy_dtype_argument(sys.argv[1:])

    # The FP16 runtime enables the plugin before lerobot-record initializes its logger.
    # Configure logging here so the plugin activation message is visible at startup.
    from lerobot.utils.utils import init_logging

    init_logging()
    install_record_runtime()

    from lerobot.scripts.lerobot_record import main as lerobot_record_main

    lerobot_record_main()


if __name__ == "__main__":
    main()
