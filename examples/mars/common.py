#!/usr/bin/env python3
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from typing import Any

import numpy as np

from lerobot.datasets.utils import build_dataset_frame
from lerobot.utils.constants import ACTION, OBS_STR


def normalize_observation_for_dataset(observation: dict[str, Any]) -> dict[str, Any]:
    """Map Mars observation keys to dataset feature keys."""
    normalized = dict(observation)
    camera_names = {
        key.removeprefix(f"{OBS_STR}.images.")
        for key, feature in normalized.items()
        if key.startswith(f"{OBS_STR}.images.") and isinstance(feature, np.ndarray)
    }
    camera_names.update(
        key
        for key, value in normalized.items()
        if isinstance(value, np.ndarray) and key not in {OBS_STR, ACTION}
    )

    for camera_name in camera_names:
        if camera_name in normalized:
            normalized[f"{OBS_STR}.images.{camera_name}"] = normalized[camera_name]
        prefixed_key = f"{OBS_STR}.images.{camera_name}"
        if prefixed_key in normalized and camera_name not in normalized:
            normalized[camera_name] = normalized[prefixed_key]
    return normalized


def normalize_action_for_dataset(action: dict[str, Any]) -> dict[str, Any]:
    """Map Mars action keys to dataset feature keys."""
    normalized = dict(action)
    for key in list(action.keys()):
        value = normalized.pop(key)
        candidate_keys = {key}
        if key.startswith("arm_"):
            candidate_keys.add(key.removeprefix("arm_"))
        else:
            candidate_keys.add(f"arm_{key}")

        for candidate_key in candidate_keys:
            normalized[candidate_key] = value
            normalized[f"{ACTION}.{candidate_key}"] = value
    return normalized


def build_mars_observation_frame(
    dataset_features: dict[str, dict], observation: dict[str, Any]
) -> dict[str, Any]:
    """Build a dataset-compatible observation frame for Mars."""
    normalized_observation = normalize_observation_for_dataset(observation)
    available_features = {}
    for key, feature in dataset_features.items():
        if not key.startswith(OBS_STR):
            available_features[key] = feature
            continue

        if feature["dtype"] not in ["image", "video"]:
            available_features[key] = feature
            continue

        camera_name = key.removeprefix(f"{OBS_STR}.images.")
        if camera_name in normalized_observation:
            available_features[key] = feature

    frame = build_dataset_frame(available_features, normalized_observation, prefix=OBS_STR)

    return frame


def build_mars_action_frame(dataset_features: dict[str, dict], action: dict[str, Any]) -> dict[str, Any]:
    """Build a dataset-compatible action frame for Mars."""
    normalized_action = normalize_action_for_dataset(action)
    return build_dataset_frame(dataset_features, normalized_action, prefix=ACTION)
