#!/usr/bin/env python3
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import time
from typing import Any

import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.processor import make_default_processors
from lerobot.robots.mars import MarsClient, MarsClientConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener, predict_action
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import get_safe_torch_device
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import log_rerun_data
from common import build_mars_action_frame, build_mars_observation_frame

REMOTE_IP = "127.0.0.1"
ROBOT_ID = "my_mars"
TRAIN_DATASET_REPO_ID = "annyi/mars-pick-place-move-v2"
NUM_EPISODES = 1
FPS = 30
EPISODE_TIME_SEC = 180
RESET_TIME_SEC = 20
TASK_DESCRIPTION = "pick and place the cube on the orange box"
HF_MODEL_ID = "/root/lerobot/outputs/train/mars_act_pick_place_move_v2/checkpoints/100000/pretrained_model"
HF_DATASET_ID = "annyi/mars-pick-place-move-v2-eval"
PUSH_TO_HUB = False
PRINT_INFERENCE_VALUES = True
BASE_VELOCITY_SCALE = 1
ENABLE_VISUALIZATION = False


def _format_value(value) -> float | list[float] | str:
    """Format scalar/tensor-like values for concise inference logging."""
    if isinstance(value, np.ndarray):
        if value.ndim == 0:
            return round(float(value), 6)
        return np.round(value.astype(np.float64), 6).tolist()
    if isinstance(value, np.generic):
        return round(float(value), 6)
    if isinstance(value, float):
        return round(value, 6)
    return value


def _print_inference_values(action_values: Any, processed_action: dict, sent_action: dict) -> None:
    """Print policy output and downstream Mars actions for debugging."""
    if hasattr(action_values, "items"):
        raw_summary = {key: _format_value(value) for key, value in action_values.items()}
    else:
        raw_summary = _format_value(action_values)
    processed_summary = {key: _format_value(value) for key, value in processed_action.items()}
    sent_summary = {
        key: _format_value(value)
        for key, value in sent_action.items()
        if key != ACTION
    }
    print(f"[Mars Eval] policy raw action: {raw_summary}", flush=True)
    print(f"[Mars Eval] processed action: {processed_summary}", flush=True)
    print(f"[Mars Eval] sent action: {sent_summary}", flush=True)


def _scale_base_velocity(action: dict[str, Any], scale: float) -> dict[str, Any]:
    """Scale Mars base velocity outputs for quick runtime experiments."""
    scaled_action = dict(action)
    for key in ("x.vel", "y.vel", "theta.vel"):
        value = scaled_action.get(key)
        if isinstance(value, np.ndarray):
            scaled_action[key] = value * scale
        elif value is not None:
            scaled_action[key] = float(value) * scale
    return scaled_action


def _announce(message: str) -> None:
    """Always show Mars evaluate status in terminal and via existing log/TTS path."""
    print(message, flush=True)
    log_say(message)


def _get_available_camera_features(robot: MarsClient) -> dict[str, tuple[int, int, int]]:
    """Probe one observation and keep only cameras that are actually present."""
    observation = robot.get_observation()
    available_camera_features = {
        name: feature
        for name, feature in robot.observation_features.items()
        if isinstance(feature, tuple) and name in observation
    }
    return available_camera_features

def _run_policy_episode(
    robot: MarsClient,
    policy: ACTPolicy,
    preprocessor,
    postprocessor,
    dataset: LeRobotDataset,
    events: dict,
    fps: int,
    control_time_s: int,
    single_task: str,
    robot_action_processor,
    robot_observation_processor,
    display_data: bool = True,
    record_data: bool = True,
) -> tuple[bool, int]:
    """Run one Mars policy episode and append frames to a dataset."""
    policy.reset()
    preprocessor.reset()
    postprocessor.reset()

    timestamp = 0.0
    exited_early = False
    frames_recorded = 0
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            exited_early = True
            break
        if events["stop_recording"]:
            break

        obs = robot.get_observation()
        obs_processed = robot_observation_processor(obs)
        observation_frame = build_mars_observation_frame(dataset.features, obs)

        action_values = predict_action(
            observation=observation_frame,
            policy=policy,
            device=get_safe_torch_device(policy.config.device or "cpu"),
            preprocessor=preprocessor,
            postprocessor=postprocessor,
            use_amp=policy.config.use_amp,
            task=single_task,
            robot_type=robot.robot_type,
        )
        act_processed_policy = make_robot_action(action_values, dataset.features)
        act_processed_policy = _scale_base_velocity(act_processed_policy, BASE_VELOCITY_SCALE)
        robot_action_to_send = robot_action_processor((act_processed_policy, obs))
        sent_action = robot.send_action(robot_action_to_send)

        if PRINT_INFERENCE_VALUES:
            _print_inference_values(action_values, act_processed_policy, sent_action)

        if record_data:
            action_frame = build_mars_action_frame(dataset.features, act_processed_policy)
            frame = {**observation_frame, **action_frame, "task": single_task}
            dataset.add_frame(frame)
            frames_recorded += 1

        if display_data:
            log_rerun_data(observation=obs_processed, action=sent_action, compress_images=False)

        loop_dt = time.perf_counter() - start_loop_t
        precise_sleep(max(1 / fps - loop_dt, 0.0))
        timestamp += time.perf_counter() - start_loop_t

    return (not events["stop_recording"] or exited_early), frames_recorded


def main() -> None:
    """Run Mars policy inference and record evaluation episodes."""
    robot_config = MarsClientConfig(remote_ip=REMOTE_IP, id=ROBOT_ID)
    robot = MarsClient(robot_config)

    policy = ACTPolicy.from_pretrained(HF_MODEL_ID)

    train_dataset = LeRobotDataset(TRAIN_DATASET_REPO_ID)

    robot.connect()
    available_camera_features = _get_available_camera_features(robot)

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_hw_features = {
        **robot._state_ft,
        **available_camera_features,
    }
    obs_features = hw_to_dataset_features(obs_hw_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    dataset = LeRobotDataset.create(
        repo_id=HF_DATASET_ID,
        fps=FPS,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy,
        pretrained_path=HF_MODEL_ID,
        dataset_stats=train_dataset.meta.stats,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
    )

    _, robot_action_processor, robot_observation_processor = make_default_processors()

    listener, events = init_keyboard_listener()
    should_push = False

    try:
        if not robot.is_connected:
            raise ValueError("Mars robot is not connected!")

        _announce("Starting Mars evaluate loop...")
        recorded_episodes = 0
        while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
            current_episode = recorded_episodes + 1
            _announce(f"Running Mars inference, recording eval episode {current_episode}/{NUM_EPISODES}")

            episode_completed, episode_frames = _run_policy_episode(
                robot=robot,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                dataset=dataset,
                events=events,
                fps=FPS,
                control_time_s=EPISODE_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=ENABLE_VISUALIZATION,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )
            if episode_frames == 0:
                _announce("No frames recorded for current Mars eval episode, skipping save")
                if events["rerecord_episode"]:
                    _announce(f"Re-record Mars eval episode {current_episode}/{NUM_EPISODES}")
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue
                if events["stop_recording"]:
                    break
                continue

            if not episode_completed or events["stop_recording"]:
                break

            if not events["stop_recording"] and (
                (recorded_episodes < NUM_EPISODES - 1) or events["rerecord_episode"]
            ):
                next_label = current_episode if events["rerecord_episode"] else current_episode + 1
                _announce(f"Reset the Mars environment before episode {next_label}/{NUM_EPISODES}")
                _, reset_frames = _run_policy_episode(
                    robot=robot,
                    policy=policy,
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    dataset=dataset,
                    events=events,
                    fps=FPS,
                    control_time_s=RESET_TIME_SEC,
                    single_task=TASK_DESCRIPTION,
                    display_data=True,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    record_data=False,
                )
                if reset_frames == 0:
                    _announce("Mars eval reset window not written to dataset")

            if events["rerecord_episode"]:
                _announce(f"Re-record Mars eval episode {current_episode}/{NUM_EPISODES}")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            _announce(f"Saved Mars eval episode {current_episode}/{NUM_EPISODES}")
            recorded_episodes += 1

        should_push = recorded_episodes > 0 and not events["rerecord_episode"]
    finally:
        _announce("Stop Mars evaluation")
        if robot.is_connected:
            robot.disconnect()
        if listener is not None:
            listener.stop()

        dataset.finalize()
        if PUSH_TO_HUB and should_push:
            dataset.push_to_hub()


if __name__ == "__main__":
    main()
