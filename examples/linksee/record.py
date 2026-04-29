#!/usr/bin/env python3
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import time
from pathlib import Path

import numpy as np
from common import build_linksee_action_frame, build_linksee_observation_frame

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.processor import make_default_processors
from lerobot.robots.linksee import LinkseeClient, LinkseeClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener, sanity_check_dataset_robot_compatibility
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

REMOTE_IP = "10.0.90.55"
ROBOT_ID = "my_linksee"
LEADER_PORT = "/dev/ttyACM0"
LEADER_ID = "my_linksee_leader"
KEYBOARD_ID = "my_keyboard"

NUM_EPISODES = 30
FPS = 30
EPISODE_TIME_SEC = 600
RESET_TIME_SEC = 20
TASK_DESCRIPTION = "pick and place the cube on the orange box"
HF_REPO_ID = "annyi/linksee-pick-place-move"
PUSH_TO_HUB = False
RESUME = True


def _announce(message: str) -> None:
    """Always show Linksee record status in terminal and via existing log/TTS path."""
    print(message, flush=True)
    log_say(message)


def _get_available_camera_features(robot: LinkseeClient) -> dict[str, tuple[int, int, int]]:
    """Probe one observation and keep only cameras that are actually present."""
    observation = robot.get_observation()
    available_camera_features = {
        name: feature
        for name, feature in robot.observation_features.items()
        if isinstance(feature, tuple) and name in observation
    }
    return available_camera_features


def _record_episode(
    robot: LinkseeClient,
    leader_arm: SO101Leader,
    keyboard: KeyboardTeleop,
    dataset: LeRobotDataset,
    events: dict,
    fps: int,
    control_time_s: int,
    single_task: str,
    teleop_action_processor,
    robot_action_processor,
    robot_observation_processor,
    display_data: bool = True,
    record_data: bool = True,
) -> tuple[bool, int]:
    """Record one Linksee episode using leader arm + keyboard teleop."""
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
        observation_frame = build_linksee_observation_frame(dataset.features, obs)

        arm_action = leader_arm.get_action()
        arm_action = {f"arm_{k}": v for k, v in arm_action.items() if not k.startswith("arm_")}

        keyboard_action = keyboard.get_action()
        pressed_keys = np.array(list(keyboard_action.keys()), dtype=object)
        base_action = robot._from_keyboard_to_base_action(pressed_keys)
        act = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

        act_processed_teleop = teleop_action_processor((act, obs))
        robot_action_to_send = robot_action_processor((act_processed_teleop, obs))
        sent_action = robot.send_action(robot_action_to_send)

        if record_data:
            action_frame = build_linksee_action_frame(dataset.features, act_processed_teleop)
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
    """Record Linksee datasets with the same high-level flow as the LeKiwi example."""
    robot_config = LinkseeClientConfig(remote_ip=REMOTE_IP, id=ROBOT_ID)
    leader_arm_config = SO101LeaderConfig(port=LEADER_PORT, id=LEADER_ID)
    keyboard_config = KeyboardTeleopConfig(id=KEYBOARD_ID)

    robot = LinkseeClient(robot_config)
    leader_arm = SO101Leader(leader_arm_config)
    keyboard = KeyboardTeleop(keyboard_config)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    robot.connect()
    available_camera_features = _get_available_camera_features(robot)

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_hw_features = {
        **robot._state_ft,
        **available_camera_features,
    }
    obs_features = hw_to_dataset_features(obs_hw_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    if RESUME:
        dataset = LeRobotDataset(
            repo_id=HF_REPO_ID,
        )
        sanity_check_dataset_robot_compatibility(dataset, robot, FPS, dataset_features)
        if len(dataset.meta.video_keys) > 0:
            dataset.start_image_writer(num_threads=4)
        _announce(
            f"Resume recording on existing dataset {HF_REPO_ID} from episode {dataset.num_episodes + 1}"
        )
    else:
        dataset_root = Path.home() / ".cache/huggingface/lerobot" / HF_REPO_ID
        if dataset_root.exists():
            raise FileExistsError(
                f"Dataset already exists at {dataset_root}. Set RESUME = True to continue recording into "
                f"{HF_REPO_ID}, or change HF_REPO_ID to a new dataset name."
            )

        dataset = LeRobotDataset.create(
            repo_id=HF_REPO_ID,
            fps=FPS,
            features=dataset_features,
            robot_type=robot.name,
            use_videos=True,
            image_writer_threads=4,
        )

    leader_arm.connect()
    keyboard.connect()

    listener, events = init_keyboard_listener()
    init_rerun(session_name="linksee_record")
    should_push = False

    try:
        if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
            raise ValueError("Linksee robot or teleop is not connected!")

        _announce("Starting Linksee record loop...")
        recorded_episodes = 0
        while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
            current_episode = recorded_episodes + 1
            _announce(f"Recording Linksee episode {current_episode}/{NUM_EPISODES}")

            episode_completed, episode_frames = _record_episode(
                robot=robot,
                leader_arm=leader_arm,
                keyboard=keyboard,
                dataset=dataset,
                events=events,
                fps=FPS,
                control_time_s=EPISODE_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=True,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )

            if episode_frames == 0:
                _announce("No frames recorded for current Linksee episode, skipping save")
                if events["rerecord_episode"]:
                    _announce(f"Re-record Linksee episode {current_episode}/{NUM_EPISODES}")
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
                _announce(f"Reset the Linksee environment before episode {next_label}/{NUM_EPISODES}")
                _, reset_frames = _record_episode(
                    robot=robot,
                    leader_arm=leader_arm,
                    keyboard=keyboard,
                    dataset=dataset,
                    events=events,
                    fps=FPS,
                    control_time_s=RESET_TIME_SEC,
                    single_task=TASK_DESCRIPTION,
                    display_data=True,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    record_data=False,
                )
                if reset_frames == 0:
                    _announce("Linksee reset window not written to dataset")

            if events["rerecord_episode"]:
                _announce(f"Re-record Linksee episode {current_episode}/{NUM_EPISODES}")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            _announce(f"Saved Linksee episode {current_episode}/{NUM_EPISODES}")
            recorded_episodes += 1

        should_push = recorded_episodes > 0 and not events["rerecord_episode"]
    finally:
        _announce("Stop Linksee recording")
        if robot.is_connected:
            robot.disconnect()
        if leader_arm.is_connected:
            leader_arm.disconnect()
        if keyboard.is_connected:
            keyboard.disconnect()
        if listener is not None:
            listener.stop()

        dataset.finalize()
        if PUSH_TO_HUB and should_push:
            dataset.push_to_hub()


if __name__ == "__main__":
    main()
