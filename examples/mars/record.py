#!/usr/bin/env python3
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.processor import make_default_processors
from lerobot.robots.mars import MarsClient, MarsClientConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun

REMOTE_IP = "10.0.91.173"
ROBOT_ID = "my_mars"
LEADER_PORT = "/dev/ttyACM0"
LEADER_ID = "my_mars_leader"
KEYBOARD_ID = "my_keyboard"

NUM_EPISODES = 2
FPS = 30
EPISODE_TIME_SEC = 30
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "My Mars task description"
HF_REPO_ID = "<hf_username>/<mars_dataset_repo_id>"


def main() -> None:
    """Record Mars datasets with the same high-level flow as the LeKiwi example."""
    robot_config = MarsClientConfig(remote_ip=REMOTE_IP, id=ROBOT_ID)
    leader_arm_config = SO101LeaderConfig(port=LEADER_PORT, id=LEADER_ID)
    keyboard_config = KeyboardTeleopConfig(id=KEYBOARD_ID)

    robot = MarsClient(robot_config)
    leader_arm = SO101Leader(leader_arm_config)
    keyboard = KeyboardTeleop(keyboard_config)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    dataset = LeRobotDataset.create(
        repo_id=HF_REPO_ID,
        fps=FPS,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )

    robot.connect()
    leader_arm.connect()
    keyboard.connect()

    listener, events = init_keyboard_listener()
    init_rerun(session_name="mars_record")

    try:
        if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
            raise ValueError("Mars robot or teleop is not connected!")

        print("Starting Mars record loop...")
        recorded_episodes = 0
        while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
            log_say(f"Recording Mars episode {recorded_episodes}")

            record_loop(
                robot=robot,
                events=events,
                fps=FPS,
                dataset=dataset,
                teleop=[leader_arm, keyboard],
                control_time_s=EPISODE_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=True,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )

            if not events["stop_recording"] and (
                (recorded_episodes < NUM_EPISODES - 1) or events["rerecord_episode"]
            ):
                log_say("Reset the Mars environment")
                record_loop(
                    robot=robot,
                    events=events,
                    fps=FPS,
                    teleop=[leader_arm, keyboard],
                    control_time_s=RESET_TIME_SEC,
                    single_task=TASK_DESCRIPTION,
                    display_data=True,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                )

            if events["rerecord_episode"]:
                log_say("Re-record Mars episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            recorded_episodes += 1
    finally:
        log_say("Stop Mars recording")
        if robot.is_connected:
            robot.disconnect()
        if leader_arm.is_connected:
            leader_arm.disconnect()
        if keyboard.is_connected:
            keyboard.disconnect()
        if listener is not None:
            listener.stop()

        dataset.finalize()
        dataset.push_to_hub()


if __name__ == "__main__":
    main()