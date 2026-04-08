#!/usr/bin/env python3
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.processor import make_default_processors
from lerobot.robots.mars import MarsClient, MarsClientConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun

REMOTE_IP = "10.0.91.173"
ROBOT_ID = "my_mars"
TRAIN_DATASET_REPO_ID = "<hf_username>/<mars_dataset_repo_id>"

NUM_EPISODES = 2
FPS = 30
EPISODE_TIME_SEC = 60
TASK_DESCRIPTION = "My Mars eval task description"
HF_MODEL_ID = "<hf_username>/<mars_model_repo_id>"
HF_DATASET_ID = "<hf_username>/<mars_eval_dataset_repo_id>"


def main() -> None:
    """Run Mars policy inference and record evaluation episodes like the LeKiwi example."""
    robot_config = MarsClientConfig(remote_ip=REMOTE_IP, id=ROBOT_ID)
    robot = MarsClient(robot_config)

    policy = ACTPolicy.from_pretrained(HF_MODEL_ID)

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    dataset = LeRobotDataset.create(
        repo_id=HF_DATASET_ID,
        fps=FPS,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )

    # Inference must reuse the normalization statistics from the training dataset
    # rather than the fresh evaluation dataset being recorded in this run.
    train_dataset = LeRobotDataset(TRAIN_DATASET_REPO_ID)

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy,
        pretrained_path=HF_MODEL_ID,
        dataset_stats=train_dataset.meta.stats,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
    )

    robot.connect()

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    listener, events = init_keyboard_listener()
    init_rerun(session_name="mars_evaluate")

    try:
        if not robot.is_connected:
            raise ValueError("Mars robot is not connected!")

        print("Starting Mars evaluate loop...")
        recorded_episodes = 0
        while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
            log_say(f"Running Mars inference, recording eval episode {recorded_episodes} of {NUM_EPISODES}")

            record_loop(
                robot=robot,
                events=events,
                fps=FPS,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                dataset=dataset,
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
                    control_time_s=EPISODE_TIME_SEC,
                    single_task=TASK_DESCRIPTION,
                    display_data=True,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                )

            if events["rerecord_episode"]:
                log_say("Re-record Mars eval episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            recorded_episodes += 1
    finally:
        log_say("Stop Mars evaluation")
        if robot.is_connected:
            robot.disconnect()
        if listener is not None:
            listener.stop()

        dataset.finalize()
        dataset.push_to_hub()


if __name__ == "__main__":
    main()