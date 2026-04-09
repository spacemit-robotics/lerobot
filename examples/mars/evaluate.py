#!/usr/bin/env python3
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import time

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
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from common import build_mars_action_frame, build_mars_observation_frame

REMOTE_IP = "10.0.90.55"
ROBOT_ID = "my_mars"
TRAIN_DATASET_REPO_ID = "annyi/mars-pick-place"
NUM_EPISODES = 2
FPS = 30
EPISODE_TIME_SEC = 60
RESET_TIME_SEC = 20
TASK_DESCRIPTION = "pick and place the cube on the orange box"
HF_MODEL_ID = "/home/anny/workspaces/lerobot-0.5.0/outputs/train/mars_act_pick_place/checkpoints/080000/pretrained_model"
HF_DATASET_ID = "annyi/mars-pick-place-eval"
PUSH_TO_HUB = False


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
        robot_action_to_send = robot_action_processor((act_processed_policy, obs))
        sent_action = robot.send_action(robot_action_to_send)

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
    init_rerun(session_name="mars_evaluate")
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
                display_data=True,
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