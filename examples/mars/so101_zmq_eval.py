#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import sys
import time
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar, cast

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import build_dataset_frame, hw_to_dataset_features
import zmq
from lerobot.cameras import CameraConfig  # noqa: F401
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.processor import PolicyProcessorPipeline, RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.robots.robot import Robot
from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import predict_action
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import get_safe_torch_device
import torch

print("mkldnn before:", torch.backends.mkldnn.enabled)
torch.backends.mkldnn.enabled = False
print("mkldnn after:", torch.backends.mkldnn.enabled)

DEFAULT_ZMQ_CONNECT = "tcp://127.0.0.1:5777"
DEFAULT_START_MESSAGE = "start grab"
DEFAULT_FINISH_MESSAGE = "finish grab"
DEFAULT_RELEASE_MESSAGE = "Release the claws"
DEFAULT_WAIT_INTERVAL_S = 0.1
DEFAULT_RELEASE_HOLD_TIME_S = 1.0
DEFAULT_RELEASE_DISCONNECT_DELAY_S = 5.0
DEFAULT_GRIPPER_OPEN_POS = 100.0
DEFAULT_GRAB_TIMEOUT_S = 40.0
DEFAULT_ELBOW_FLEX_CLEARANCE_DELTA = -30.0
DEFAULT_ELBOW_FLEX_CLEARANCE_HOLD_S = 0.5
DEFAULT_POLICY_PATH = Path("outputs/train/mars_act_pick_cube/checkpoints/100000/pretrained_model")
DEFAULT_CALIBRATION_PATH = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json"


def _make_default_policy() -> PreTrainedConfig:
    policy = PreTrainedConfig.from_pretrained(DEFAULT_POLICY_PATH)
    policy.pretrained_path = DEFAULT_POLICY_PATH
    return policy


def _announce(message: str) -> None:
    print(message, flush=True)


def _get_dataset_root(repo_id: str, root: str | Path | None) -> Path:
    if root is not None:
        return Path(root)
    from lerobot.utils.constants import HF_LEROBOT_HOME

    return HF_LEROBOT_HOME / repo_id


def _remove_existing_dataset_dir(repo_id: str, root: str | Path | None) -> None:
    dataset_root = _get_dataset_root(repo_id, root)
    if dataset_root.exists():
        _announce(f"检测到已存在数据目录，先删除: {dataset_root}")
        shutil.rmtree(dataset_root)


@dataclass
class PairZMQConfig:
    connect: str = DEFAULT_ZMQ_CONNECT
    start_message: str = DEFAULT_START_MESSAGE
    finish_message: str = DEFAULT_FINISH_MESSAGE
    release_message: str = DEFAULT_RELEASE_MESSAGE
    wait_interval_sec: float = DEFAULT_WAIT_INTERVAL_S
    grab_timeout_s: float = DEFAULT_GRAB_TIMEOUT_S


@dataclass
class ReleaseConfig:
    gripper_open_pos: float = DEFAULT_GRIPPER_OPEN_POS
    hold_time_s: float = DEFAULT_RELEASE_HOLD_TIME_S
    disconnect_delay_s: float = DEFAULT_RELEASE_DISCONNECT_DELAY_S
    calibration_path: Path = DEFAULT_CALIBRATION_PATH
    retries: int = 5
    retry_interval_s: float = 0.2


@dataclass
class ClearanceConfig:
    elbow_flex_delta: float = DEFAULT_ELBOW_FLEX_CLEARANCE_DELTA
    hold_time_s: float = DEFAULT_ELBOW_FLEX_CLEARANCE_HOLD_S


@dataclass
class SO101ZMQEvalConfig:
    __path_fields__: ClassVar[list[str]] = ["policy"]

    zmq: PairZMQConfig = field(default_factory=PairZMQConfig)
    release: ReleaseConfig = field(default_factory=ReleaseConfig)
    clearance: ClearanceConfig = field(default_factory=ClearanceConfig)
    robot: SO101FollowerConfig = field(
        default_factory=lambda: SO101FollowerConfig(
            port="/dev/ttyACM0",
            id="my_awesome_follower_arm",
            cameras={
                "front": OpenCVCameraConfig(
                    index_or_path=15,
                    width=640,
                    height=480,
                    fps=30,
                    fourcc="MJPG",
                ),
                "wrist": OpenCVCameraConfig(
                    index_or_path=13,
                    width=640,
                    height=480,
                    fps=30,
                    fourcc="MJPG",
                ),
            },
        )
    )
    dataset: DatasetRecordConfig = field(
        default_factory=lambda: DatasetRecordConfig(
            repo_id="annyi/eval_mars-pick-cube",
            single_task="pick the cube",
            episode_time_s=180,
            reset_time_s=180,
            num_episodes=1,
            vcodec="h264",
            push_to_hub=False,
        )
    )
    policy: PreTrainedConfig | None = field(default_factory=_make_default_policy)
    display_data: bool = False
    play_sounds: bool = False

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        return cls.__path_fields__


class RoundTripGrabClient:
    def __init__(self, cfg: SO101ZMQEvalConfig) -> None:
        self.cfg = cfg
        self._locked_action: RobotAction | None = None
        self._hold_robot: Robot | None = None
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(cfg.zmq.connect)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        _announce(f"ZMQ PAIR 已连接到 {cfg.zmq.connect}")

    def _detach_hold_robot(self) -> Robot | None:
        robot = self._hold_robot
        self._hold_robot = None
        return robot

    def send_message(self, message: str) -> None:
        self.socket.send_string(message)
        _announce(f"已发送 ZMQ 消息: {message}")

    def _create_policy_runtime(
        self,
    ) -> tuple[
        Robot,
        LeRobotDataset,
        PreTrainedPolicy,
        PolicyProcessorPipeline[dict[str, Any], dict[str, Any]],
        PolicyProcessorPipeline[Any, Any],
        RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
        RobotProcessorPipeline[RobotObservation, RobotObservation],
    ]:
        if self.cfg.policy is None:
            raise ValueError("需要提供 --policy.path，用于预加载并执行抓取策略。")

        robot = make_robot_from_config(self.cfg.robot)
        robot.connect()
        camera_count = len(getattr(robot, "cameras", {}))
        _remove_existing_dataset_dir(self.cfg.dataset.repo_id, self.cfg.dataset.root)

        dataset_features = {
            **hw_to_dataset_features(robot.action_features, ACTION, use_video=self.cfg.dataset.video),
            **hw_to_dataset_features(robot.observation_features, OBS_STR, use_video=self.cfg.dataset.video),
        }
        dataset = LeRobotDataset.create(
            repo_id=self.cfg.dataset.repo_id,
            fps=self.cfg.dataset.fps,
            root=self.cfg.dataset.root,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=self.cfg.dataset.video,
            image_writer_processes=self.cfg.dataset.num_image_writer_processes,
            image_writer_threads=self.cfg.dataset.num_image_writer_threads_per_camera * camera_count,
            batch_encoding_size=self.cfg.dataset.video_encoding_batch_size,
            vcodec=self.cfg.dataset.vcodec,
            streaming_encoding=self.cfg.dataset.streaming_encoding,
            encoder_queue_maxsize=self.cfg.dataset.encoder_queue_maxsize,
            encoder_threads=self.cfg.dataset.encoder_threads,
        )
        policy = make_policy(self.cfg.policy, ds_meta=dataset.meta)
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=self.cfg.policy,
            pretrained_path=(str(self.cfg.policy.pretrained_path) if self.cfg.policy.pretrained_path else None),
            dataset_stats=cast(Any, dataset.meta.stats),
            preprocessor_overrides={"device_processor": {"device": self.cfg.policy.device}},
        )
        _, robot_action_processor, robot_observation_processor = make_default_processors()
        return (
            robot,
            dataset,
            cast(PreTrainedPolicy, policy),
            preprocessor,
            cast(PolicyProcessorPipeline[Any, Any], postprocessor),
            robot_action_processor,
            robot_observation_processor,
        )

    def _lock_current_pose(self) -> None:
        if self._locked_action is None:
            raise RuntimeError("当前没有可锁定的机械臂姿态。")

        if self._hold_robot is None:
            robot = make_robot_from_config(self.cfg.robot)
            robot.connect(calibrate=False)
            self._hold_robot = robot

        self._hold_robot.send_action(self._locked_action)
        _announce(f"已锁定当前机械臂姿态: {self._locked_action}")

    def _lower_third_joint_for_navigation(self, robot: Robot, action: RobotAction) -> RobotAction:
        if "elbow_flex.pos" not in action:
            _announce("当前动作中没有 elbow_flex.pos，跳过第三关节避让姿态调整")
            return action

        adjusted_action = dict(action)
        current_pos = float(adjusted_action["elbow_flex.pos"])
        target_pos = current_pos + self.cfg.clearance.elbow_flex_delta
        adjusted_action["elbow_flex.pos"] = target_pos

        sent_action = robot.send_action(adjusted_action)
        time.sleep(self.cfg.clearance.hold_time_s)
        _announce(
            "抓取完成前已下压第三关节 elbow_flex 避让雷达: "
            f"{current_pos:.2f} -> {target_pos:.2f}"
        )
        return sent_action

    def wait_for_message(self, timeout_sec: float | None = None) -> str | None:
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec

        while True:
            if deadline is not None and time.monotonic() > deadline:
                _announce("等待消息超时")
                return None

            events = dict(self.poller.poll(timeout=int(self.cfg.zmq.wait_interval_sec * 1000)))
            if self.socket in events and events[self.socket] == zmq.POLLIN:
                message = self.socket.recv_string().strip()
                _announce(f"收到 ZMQ 消息: {message}")
                return message

    def _build_record_config(self) -> RecordConfig:
        if self.cfg.policy is None:
            raise ValueError("需要提供 --policy.path，用于预加载并执行抓取策略。")

        return RecordConfig(
            robot=self.cfg.robot,
            dataset=self.cfg.dataset,
            policy=self.cfg.policy,
            display_data=self.cfg.display_data,
            play_sounds=self.cfg.play_sounds,
        )

    def _run_grab(self) -> None:
        _announce(
            "开始执行 lerobot record 抓取流程，"
            f"episode_time_s={self.cfg.dataset.episode_time_s}，"
            f"num_episodes={self.cfg.dataset.num_episodes}"
        )
        robot: Robot | None = None
        dataset: LeRobotDataset | None = None
        (
            robot,
            dataset,
            policy,
            preprocessor,
            postprocessor,
            robot_action_processor,
            robot_observation_processor,
        ) = self._create_policy_runtime()

        try:
            policy.reset()
            preprocessor.reset()
            postprocessor.reset()

            start_time = time.monotonic()
            last_action: RobotAction | None = None

            while True:
                loop_start = time.perf_counter()
                elapsed = time.monotonic() - start_time
                if elapsed >= self.cfg.zmq.grab_timeout_s:
                    _announce(
                        f"抓取达到 {self.cfg.zmq.grab_timeout_s:.1f} 秒，停止推理并锁定当前机械臂状态"
                    )
                    break

                observation = robot.get_observation()
                obs_processed = robot_observation_processor(observation)
                observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)
                action_values = predict_action(
                    observation=observation_frame,
                    policy=policy,
                    device=get_safe_torch_device(policy.config.device or "cpu"),
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    use_amp=policy.config.use_amp,
                    task=self.cfg.dataset.single_task,
                    robot_type=robot.robot_type,
                )
                robot_action = make_robot_action(action_values, dataset.features)
                last_action = robot_action_processor((robot_action, observation))
                robot.send_action(last_action)

                if self.cfg.display_data:
                    _ = obs_processed

                precise_sleep(max(1 / self.cfg.dataset.fps - (time.perf_counter() - loop_start), 0.0))

            if last_action is None:
                observation = robot.get_observation()
                last_action = {
                    key: value
                    for key, value in observation.items()
                    if key.endswith(".pos") and isinstance(value, (int, float))
                }

            self._locked_action = last_action
            robot.send_action(last_action)
            self._locked_action = self._lower_third_joint_for_navigation(robot, last_action)
            self._hold_robot = robot
            robot = None
            self.send_message(self.cfg.zmq.finish_message)
        finally:
            if dataset is not None:
                dataset.finalize()
            if robot is not None and robot.is_connected:
                robot.disconnect()

    def _release_gripper(self) -> None:
        if self._locked_action is not None:
            self._lock_current_pose()
        _announce(f"开始释放夹爪，校准文件: {self.cfg.release.calibration_path}")
        robot = self._hold_robot
        created_here = False
        if robot is None:
            robot = make_robot_from_config(self.cfg.robot)
            created_here = True
        try:
            if not robot.is_connected:
                robot.connect(calibrate=False)
            observation = robot.get_observation()
            _announce(f"释放前夹爪位置: {observation.get('gripper.pos')}")

            sent_action = None
            for _ in range(self.cfg.release.retries):
                sent_action = robot.send_action({"gripper.pos": self.cfg.release.gripper_open_pos})
                time.sleep(self.cfg.release.retry_interval_s)

            _announce(f"释放夹爪动作已发送: {sent_action}")
            observation = robot.get_observation()
            _announce(f"释放后夹爪位置: {observation.get('gripper.pos')}")
            time.sleep(self.cfg.release.hold_time_s)
            _announce(f"释放完成，等待 {self.cfg.release.disconnect_delay_s:.1f} 秒后断开机械臂")
            time.sleep(self.cfg.release.disconnect_delay_s)
        finally:
            if robot.is_connected:
                robot.disconnect()
            self._hold_robot = None

    def handle_message(self, message: str) -> bool:
        if message == self.cfg.zmq.start_message:
            self._run_grab()
            return True

        if message == self.cfg.zmq.release_message:
            self._release_gripper()
            _announce("收到释放夹爪指令，任务结束")
            return False

        _announce(f"收到未知消息: {message}")
        return True

    def run(self) -> int:
        _announce("等待导航任务端指令...")
        while True:
            message = self.wait_for_message()
            if message is None:
                return 1
            if not self.handle_message(message):
                return 0

    def shutdown(self) -> None:
        robot = self._detach_hold_robot()
        if robot is not None and robot.is_connected:
            _announce("关闭客户端时检测到机械臂仍保持上电，跳过 disconnect 以避免塌陷")
        self.poller.unregister(self.socket)
        self.socket.close()
        self.context.term()


@parser.wrap()
def main(cfg: SO101ZMQEvalConfig) -> int:
    client = RoundTripGrabClient(cfg)

    try:
        return client.run()
    except KeyboardInterrupt:
        _announce("收到中断，客户端退出")
        return 130
    finally:
        client.shutdown()


def cli() -> None:
    sys.exit(main())  # type: ignore[call-arg]


if __name__ == "__main__":
    cli()