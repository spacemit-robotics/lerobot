#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import base64
import json
import logging
import math
from functools import cached_property

import cv2
import numpy as np

from lerobot.cameras.configs import Cv2Rotation
from lerobot.processor import RobotAction, RobotObservation
from lerobot.cameras.utils import get_cv2_rotation
from lerobot.utils.constants import ACTION, OBS_STATE
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError

from ..robot import Robot
from .config_mars import MarsClientConfig


class MarsClient(Robot):
    config_class = MarsClientConfig
    name = "mars_client"

    def __init__(self, config: MarsClientConfig):
        import zmq

        self._zmq = zmq
        super().__init__(config)
        self.config = config
        self.id = config.id
        self.robot_type = config.type
        self.remote_ip = config.remote_ip
        self.port_zmq_cmd = config.port_zmq_cmd
        self.port_zmq_observations = config.port_zmq_observations
        self.teleop_keys = config.teleop_keys
        self.polling_timeout_ms = config.polling_timeout_ms
        self.connect_timeout_s = config.connect_timeout_s
        self.zmq_context = None
        self.zmq_cmd_socket = None
        self.zmq_observation_socket = None
        self.last_frames = {}
        self.last_remote_state = {}
        self.speed_levels = [
            {"xy": 0.2, "theta": math.pi / 3},
            {"xy": 0.25, "theta": math.pi / 3},
            {"xy": 0.3, "theta": math.pi / 2},
        ]
        self.speed_index = 0
        self._is_connected = False
        self.logs = {}

    @cached_property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "arm_shoulder_pan.pos",
                "arm_shoulder_lift.pos",
                "arm_elbow_flex.pos",
                "arm_wrist_flex.pos",
                "arm_wrist_roll.pos",
                "arm_gripper.pos",
                "x.vel",
                "y.vel",
                "theta.vel",
            ),
            float,
        )

    @cached_property
    def _state_order(self) -> tuple[str, ...]:
        return tuple(self._state_ft.keys())

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
        camera_features: dict[str, tuple[int, int, int]] = {}
        for name, cfg in self.config.cameras.items():
            height = cfg.height or 480
            width = cfg.width or 640
            rotation = get_cv2_rotation(getattr(cfg, "rotation", Cv2Rotation.NO_ROTATION))
            if rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE]:
                height, width = width, height
            camera_features[name] = (height, width, 3)
        return camera_features

    @property
    def observation_features(self) -> dict:
        return {**self._state_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        zmq = self._zmq
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_cmd_socket.connect(f"tcp://{self.remote_ip}:{self.port_zmq_cmd}")
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_observation_socket.connect(f"tcp://{self.remote_ip}:{self.port_zmq_observations}")
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)

        poller = zmq.Poller()
        poller.register(self.zmq_observation_socket, zmq.POLLIN)
        socks = dict(poller.poll(self.connect_timeout_s * 1000))
        if self.zmq_observation_socket not in socks or socks[self.zmq_observation_socket] != zmq.POLLIN:
            raise DeviceNotConnectedError("Timeout waiting for Mars Host to connect expired.")

        self._is_connected = True

    def calibrate(self) -> None:
        return None

    def _poll_and_get_latest_message(self) -> str | None:
        zmq = self._zmq
        if self.zmq_observation_socket is None:
            return None

        poller = zmq.Poller()
        poller.register(self.zmq_observation_socket, zmq.POLLIN)
        try:
            socks = dict(poller.poll(self.polling_timeout_ms))
        except zmq.ZMQError as e:
            logging.error(f"ZMQ polling error: {e}")
            return None
        if self.zmq_observation_socket not in socks:
            return None

        last_msg = None
        while True:
            try:
                last_msg = self.zmq_observation_socket.recv_string(zmq.NOBLOCK)
            except zmq.Again:
                break
        return last_msg

    def _parse_observation_json(self, obs_string: str) -> RobotObservation | None:
        try:
            return json.loads(obs_string)
        except json.JSONDecodeError as e:
            logging.error(f"Error decoding JSON observation: {e}")
            return None

    def _decode_image_from_b64(self, image_b64: str) -> np.ndarray | None:
        if not image_b64:
            return None
        try:
            jpg_data = base64.b64decode(image_b64)
            np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except (TypeError, ValueError) as e:
            logging.error(f"Error decoding base64 image data: {e}")
            return None

    def _remote_state_from_obs(self, observation: RobotObservation) -> tuple[dict[str, np.ndarray], RobotObservation]:
        flat_state = {key: observation.get(key, 0.0) for key in self._state_order}
        state_vec = np.array([flat_state[key] for key in self._state_order], dtype=np.float32)
        obs_dict: RobotObservation = {**flat_state, OBS_STATE: state_vec}

        current_frames: dict[str, np.ndarray] = {}
        for cam_name, image_b64 in observation.items():
            if cam_name not in self._cameras_ft:
                continue
            frame = self._decode_image_from_b64(image_b64)
            if frame is not None:
                current_frames[cam_name] = frame

        return current_frames, obs_dict

    def _get_data(self) -> tuple[dict[str, np.ndarray], RobotObservation]:
        latest_message_str = self._poll_and_get_latest_message()
        if latest_message_str is None:
            return self.last_frames, self.last_remote_state

        observation = self._parse_observation_json(latest_message_str)
        if observation is None:
            return self.last_frames, self.last_remote_state

        try:
            new_frames, new_state = self._remote_state_from_obs(observation)
        except Exception as e:
            logging.error(f"Error processing observation data, serving last observation: {e}")
            return self.last_frames, self.last_remote_state

        self.last_frames = new_frames
        self.last_remote_state = new_state
        return new_frames, new_state

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        frames, obs_dict = self._get_data()
        for cam_name, frame in frames.items():
            obs_dict[cam_name] = frame if frame is not None else np.zeros((640, 480, 3), dtype=np.uint8)
        return obs_dict

    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray) -> dict[str, float]:
        if self.teleop_keys["speed_up"] in pressed_keys:
            self.speed_index = min(self.speed_index + 1, 2)
        if self.teleop_keys["speed_down"] in pressed_keys:
            self.speed_index = max(self.speed_index - 1, 0)
        speed_setting = self.speed_levels[self.speed_index]
        xy_speed = speed_setting["xy"]
        theta_speed = speed_setting["theta"]

        x_cmd = 0.0
        y_cmd = 0.0
        theta_cmd = 0.0
        if self.teleop_keys["forward"] in pressed_keys:
            x_cmd += xy_speed
        if self.teleop_keys["backward"] in pressed_keys:
            x_cmd -= xy_speed
        if self.teleop_keys["left"] in pressed_keys:
            y_cmd += xy_speed
        if self.teleop_keys["right"] in pressed_keys:
            y_cmd -= xy_speed
        if self.teleop_keys["rotate_left"] in pressed_keys:
            theta_cmd += theta_speed
        if self.teleop_keys["rotate_right"] in pressed_keys:
            theta_cmd -= theta_speed
        return {"x.vel": x_cmd, "y.vel": y_cmd, "theta.vel": theta_cmd}

    def configure(self) -> None:
        return None

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        if self.zmq_cmd_socket is None:
            raise DeviceNotConnectedError("Mars client command socket is not connected.")

        self.zmq_cmd_socket.send_string(json.dumps(action))
        actions = np.array([action.get(k, 0.0) for k in self._state_order], dtype=np.float32)
        action_sent = {key: actions[i] for i, key in enumerate(self._state_order)}
        action_sent[ACTION] = actions
        return action_sent

    @check_if_not_connected
    def disconnect(self) -> None:
        if self.zmq_observation_socket is not None:
            self.zmq_observation_socket.close()
            self.zmq_observation_socket = None
        if self.zmq_cmd_socket is not None:
            self.zmq_cmd_socket.close()
            self.zmq_cmd_socket = None
        if self.zmq_context is not None:
            self.zmq_context.term()
            self.zmq_context = None
        self._is_connected = False
