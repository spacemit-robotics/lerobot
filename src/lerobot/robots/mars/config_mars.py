#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig, Cv2Rotation
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

from ..config import RobotConfig


def mars_cameras_config() -> dict[str, CameraConfig]:
    return {
        "front": OpenCVCameraConfig(
            index_or_path="/dev/video0", fps=30, width=640, height=480, rotation=Cv2Rotation.ROTATE_180
        ),
        "wrist": OpenCVCameraConfig(
            index_or_path="/dev/video2", fps=30, width=480, height=640, rotation=Cv2Rotation.ROTATE_90
        ),
    }


@RobotConfig.register_subclass("mars")
@dataclass
class MarsConfig(RobotConfig):
    port: str = "/dev/ttyACM0"
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=mars_cameras_config)
    use_degrees: bool = False
    # Mars base defaults to UART transport.
    # Keep RPMSG fields below only for optional compatibility with older deployments.
    base_driver: str = "drv_uart_esp32"
    base_control_library_path: str | None = None
    base_type: str = "diff_2wd"
    base_dev_path: str = "/dev/ttyACM1"
    base_baud: int = 115200
    base_wheel_diameter: float = 0.067
    base_wheel_base: float = 0.183
    base_wheel_track: float = 0.0
    base_left_wheel_gain: float = 1.0
    base_max_speed: float = 1.0
    base_max_angular: float = 3.14
    base_rpmsg_ctrl_dev: str = "/dev/rpmsg_ctrl0"
    base_rpmsg_data_dev: str = "/dev/rpmsg0"
    base_rpmsg_service_name: str = "rpmsg:motor_ctrl"
    base_rpmsg_local_addr: int = 1003
    base_rpmsg_remote_addr: int = 1002


@RobotConfig.register_subclass("mars_host")
@dataclass
class MarsHostConfig:
    port_zmq_cmd: int = 5565
    port_zmq_observations: int = 5566
    connection_time_s: int = 30
    watchdog_timeout_ms: int = 500
    max_loop_freq_hz: int = 30


@RobotConfig.register_subclass("mars_client")
@dataclass
class MarsClientConfig(RobotConfig):
    remote_ip: str
    host_type: str = "mars_host"
    port_zmq_cmd: int = 5565
    port_zmq_observations: int = 5566
    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            "forward": "w",
            "backward": "s",
            "left": "a",
            "right": "d",
            "rotate_left": "z",
            "rotate_right": "x",
            "speed_up": "r",
            "speed_down": "f",
            "quit": "q",
        }
    )
    cameras: dict[str, CameraConfig] = field(default_factory=mars_cameras_config)
    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
