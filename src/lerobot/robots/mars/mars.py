#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import ctypes
import logging
import time
from functools import cached_property
from itertools import chain
from pathlib import Path

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_mars import MarsConfig

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[4]


class ChassisVelocity(ctypes.Structure):
    _fields_ = [("vx", ctypes.c_float), ("vy", ctypes.c_float), ("wz", ctypes.c_float)]


class ChassisPose(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("yaw", ctypes.c_float)]


class ChassisConfig(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("wheel_diameter", ctypes.c_float),
        ("wheel_base", ctypes.c_float),
        ("wheel_track", ctypes.c_float),
        ("left_wheel_gain", ctypes.c_float),
        ("max_speed", ctypes.c_float),
        ("max_angular", ctypes.c_float),
    ]


class ChassisUARTConfig(ctypes.Structure):
    _fields_ = [
        ("base", ChassisConfig),
        ("dev_path", ctypes.c_char_p),
        ("baud", ctypes.c_uint32),
    ]


class ChassisRPMSGConfig(ctypes.Structure):
    _fields_ = [
        ("base", ChassisConfig),
        ("ctrl_dev", ctypes.c_char_p),
        ("data_dev", ctypes.c_char_p),
        ("service_name", ctypes.c_char_p),
        ("local_addr", ctypes.c_uint32),
        ("remote_addr", ctypes.c_uint32),
    ]


class MarsBaseAdapter:
    """Bridge Mars base commands to the external SpacemiT chassis control library."""

    CHASSIS_TYPE_MAP = {
        "diff_2wd": 0,
        "diff_4wd": 1,
        "mecanum_4wd": 2,
        "omni_3wd": 3,
        "omni_4wd": 4,
    }

    DEFAULT_LIBRARY_CANDIDATES = (
        REPO_ROOT / "third_party/chassis/build/libchassis.so",
        REPO_ROOT / "third_party/chassis/build/libbase.so",
    )

    def __init__(self, robot: "Mars"):
        self.robot = robot

        self._lib: ctypes.CDLL | None = None
        self._dev = None
        self._config_buffer: ChassisUARTConfig | ChassisRPMSGConfig | None = None
        self._connected = False
        self._last_velocity = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def _resolve_library_path(self) -> Path:
        configured = self.robot.config.base_control_library_path
        if configured:
            candidate = Path(configured).expanduser()
            if candidate.is_file():
                return candidate
            raise FileNotFoundError(f"Configured Mars base library not found: {candidate}")

        for candidate in self.DEFAULT_LIBRARY_CANDIDATES:
            if candidate.is_file():
                return candidate

        raise FileNotFoundError(
            "Could not locate the SpacemiT chassis control shared library. "
            "Build third_party/chassis (for example with scripts/build_mars_chassis.sh) "
            "or set MarsConfig.base_control_library_path."
        )

    def _load_library(self) -> ctypes.CDLL:
        if self._lib is not None:
            return self._lib

        library_path = self._resolve_library_path()
        lib = ctypes.CDLL(str(library_path))
        lib.chassis_alloc.argtypes = [ctypes.c_char_p, ctypes.c_void_p]
        lib.chassis_alloc.restype = ctypes.c_void_p
        lib.chassis_set_velocity.argtypes = [ctypes.c_void_p, ctypes.POINTER(ChassisVelocity)]
        lib.chassis_set_velocity.restype = ctypes.c_int
        lib.chassis_get_odom.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ChassisVelocity),
            ctypes.POINTER(ChassisPose),
        ]
        lib.chassis_get_odom.restype = ctypes.c_int
        lib.chassis_brake.argtypes = [ctypes.c_void_p]
        lib.chassis_brake.restype = None
        lib.chassis_relax.argtypes = [ctypes.c_void_p]
        lib.chassis_relax.restype = None
        lib.chassis_free.argtypes = [ctypes.c_void_p]
        lib.chassis_free.restype = None
        self._lib = lib
        logger.info("Loaded Mars base control library from %s", library_path)
        return lib

    def _make_base_config(self) -> ChassisConfig:
        chassis_type = self.CHASSIS_TYPE_MAP.get(self.robot.config.base_type.lower())
        if chassis_type is None:
            raise ValueError(
                f"Unsupported Mars base_type '{self.robot.config.base_type}'. "
                f"Expected one of {sorted(self.CHASSIS_TYPE_MAP)}."
            )

        return ChassisConfig(
            type=chassis_type,
            wheel_diameter=self.robot.config.base_wheel_diameter,
            wheel_base=self.robot.config.base_wheel_base,
            wheel_track=self.robot.config.base_wheel_track,
            left_wheel_gain=self.robot.config.base_left_wheel_gain,
            max_speed=self.robot.config.base_max_speed,
            max_angular=self.robot.config.base_max_angular,
        )

    def connect(self) -> None:
        if self._connected:
            return

        lib = self._load_library()
        driver_name = self.robot.config.base_driver.encode("utf-8")
        base_cfg = self._make_base_config()

        if self.robot.config.base_driver == "drv_uart_esp32":
            config = ChassisUARTConfig(
                base=base_cfg,
                dev_path=self.robot.config.base_dev_path.encode("utf-8"),
                baud=self.robot.config.base_baud,
            )
        elif self.robot.config.base_driver == "drv_rpmsg_esos":
            config = ChassisRPMSGConfig(
                base=base_cfg,
                ctrl_dev=self.robot.config.base_rpmsg_ctrl_dev.encode("utf-8"),
                data_dev=self.robot.config.base_rpmsg_data_dev.encode("utf-8"),
                service_name=self.robot.config.base_rpmsg_service_name.encode("utf-8"),
                local_addr=self.robot.config.base_rpmsg_local_addr,
                remote_addr=self.robot.config.base_rpmsg_remote_addr,
            )
        else:
            raise ValueError(
                f"Unsupported Mars base_driver '{self.robot.config.base_driver}'. "
                "Expected 'drv_uart_esp32' or 'drv_rpmsg_esos'."
            )

        self._config_buffer = config
        self._dev = lib.chassis_alloc(driver_name, ctypes.byref(config))
        if not self._dev:
            raise ConnectionError(
                "Failed to allocate Mars base chassis device. "
                "Check the shared library, driver name, and base transport settings."
            )

        self._connected = True
        logger.info("Mars base adapter connected using %s", self.robot.config.base_driver)

    def send_velocity(self, x: float, y: float, theta: float) -> dict[str, float]:
        if not self._connected or self._lib is None or self._dev is None:
            raise ConnectionError("Mars base adapter is not connected")

        command = ChassisVelocity(vx=x, vy=y, wz=theta)
        status = self._lib.chassis_set_velocity(self._dev, ctypes.byref(command))
        if status != 0:
            raise RuntimeError(f"Failed to send Mars base velocity command, status={status}")

        self._last_velocity = {"x.vel": x, "y.vel": y, "theta.vel": theta}
        return dict(self._last_velocity)

    def get_state(self) -> dict[str, float]:
        if not self._connected or self._lib is None or self._dev is None:
            raise ConnectionError("Mars base adapter is not connected")

        vel = ChassisVelocity()
        pose = ChassisPose()
        status = self._lib.chassis_get_odom(self._dev, ctypes.byref(vel), ctypes.byref(pose))
        if status != 0:
            logger.warning("Failed to read Mars base odometry, status=%s. Reusing last velocity.", status)
            return dict(self._last_velocity)

        self._last_velocity = {"x.vel": float(vel.vx), "y.vel": float(vel.vy), "theta.vel": float(vel.wz)}
        return dict(self._last_velocity)

    def stop(self) -> None:
        if not self._connected or self._lib is None or self._dev is None:
            return
        self._lib.chassis_brake(self._dev)
        self._last_velocity = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

    def disconnect(self) -> None:
        if not self._connected or self._lib is None or self._dev is None:
            return

        try:
            self.stop()
            self._lib.chassis_free(self._dev)
        finally:
            self._dev = None
            self._connected = False


class Mars(Robot):
    """Mars mobile manipulator using SO101 arm control plus external base adapter."""

    config_class = MarsConfig
    name = "mars"

    def __init__(self, config: MarsConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "arm_shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "arm_shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "arm_elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "arm_wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "arm_wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.arm_motors = [motor for motor in self.bus.motors if motor.startswith("arm")]
        self.base_adapter = MarsBaseAdapter(self)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
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

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.bus.is_connected
            and self.base_adapter.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.base_adapter.connect()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        motors = list(self.arm_motors)

        self.bus.disable_torque(self.arm_motors)
        for name in self.arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

        input("Move robot to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings(self.arm_motors)

        full_turn_motor = [motor for motor in motors if any(keyword in motor for keyword in ["wrist_roll"])]
        unknown_range_motors = [motor for motor in motors if motor not in full_turn_motor]

        print(
            f"Move all arm joints except '{full_turn_motor}' sequentially through their entire ranges of motion.\n"
            "Recording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        for name in full_turn_motor:
            range_mins[name] = 0
            range_maxes[name] = 4095

        self.calibration = {}
        for name, motor in self.bus.motors.items():
            self.calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=int(homing_offsets[name]),
                range_min=int(range_mins[name]),
                range_max=int(range_maxes[name]),
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self):
        self.bus.disable_torque()
        self.bus.configure_motors()
        for name in self.arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self.bus.write("P_Coefficient", name, 16)
            self.bus.write("I_Coefficient", name, 0)
            self.bus.write("D_Coefficient", name, 32)

        self.bus.enable_torque()

    def setup_motors(self) -> None:
        for motor in chain(reversed(self.arm_motors)):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        arm_pos = self.bus.sync_read("Present_Position", self.arm_motors)
        base_vel = self.base_adapter.get_state()
        arm_state = {f"{k}.pos": v for k, v in arm_pos.items()}
        obs_dict = {**arm_state, **base_vel}
        logger.debug(f"{self} read state: {(time.perf_counter() - start) * 1e3:.1f}ms")

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.read_latest()

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        arm_goal_pos = {k: v for k, v in action.items() if k.endswith(".pos")}
        base_goal_vel = {k: v for k, v in action.items() if k.endswith(".vel")}

        if self.config.max_relative_target is not None and arm_goal_pos:
            present_pos = self.bus.sync_read("Present_Position", self.arm_motors)
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in arm_goal_pos.items()}
            arm_goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        if arm_goal_pos:
            arm_goal_pos_raw = {k.replace(".pos", ""): v for k, v in arm_goal_pos.items()}
            self.bus.sync_write("Goal_Position", arm_goal_pos_raw)
        self.base_adapter.send_velocity(
            float(base_goal_vel.get("x.vel", 0.0)),
            float(base_goal_vel.get("y.vel", 0.0)),
            float(base_goal_vel.get("theta.vel", 0.0)),
        )

        return {**arm_goal_pos, **base_goal_vel}

    def stop_base(self) -> None:
        self.base_adapter.stop()
        logger.info("Base motors stopped")

    @check_if_not_connected
    def disconnect(self):
        self.stop_base()
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        self.base_adapter.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
