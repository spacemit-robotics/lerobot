#!/usr/bin/env python3
# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import time

import numpy as np

from lerobot.robots.mars import MarsClient, MarsClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

FPS = 30


def _stop_base_action() -> dict[str, float]:
    return {
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }


def main():
    """Run minimal Mars host/client teleoperation."""
    # 1) Mars host 必须已在机器人侧启动，例如：
    # python -m lerobot.robots.mars.mars_host \
    #   --robot.id=my_mars \
    #   --robot.port=/dev/ttyACM0 \
    #   --robot.base_driver=drv_uart_esp32 \
    #   --robot.base_dev_path=/dev/ttyACM1

    # 2) 本脚本运行在操作者电脑侧，通过 ZMQ 连接 Mars host。
    # 注意不要混淆 3 个端口/接口：
    # - --robot.port: 机器人侧 follower arm 串口
    # - --robot.base_dev_path: 机器人侧 base 串口
    # - SO101LeaderConfig(port=...): 操作端 leader arm 串口
    robot_config = MarsClientConfig(remote_ip="10.0.91.173", id="my_mars")
    teleop_arm_config = SO101LeaderConfig(port="/dev/ttyACM0", id="my_mars_leader")
    keyboard_config = KeyboardTeleopConfig(id="my_keyboard")

    robot = MarsClient(robot_config)
    leader_arm = SO101Leader(teleop_arm_config)
    keyboard = KeyboardTeleop(keyboard_config)

    robot.connect()
    leader_arm.connect()
    keyboard.connect()

    init_rerun(session_name="mars_host_client_teleop")

    if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
        raise RuntimeError("Mars robot or teleop device is not connected")

    print("Starting Mars host/client teleop loop...")
    print("Press q to stop the teleop loop. The script will send a zero base command before exit.")
    try:
        while True:
            t0 = time.perf_counter()

            observation = robot.get_observation()

            arm_action = leader_arm.get_action()
            arm_action = {f"arm_{k}": v for k, v in arm_action.items()}

            keyboard_keys = keyboard.get_action()
            if "q" in keyboard_keys:
                robot.send_action(_stop_base_action())
                log_rerun_data(observation=observation, action=_stop_base_action())
                print("Quit requested from keyboard teleop. Sent zero base command and exiting.")
                break

            pressed_keys = np.array(list(keyboard_keys.keys()), dtype=object)
            base_action = robot._from_keyboard_to_base_action(pressed_keys)

            action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action
            robot.send_action(action)

            log_rerun_data(observation=observation, action=action)
            precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
    finally:
        if keyboard.is_connected:
            keyboard.disconnect()
        if leader_arm.is_connected:
            leader_arm.disconnect()
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
