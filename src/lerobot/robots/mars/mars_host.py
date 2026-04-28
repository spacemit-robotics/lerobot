#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import base64
import json
import logging
import time
from dataclasses import dataclass, field

import cv2
import draccus
import zmq

from .config_mars import MarsConfig, MarsHostConfig
from .mars import Mars


@dataclass
class MarsServerConfig:
    robot: MarsConfig = field(default_factory=MarsConfig)
    host: MarsHostConfig = field(default_factory=MarsHostConfig)


class MarsHost:
    def __init__(self, config: MarsHostConfig):
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")

        self.connection_time_s = config.connection_time_s
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz

    def disconnect(self) -> None:
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()


@draccus.wrap()
def main(cfg: MarsServerConfig):
    """Run Mars host process near the robot hardware."""
    logging.info("Configuring Mars")
    robot = Mars(cfg.robot)

    logging.info("Connecting Mars")
    robot.connect()

    logging.info("Starting Mars host")
    host = MarsHost(cfg.host)

    last_cmd_time = time.time()
    watchdog_active = False
    try:
        start = time.perf_counter()
        duration = 0
        while duration < host.connection_time_s:
            loop_start_time = time.time()
            try:
                msg = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                data = dict(json.loads(msg))
                robot.send_action(data)
                last_cmd_time = time.time()
                watchdog_active = False
            except zmq.Again:
                pass
            except Exception as e:
                logging.error("Message fetching failed: %s", e)

            now = time.time()
            if (now - last_cmd_time > host.watchdog_timeout_ms / 1000) and not watchdog_active:
                logging.warning(
                    f"Command not received for more than {host.watchdog_timeout_ms} milliseconds. Stopping the base."
                )
                watchdog_active = True
                robot.stop_base()

            last_observation = robot.get_observation()
            for cam_key in robot.cameras:
                ret, buffer = cv2.imencode(
                    ".jpg", last_observation[cam_key], [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )
                last_observation[cam_key] = base64.b64encode(buffer).decode("utf-8") if ret else ""

            try:
                host.zmq_observation_socket.send_string(json.dumps(last_observation), flags=zmq.NOBLOCK)
            except zmq.Again:
                logging.info("Dropping observation, no client connected")

            elapsed = time.time() - loop_start_time
            time.sleep(max(1 / host.max_loop_freq_hz - elapsed, 0))
            duration = time.perf_counter() - start
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
    finally:
        print("Shutting down Mars Host.")
        robot.disconnect()
        host.disconnect()

    logging.info("Finished Mars cleanly")


if __name__ == "__main__":
    main()
