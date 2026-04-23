#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import sys
import time

import zmq


DEFAULT_BIND = "tcp://*:5777"
DEFAULT_START_MESSAGE = "start grab"
DEFAULT_FINISH_MESSAGE = "finish grab"
DEFAULT_RELEASE_MESSAGE = "Release the claws"
DEFAULT_NAVIGATION_WAIT_S = 5.0
DEFAULT_RELEASE_DELAY_S = 2.0


def _announce(message: str) -> None:
    print(message, flush=True)


class NavigationPairServer:
    def __init__(self, bind_address: str) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(bind_address)
        _announce(f"导航端 ZMQ PAIR 已监听: {bind_address}")

    def send_message(self, message: str) -> None:
        self.socket.send_string(message)
        _announce(f"导航端发送: {message}")

    def wait_for_message(self, expected_message: str | None = None) -> str:
        message = self.socket.recv_string().strip()
        _announce(f"导航端收到: {message}")
        if expected_message is not None and message != expected_message:
            raise ValueError(f"期望消息 {expected_message!r}，实际收到 {message!r}")
        return message

    def close(self) -> None:
        self.socket.close()
        self.context.term()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mars 导航端 PAIR 测试脚本")
    parser.add_argument("--bind", default=DEFAULT_BIND, help="PAIR 监听地址")
    parser.add_argument("--start-message", default=DEFAULT_START_MESSAGE, help="开始抓取消息")
    parser.add_argument("--finish-message", default=DEFAULT_FINISH_MESSAGE, help="抓取完成消息")
    parser.add_argument("--release-message", default=DEFAULT_RELEASE_MESSAGE, help="释放夹爪消息")
    parser.add_argument(
        "--navigation-wait-s",
        type=float,
        default=DEFAULT_NAVIGATION_WAIT_S,
        help="发送开始抓取前的导航等待时间",
    )
    parser.add_argument(
        "--release-delay-s",
        type=float,
        default=DEFAULT_RELEASE_DELAY_S,
        help="收到抓取完成后，发送释放夹爪前的等待时间",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    server = NavigationPairServer(args.bind)

    try:
        _announce(f"模拟导航中，等待 {args.navigation_wait_s:.1f} 秒后发送抓取指令")
        time.sleep(args.navigation_wait_s)
        server.send_message(args.start_message)
        server.wait_for_message(args.finish_message)
        _announce(f"抓取完成，等待 {args.release_delay_s:.1f} 秒后发送松夹爪指令")
        time.sleep(args.release_delay_s)
        server.send_message(args.release_message)
        _announce("导航端流程结束")
        return 0
    except KeyboardInterrupt:
        _announce("导航端收到中断，退出")
        return 130
    finally:
        server.close()


if __name__ == "__main__":
    sys.exit(main())
