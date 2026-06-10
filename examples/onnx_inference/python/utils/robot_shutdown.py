"""Shared hardware-shutdown helpers for the real-robot eval entrypoints.

act_evaluate.py drives a live SO-101 arm + cameras
and must release them cleanly on Ctrl+C. The two concerns below are common to
every such entrypoint, so they live here rather than being duplicated per script.
"""

from __future__ import annotations

import contextlib
import signal


@contextlib.contextmanager
def suspend_sigint():
    """Ignore SIGINT inside the block.

    Releasing the arm takes a few seconds (serial torque-disable with retries +
    each camera thread joining up to 2s). If an impatient second Ctrl+C lands in
    that window it would abort ``bus.disconnect()`` *before* ``closePort()`` runs,
    leaving the motor torque on and the serial port held open (next run fails with
    'device busy'). Deferring SIGINT makes the cleanup atomic.

    No-op outside the main thread (``signal.signal`` raises there).
    """
    try:
        prev = signal.signal(signal.SIGINT, signal.SIG_IGN)
    except ValueError:  # not in main thread — nothing to shield
        prev = None
    try:
        yield
    finally:
        if prev is not None:
            signal.signal(signal.SIGINT, prev)


def safe_disconnect(robot, log=print) -> None:
    """Best-effort hardware release that never leaks one subsystem on another's failure.

    ``SO101Follower.disconnect()`` disables motor torque first and only then
    releases the cameras, so if the serial side raises (e.g. the link was
    interrupted mid-frame) the cameras would never be freed. Fall back to releasing
    each part independently, and force the serial port closed even if torque-disable
    fails.

    Args:
        robot: a connected (or partially connected) lerobot robot.
        log: callable used for diagnostics; defaults to ``print`` (pass
            ``logging.warning`` from scripts that use the logging module).
    """
    try:
        robot.disconnect()
        return
    except Exception as exc:  # noqa: BLE001 - cleanup must be exhaustive
        log(f"robot.disconnect() failed ({exc!r}); releasing parts individually.")

    bus = getattr(robot, "bus", None)
    if bus is not None:
        try:
            bus.disconnect(True)  # disable torque + close port
        except Exception as exc:  # noqa: BLE001
            log(f"  torque-disable failed ({exc!r}); forcing port close.")
            with contextlib.suppress(Exception):
                bus.disconnect(False)  # at least close the serial port

    for name, cam in getattr(robot, "cameras", {}).items():
        try:
            cam.disconnect()
        except Exception as exc:  # noqa: BLE001
            log(f"  camera '{name}' release failed: {exc!r}")
