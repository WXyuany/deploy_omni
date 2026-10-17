#!/usr/bin/env python3
"""CArm A3 joint and gripper acceptance demo.

The demo moves one joint at a time, returns to the sampled home pose, and
then opens/closes the single-axis gripper with a low force. It does not use
low-level passthrough commands.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a visible, bounded A3 six-joint and gripper demo."
    )
    parser.add_argument("--ip", default="10.42.0.101", help="CArm controller IP")
    parser.add_argument("--port", type=int, default=8090, help="CArm WebSocket port")
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.35,
        help="joint offset in radians; joint 2 uses the negative direction",
    )
    parser.add_argument(
        "--move-time",
        type=float,
        default=3.0,
        help="seconds for each joint move",
    )
    parser.add_argument(
        "--gripper-open",
        type=float,
        default=0.04,
        help="gripper opening in metres",
    )
    parser.add_argument(
        "--gripper-torque",
        type=float,
        default=5.0,
        help="gripper force/torque limit for this test",
    )
    parser.add_argument(
        "--skip-gripper", action="store_true", help="skip the gripper test"
    )
    parser.add_argument(
        "--yes", action="store_true", help="start without the interactive confirmation"
    )
    return parser.parse_args()


def _print(label: str, value: Any) -> None:
    print(f"{label}: {value}", flush=True)


def _check_motion_target(
    target: list[float], lower: list[float], upper: list[float], joint_index: int
) -> None:
    value = target[joint_index]
    if not math.isfinite(value) or not lower[joint_index] + 0.05 < value < upper[joint_index] - 0.05:
        raise RuntimeError(
            f"joint {joint_index} target {value:.6f} is outside the 0.05 rad safety margin"
        )


def main() -> int:
    args = _parse_args()
    if args.amplitude <= 0 or args.move_time <= 0:
        raise RuntimeError("--amplitude and --move-time must be positive")
    if args.gripper_open < 0 or args.gripper_torque <= 0:
        raise RuntimeError("gripper opening must be non-negative and torque positive")

    try:
        from carm import CArmSingleCol
    except ImportError as error:
        raise RuntimeError(
            "Install the official carm package in your Python environment; see docs/cvte_carm/README.md"
        ) from error

    robot = None
    home: list[float] | None = None
    completed = False
    try:
        robot = CArmSingleCol(args.ip, port=args.port, timeout=2)
        time.sleep(0.4)
        status = robot.get_status()
        debug_mode = bool((robot._impl.state or {}).get("debug", False))
        _print(
            "precheck",
            {
                "connected": robot.is_connected(),
                "debug": debug_mode,
                "arm_name": status.get("arm_name"),
                "arm_dof": status.get("arm_dof"),
                "servo": status.get("servo"),
                "fsm_state": status.get("fsm_state"),
                "state": status.get("state"),
            },
        )
        if debug_mode or not robot.is_connected():
            raise RuntimeError("controller is not connected to a real arm")
        if status.get("arm_dof") != 6:
            raise RuntimeError(f"this demo requires an A3 six-axis arm: {status}")
        home = [float(value) for value in robot.get_joint_pos()]
        config = robot.get_config()
        lower = [float(value) for value in config.get("limit_lower", [])]
        upper = [float(value) for value in config.get("limit_upper", [])]
        if len(home) != 6 or len(lower) != 6 or len(upper) != 6:
            raise RuntimeError("A3 joint state or limits have an unexpected width")
        _print("home_joint_pos", home)
        _print("joint_limits", {"lower": lower, "upper": upper})

        offsets = [args.amplitude, args.amplitude, -args.amplitude,
                   args.amplitude, args.amplitude, args.amplitude]
        _print("planned_offsets", offsets)
        if not args.yes:
            input("确认周围无障碍物后按回车开始运动测试，Ctrl+C 可中止: ")

        if robot.set_ready() != 1:
            raise RuntimeError("set_ready failed")

        for joint_index, offset in enumerate(offsets):
            target = list(home)
            target[joint_index] += offset
            _check_motion_target(target, lower, upper, joint_index)
            _print(f"joint_{joint_index}_target", target[joint_index])

            code = robot.move_joint(target, desire_time=args.move_time, is_sync=True)
            time.sleep(0.2)
            observed = [float(value) for value in robot.get_joint_pos()]
            _print(
                f"joint_{joint_index}_out",
                {"code": code, "target": target[joint_index], "observed": observed[joint_index]},
            )
            if code != 1:
                raise RuntimeError(f"joint {joint_index} outward move failed: {code}")

            code = robot.move_joint(list(home), desire_time=args.move_time, is_sync=True)
            time.sleep(0.2)
            returned = [float(value) for value in robot.get_joint_pos()]
            _print(
                f"joint_{joint_index}_return",
                {"code": code, "observed": returned[joint_index],
                 "error": returned[joint_index] - home[joint_index]},
            )
            if code != 1:
                raise RuntimeError(f"joint {joint_index} return failed: {code}")

        if not args.skip_gripper:
            eeff = robot.get_eeff_config()
            lower_eeff = float(eeff.get("eeff_lower", [0.0])[0])
            upper_eeff = float(eeff.get("eeff_upper", [0.077])[0])
            if eeff.get("eeff_type") != "gripper":
                raise RuntimeError(f"A3 end effector is not a gripper: {eeff}")
            if not lower_eeff <= args.gripper_open <= upper_eeff:
                raise RuntimeError("--gripper-open is outside the configured gripper limits")

            _print(
                "gripper_precheck",
                {"type": eeff.get("eeff_type"), "connected": robot.get_eeff_connect(),
                 "limits": [lower_eeff, upper_eeff],
                 "home": robot.get_gripper_pos(), "target": args.gripper_open},
            )
            code = robot.set_gripper(args.gripper_open, tau=args.gripper_torque)
            time.sleep(1.5)
            _print("gripper_open", {"code": code, "observed": robot.get_gripper_pos()})
            if code != 1:
                raise RuntimeError(f"gripper open failed: {code}")

            code = robot.set_gripper(lower_eeff, tau=args.gripper_torque)
            time.sleep(1.5)
            _print("gripper_close", {"code": code, "observed": robot.get_gripper_pos()})
            if code != 1:
                raise RuntimeError(f"gripper close failed: {code}")

        final_status = robot.get_status()
        _print(
            "final",
            {"joint_pos": robot.get_joint_pos(), "gripper_pos": robot.get_gripper_pos(),
             "servo": final_status.get("servo"), "fsm_state": final_status.get("fsm_state"),
             "state": final_status.get("state"), "task": final_status.get("task")},
        )
        _print("result", "PASS")
        completed = True
        return 0
    except KeyboardInterrupt:
        _print("result", "ABORTED_BY_USER")
        return 130
    finally:
        if robot is not None and not completed:
            try:
                _print("abort_emergency_stop", robot.emergency_stop())
            except Exception as error:
                _print("abort_stop_error", repr(error))
        if robot is not None:
            robot.disconnect()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        raise SystemExit(1)
