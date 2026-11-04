#!/usr/bin/env python3
"""Read-only Agibot X2 ROS2 probe.

This intentionally uses ROS2 CLI introspection only. It never publishes a command
and never calls a service, so it is safe to run before the first hardware test.
Source ROS 2 Humble and the robot's aimdk_msgs local setup before invoking it.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys


GROUPS = ("head", "waist", "arm", "leg")


def run(*args: str) -> tuple[int, str]:
    proc = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, check=False)
    return proc.returncode, proc.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="print machine-readable output")
    args = parser.parse_args()
    if shutil.which("ros2") is None:
        print("ros2 CLI not found; source the X2 ROS2 environment first", file=sys.stderr)
        return 2

    result = {"groups": {}, "services": []}
    rc, topics = run("ros2", "topic", "list", "-t")
    if rc != 0:
        print(topics, file=sys.stderr)
        return rc
    topic_lines = set(topics.splitlines())
    for group in GROUPS:
        prefix = f"/aima/hal/joint/{group}"
        state = next((line for line in topic_lines if f"{prefix}/state " in line), None)
        command = next((line for line in topic_lines if f"{prefix}/command " in line), None)
        result["groups"][group] = {"state": state, "command": command}

    rc, services = run("ros2", "service", "list", "-t")
    if rc == 0:
        result["services"] = [line for line in services.splitlines()
                               if "GetAllJointState" in line or "SystemState" in line]
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        for group, values in result["groups"].items():
            print(f"{group}: state={values['state'] or 'MISSING'} command={values['command'] or 'MISSING'}")
        print("services:")
        for service in result["services"]:
            print(f"  {service}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
