#!/usr/bin/env python3
"""Read A3 or D3 state without ready, servo, motion or gripper commands."""
import argparse
import json
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=('a3', 'd3'), required=True)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    from carm import CArmSingleCol, CArmDualBot

    robot_class = CArmSingleCol if args.model == 'a3' else CArmDualBot
    robot = robot_class(args.ip, port=args.port, timeout=2)
    try:
        deadline = time.monotonic() + 2
        while not robot.is_connected() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not robot.is_connected():
            raise RuntimeError('Controller not connected')
        if args.model == 'a3':
            result = {'status': robot.get_status(), 'joint_pos': robot.get_joint_pos(),
                      'cart_pose': robot.get_cart_pose()}
            positions = [result['joint_pos']]
        else:
            result = {
                'left': {'status': robot.get_left_status(),
                         'joint_pos': robot.get_left_joint_pos(),
                         'cart_pose': robot.get_left_cart_pose()},
                'right': {'status': robot.get_right_status(),
                          'joint_pos': robot.get_right_joint_pos(),
                          'cart_pose': robot.get_right_cart_pose()}}
            positions = [result[side]['joint_pos'] for side in ('left', 'right')]
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if not all(positions):
            raise RuntimeError('Empty joint state; inspect controller readiness')
        return 0
    finally:
        robot.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())
