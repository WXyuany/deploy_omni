#!/usr/bin/env python3
"""Verify D3 joints sequentially, using small timed moves and live feedback."""
import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

SIDES = ('left', 'right')
TOLERANCE = 0.008
DRIFT_LIMIT = 0.015
STALE_SECONDS = 0.5


def vector(values, label):
    if not isinstance(values, list) or len(values) != 7:
        raise RuntimeError(f'{label}: expected seven joint values')
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise RuntimeError(f'{label}: non-finite value')
    return result


class Feedback:
    def __init__(self, robot):
        self.robot = robot
        self.last = {side: (None, 0.0) for side in SIDES}

    def read(self, idle=False):
        if not self.robot.is_connected():
            raise RuntimeError('D3 connection lost')
        states = {}
        for index, side in enumerate(SIDES):
            # SDK 0.1.20260819 exposes global errors and the frame sequence here.
            raw = getattr(self.robot, '_' + side)._impl.state
            if not raw or len(raw.get('arm', [])) != 2:
                raise RuntimeError(f'{side}: missing dual-arm state')
            sequence = (raw.get('count'), raw.get('Unix_time_stamp'))
            if sequence == (None, None):
                raise RuntimeError(f'{side}: missing frame sequence')
            previous, received = self.last[side]
            now = time.monotonic()
            if sequence != previous:
                received = now
                self.last[side] = (sequence, received)
            if now - received > STALE_SECONDS:
                raise RuntimeError(f'{side}: feedback older than {STALE_SECONDS}s')
            if raw.get('debug') or raw.get('on_debug_mode'):
                raise RuntimeError('Simulation/debug mode is active')
            if raw.get('error', 0) or raw.get('errMsg'):
                raise RuntimeError(f'Controller error: {raw.get("error")} {raw.get("errMsg")}')
            state = copy.deepcopy(raw['arm'][index])
            if state.get('arm_dof') != 7 or 'D3' not in state.get('arm_name', ''):
                raise RuntimeError(f'{side}: not a seven-axis D3 arm')
            if state.get('servo') != 1 or state.get('fsm_state') != 'POSITION':
                raise RuntimeError(f'{side}: enable POSITION mode in the official UI first')
            if state.get('state') not in (0, 1):
                raise RuntimeError(f'{side}: unsafe controller state {state.get("state")}')
            state['q'] = vector(state.get('reality', {}).get('pose'), side + ' position')
            state['dq'] = vector(state.get('reality', {}).get('vel'), side + ' velocity')
            if max(abs(value) for value in state['dq']) > 0.1:
                raise RuntimeError(f'{side}: joint velocity exceeds 0.1 rad/s')
            if idle and (state['state'] != 0 or state.get('task', {}).get('exe_flag') or
                         max(abs(value) for value in state['dq']) > 0.01):
                raise RuntimeError(f'{side}: arm is not stationary and idle')
            states[side] = state
        return states


def plan_moves(home, limits, sides, joints, amplitude):
    moves = []
    for side in sides:
        lower = vector(limits[side].get('limit_lower'), side + ' lower limits')
        upper = vector(limits[side].get('limit_upper'), side + ' upper limits')
        if any(lo >= hi or not lo <= q <= hi for q, lo, hi in zip(home[side], lower, upper)):
            raise RuntimeError(f'{side}: invalid limits or initial position outside limits')
        for joint in joints:
            index = joint - 1
            candidates = (amplitude, -amplitude)
            offset = next((delta for delta in candidates if
                           lower[index] + 0.05 < home[side][index] + delta < upper[index] - 0.05), None)
            if offset is None:
                raise RuntimeError(f'{side} J{joint}: no target with 0.05 rad limit margin')
            target = list(home[side])
            target[index] += offset
            moves.append(dict(side=side, joint=joint, target=target, offset=offset))
    return moves


def check_envelope(states, home, side, joint, amplitude):
    for current_side in SIDES:
        for index, (actual, initial) in enumerate(zip(states[current_side]['q'], home[current_side])):
            limit = amplitude + DRIFT_LIMIT if (current_side, index) == (side, joint - 1) else DRIFT_LIMIT
            if abs(actual - initial) > limit:
                raise RuntimeError(f'{current_side} J{index + 1}: unexpected displacement')


def move_and_wait(robot, feedback, home, move, target, seconds):
    side, joint = move['side'], move['joint']
    initial = feedback.read(idle=True)
    check_envelope(initial, home, side, joint, abs(move['offset']))
    code = getattr(robot, 'move_' + side + '_joint')(
        list(target), desire_time=seconds, is_sync=False)
    if code != 1:
        raise RuntimeError(f'{side} J{joint}: command rejected ({code})')
    started = time.monotonic()
    deadline = started + seconds + 3
    settled = None
    peak_velocity = 0.0
    while time.monotonic() < deadline:
        states = feedback.read()
        check_envelope(states, home, side, joint, abs(move['offset']))
        current = states[side]
        peak_velocity = max(peak_velocity, *(abs(value) for value in current['dq']))
        error = max(abs(actual - desired) for actual, desired in zip(current['q'], target))
        stopped = (current['state'] == 0 and not current.get('task', {}).get('exe_flag') and
                   max(abs(value) for value in current['dq']) < 0.01)
        if time.monotonic() - started >= seconds and error <= TOLERANCE and stopped:
            settled = settled or time.monotonic()
            if time.monotonic() - settled >= 0.3:
                return dict(target=target, observed=current['q'], max_error=error,
                            peak_velocity=peak_velocity, elapsed=time.monotonic() - started)
        else:
            settled = None
        time.sleep(0.05)
    raise RuntimeError(f'{side} J{joint}: target not reached and settled before timeout')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default='10.42.0.101')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--arm', choices=(*SIDES, 'both'), default='both')
    parser.add_argument('--joints', nargs='+', type=int, default=list(range(1, 8)))
    parser.add_argument('--amplitude', type=float, default=0.035)
    parser.add_argument('--move-time', type=float, default=4)
    parser.add_argument('--check', action='store_true', help='read state/limits and print plan only')
    parser.add_argument('--yes', action='store_true', help='operator has confirmed clear space and emergency stop')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not args.ip:
        parser.error('valid IP and port required')
    if not math.isfinite(args.amplitude) or not 0.025 <= args.amplitude <= 0.05:
        parser.error('--amplitude must be 0.025..0.05 rad')
    if not math.isfinite(args.move_time) or not 4 <= args.move_time <= 15:
        parser.error('--move-time must be 4..15 seconds')
    if len(set(args.joints)) != len(args.joints) or any(joint not in range(1, 8) for joint in args.joints):
        parser.error('--joints must be unique values from 1..7')
    return args


def main():
    args = parse_args()
    from carm import CArmDualBot
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    report_path = args.report or Path(__file__).resolve().parents[2] / f'logs/verification/d3-joints-{stamp}.json'
    report = dict(result='FAIL', ip=args.ip, amplitude=args.amplitude,
                  move_time=args.move_time, tolerance=TOLERANCE, steps=[])
    robot = None
    motion_started = False
    code = 1
    try:
        robot = CArmDualBot(args.ip, port=args.port, timeout=2)
        time.sleep(0.5)
        feedback = Feedback(robot)
        states = feedback.read(idle=True)
        home = {side: states[side]['q'] for side in SIDES}
        limits = {side: getattr(robot, 'get_' + side + '_config')() for side in SIDES}
        sides = SIDES if args.arm == 'both' else (args.arm,)
        moves = plan_moves(home, limits, sides, args.joints, args.amplitude)
        report.update(home=home, limits=limits, plan=moves)
        print(json.dumps(dict(home=home, plan=moves), indent=2), flush=True)
        time.sleep(0.6)
        feedback.read(idle=True)
        if args.check:
            report['result'] = 'READ_ONLY_OK'
        else:
            if not args.yes:
                input('确认两臂周围无人无障碍、可随时急停后按回车开始，Ctrl+C 取消: ')
            states = feedback.read(idle=True)
            if any(abs(a - b) > TOLERANCE for side in SIDES for a, b in zip(states[side]['q'], home[side])):
                raise RuntimeError('Initial pose changed; rerun to sample a new plan')
            motion_started = True
            for move in moves:
                for phase, target in (('out', move['target']), ('return', home[move['side']])):
                    print(f"START {move['side']} J{move['joint']} {phase}", flush=True)
                    result = move_and_wait(robot, feedback, home, move, target, args.move_time)
                    step = dict(side=move['side'], joint=move['joint'], phase=phase, **result)
                    report['steps'].append(step)
                    print(json.dumps(step), flush=True)
                    time.sleep(0.5)
            feedback.read(idle=True)
            report['result'] = 'PASS'
        code = 0
    except (KeyboardInterrupt, EOFError):
        report['result'] = 'ABORTED'
        code = 130
    except Exception as error:
        report['error'] = str(error)
        print('ERROR: ' + str(error), file=sys.stderr, flush=True)
    finally:
        if robot is not None:
            if motion_started and report['result'] != 'PASS':
                try:
                    report['emergency_stop'] = robot.emergency_stop()
                    print('Emergency stop result: ' + str(report['emergency_stop']), flush=True)
                except Exception as error:
                    report['stop_error'] = str(error)
            robot.disconnect()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(f"[{report['result']}] Report: {report_path}", flush=True)
    return code


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, interrupted)
    raise SystemExit(main())
