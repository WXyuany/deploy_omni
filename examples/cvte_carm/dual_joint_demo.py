#!/usr/bin/env python3
"""Inspect D3 joint plans or explicitly run small joint tests from a reference posture."""
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
GRIPPER_TOLERANCE = 0.005
MAX_AVERAGE_SPEED = 0.04


def vector(values, label):
    if not isinstance(values, list) or len(values) != 7:
        raise RuntimeError(f'{label}: expected seven joint values')
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise RuntimeError(f'{label}: non-finite value')
    return result


class RequestTrace:
    """Keep vendor responses that the public SDK otherwise collapses to -1."""
    def __init__(self, robot):
        self.entries = []
        self.originals = []
        for side in SIDES:
            impl = getattr(robot, '_' + side)._impl
            original = impl.request
            self.originals.append((impl, original))

            def request(payload, *args, _side=side, _original=original, _impl=impl, **kwargs):
                entry = dict(side=_side, request=copy.deepcopy(payload))
                try:
                    response = _original(payload, *args, **kwargs)
                    entry['response'] = copy.deepcopy(response)
                    if payload.get('command') == 'webRecieveTasks' and response.get('recv') != 'Task_Recieve':
                        entry['controller_state'] = copy.deepcopy(_impl.state)
                        print(f'SDK_REPLY {_side}: ' + json.dumps(response), flush=True)
                    return response
                except Exception as error:
                    entry['error'] = str(error)
                    raise
                finally:
                    self.entries.append(entry)

            impl.request = request

    def close(self):
        for impl, original in self.originals:
            impl.request = original


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


def require_motion_speed(states, sides):
    for side in sides:
        value = states[side].get('vel_per')
        if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                not math.isfinite(value) or not 0 < value <= 1):
            raise RuntimeError(
                f'{side}: invalid motion speed scale vel_per={value!r}; '
                'confirm stop recovery and a nonzero speed in the official UI')


def load_motion_profile(args):
    path = Path(__file__).resolve().parents[2] / 'config/cvte_carm_d3.motion.json'
    profile = json.loads(path.read_text())
    if args.test != 'joints':
        raise RuntimeError('Motion execution supports joint tests only')
    if args.amplitude > profile['max_amplitude_rad'] or args.move_time < profile['min_move_time_seconds']:
        raise RuntimeError('D3 motion requires amplitude <= 0.035 rad and move-time >= 4 seconds')
    return profile


def require_reference_posture(states, profile):
    for side in SIDES:
        expected = profile['arms'][side]
        reference = vector(expected['position'], side + ' reference posture')
        if states[side]['arm_name'] != expected['name']:
            raise RuntimeError(f'{side}: robot model does not match the motion profile')
        error = max(abs(actual - target) for actual, target in zip(states[side]['q'], reference))
        if error > profile['start_tolerance_rad']:
            raise RuntimeError(
                f'{side}: current posture differs from the reference by {error:.6f} rad; '
                'inspect and position the robot on site before retrying; no automatic repositioning')


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
    require_motion_speed(initial, (side,))
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
        require_motion_speed(states, (side,))
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


def scalar(values, label):
    if not isinstance(values, list) or len(values) != 1 or not math.isfinite(float(values[0])):
        raise RuntimeError(f'{label}: expected one finite value')
    return float(values[0])


def gripper_position(state):
    eeff = state.get('eeff', {})
    if (eeff.get('eeff_type') != 'gripper' or eeff.get('eeff_dof') != 1 or
            not eeff.get('is_connect') or eeff.get('eeff_state') != 1):
        raise RuntimeError('Gripper is not connected and enabled')
    return scalar(eeff.get('eeff_pos'), 'gripper position')


def plan_grippers(states, configs, sides, opening, closing, force):
    moves = []
    for side in sides:
        initial = gripper_position(states[side])
        config = configs[side]
        lower = scalar(config.get('eeff_lower'), side + ' gripper lower')
        upper = scalar(config.get('eeff_upper'), side + ' gripper upper')
        maximum_force = scalar(config.get('eeff_tau'), side + ' gripper force')
        if not lower <= closing < opening <= upper or not lower <= initial <= upper:
            raise RuntimeError(f'{side}: gripper targets or initial opening outside limits')
        if maximum_force < force:
            raise RuntimeError(f'{side}: gripper force exceeds device configuration')
        for phase, target in (('open', opening), ('close', closing)):
            moves.append(dict(side=side, phase=phase, target=target, force=force))
    return moves


def grip_and_wait(robot, feedback, home, move):
    side, target = move['side'], move['target']
    states = feedback.read(idle=True)
    initial = {name: gripper_position(states[name]) for name in SIDES}
    code = getattr(robot, 'set_' + side + '_gripper')(target, tau=move['force'])
    if code != 1:
        raise RuntimeError(f'{side} gripper: command rejected ({code})')
    started = time.monotonic()
    settled = None
    samples = []
    while time.monotonic() - started < 6:
        states = feedback.read(idle=True)
        check_envelope(states, home, None, 0, 0)
        observed = gripper_position(states[side])
        for name in SIDES:
            position = gripper_position(states[name])
            lo, hi = sorted((initial[name], target if name == side else initial[name]))
            margin = GRIPPER_TOLERANCE if name == side else 0.003
            if not lo - margin <= position <= hi + margin:
                raise RuntimeError(f'{name} gripper: unexpected displacement')
        elapsed = time.monotonic() - started
        samples.append(dict(elapsed=elapsed, position=observed))
        if abs(observed - target) <= GRIPPER_TOLERANCE and elapsed >= 1:
            settled = settled or time.monotonic()
            if time.monotonic() - settled >= 0.3:
                return dict(target=target, observed=observed, max_error=abs(observed - target),
                            force_command=move['force'], samples=samples)
        else:
            settled = None
        time.sleep(0.05)
    raise RuntimeError(f'{side} gripper: target {target:.4f} m, observed {observed:.4f} m after timeout')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default='10.42.0.101')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--arm', choices=(*SIDES, 'both'), default='both')
    parser.add_argument('--test', choices=('joints', 'gripper', 'all'), default='joints')
    parser.add_argument('--joints', nargs='+', type=int, default=list(range(1, 8)))
    parser.add_argument('--amplitude', type=float, default=0.035)
    parser.add_argument('--move-time', type=float, default=4)
    parser.add_argument('--pause', type=float, default=0.1,
                        help='extra pause after each completed segment, 0..5 seconds')
    parser.add_argument('--gripper-open', type=float, default=0.03)
    parser.add_argument('--gripper-close', type=float, default=0.005)
    parser.add_argument('--gripper-force', type=float, default=5)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='read state/limits and print plan only')
    mode.add_argument('--enable-motion', action='store_true', help='run small joint tests from the reference posture')
    parser.add_argument('--yes', action='store_true', help='operator has confirmed clear space and emergency stop')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not args.ip:
        parser.error('valid IP and port required')
    if not math.isfinite(args.amplitude) or not 0.025 <= args.amplitude <= 0.20:
        parser.error('--amplitude must be 0.025..0.20 rad')
    if not math.isfinite(args.move_time) or not 4 <= args.move_time <= 15:
        parser.error('--move-time must be 4..15 seconds')
    if not math.isfinite(args.pause) or not 0 <= args.pause <= 5:
        parser.error('--pause must be 0..5 seconds')
    if args.amplitude / args.move_time > MAX_AVERAGE_SPEED + 1e-12:
        parser.error(f'--amplitude / --move-time must not exceed {MAX_AVERAGE_SPEED} rad/s')
    if len(set(args.joints)) != len(args.joints) or any(joint not in range(1, 8) for joint in args.joints):
        parser.error('--joints must be unique values from 1..7')
    if (not all(math.isfinite(value) for value in
                (args.gripper_open, args.gripper_close, args.gripper_force)) or
            not 0.003 <= args.gripper_close < args.gripper_open <= 0.04 or
            args.gripper_open - args.gripper_close <= 2 * GRIPPER_TOLERANCE or
            not 0 < args.gripper_force <= 5):
        parser.error('gripper: require 0.003 <= close < open <= 0.04 m, travel > 0.01 m, 0 < force <= 5 N')
    return args


def main():
    args = parse_args()
    if not (args.check or args.enable_motion):
        print('Select --check for read-only inspection or --enable-motion for small joint tests.',
              file=sys.stderr, flush=True)
        return 2
    profile = None
    if args.enable_motion:
        try:
            profile = load_motion_profile(args)
        except (OSError, ValueError, RuntimeError) as error:
            print('ERROR: ' + str(error), file=sys.stderr, flush=True)
            return 2
    from carm import CArmDualBot
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    report_path = args.report or Path(__file__).resolve().parents[2] / f'logs/verification/d3-joints-{stamp}.json'
    report = dict(result='FAIL', ip=args.ip, test=args.test, amplitude=args.amplitude,
                  move_time=args.move_time, pause=args.pause, tolerance=TOLERANCE,
                  average_speed=args.amplitude / args.move_time, steps=[])
    robot = None
    trace = None
    motion_started = False
    code = 1
    try:
        robot = CArmDualBot(args.ip, port=args.port, timeout=2)
        trace = RequestTrace(robot)
        time.sleep(0.5)
        feedback = Feedback(robot)
        states = feedback.read(idle=True)
        if profile is not None:
            require_reference_posture(states, profile)
            report['motion_profile'] = profile
        home = {side: states[side]['q'] for side in SIDES}
        limits = {side: getattr(robot, 'get_' + side + '_config')() for side in SIDES}
        sides = SIDES if args.arm == 'both' else (args.arm,)
        moves = plan_moves(home, limits, sides, args.joints, args.amplitude) if args.test != 'gripper' else []
        grippers = []
        if args.test != 'joints':
            configs = {side: getattr(robot, 'get_' + side + '_eeff_config')() for side in SIDES}
            grippers = plan_grippers(states, configs, sides, args.gripper_open,
                                     args.gripper_close, args.gripper_force)
            report['gripper_config'] = configs
        report.update(home=home, limits=limits, plan=moves, gripper_plan=grippers)
        print(json.dumps(dict(home=home, plan=moves, gripper_plan=grippers), indent=2), flush=True)
        time.sleep(0.6)
        states = feedback.read(idle=True)
        report['precheck'] = states
        fields = ('servo', 'fsm_state', 'state', 'motion_state', 'vel_per')
        print('PRECHECK ' + json.dumps({side: {key: states[side].get(key) for key in fields}
                                        for side in SIDES}), flush=True)
        if moves:
            require_motion_speed(states, sides)
        if args.check:
            report['result'] = 'READ_ONLY_OK'
        else:
            if not args.yes:
                input('确认双臂全程不会接触机身或障碍、可随时急停后按回车开始，Ctrl+C 取消: ')
            states = feedback.read(idle=True)
            report['pre_motion'] = states
            require_reference_posture(states, profile)
            if moves:
                require_motion_speed(states, sides)
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
                    time.sleep(args.pause)
            for move in grippers:
                print(f"START {move['side']} gripper {move['phase']}", flush=True)
                result = grip_and_wait(robot, feedback, home, move)
                step = dict(side=move['side'], phase=move['phase'], actuator='gripper', **result)
                report['steps'].append(step)
                print(json.dumps(step), flush=True)
                time.sleep(args.pause)
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
        if trace is not None:
            report['sdk_requests'] = trace.entries
            trace.close()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(f"[{report['result']}] Report: {report_path}", flush=True)
    return code


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, interrupted)
    raise SystemExit(main())
