#!/usr/bin/env python3
"""Build and run one reproducible verification case from the project root."""
import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Case:
    profile: str
    targets: tuple
    command: tuple
    evidence: str
    verdict: str = 'PASS'


CASES = {
    ('g1', 'state'): Case('unitree_g1', ('unitree_g1_motion_demo',),
        ('{build}/unitree_g1_motion_demo', '--check', '--interface', '{interface}'),
        'Read-only FSM and fresh LowState/CRC/health checks.'),
    ('g1', 'camera'): Case('unitree_g1', ('unitree_g1_sdk_check',),
        ('{build}/unitree_g1_sdk_check', '--test', 'camera', '--interface', '{interface}',
         '--output', '{output}/front_camera.jpg'), 'JPEG received and saved.'),
    ('g1', 'device'): Case('unitree_g1', ('unitree_g1_sdk_check',),
        ('{build}/unitree_g1_sdk_check', '--test', 'device', '--interface', '{interface}'),
        'ServiceList succeeded and returned a non-empty device service list.'),
    ('g1', 'audio'): Case('unitree_g1', ('unitree_g1_sdk_check',),
        ('{build}/unitree_g1_sdk_check', '--test', 'audio', '--interface', '{interface}'),
        'API checks passed; listen for speech. Original volume is restored.', 'API_OK'),
    ('g1', 'led'): Case('unitree_g1', ('unitree_g1_sdk_check',),
        ('{build}/unitree_g1_sdk_check', '--test', 'led', '--interface', '{interface}'),
        'API checks passed; observe red/green/blue. LED finishes off.', 'API_OK'),
    ('g1', 'pointcloud'): Case('unitree_g1', ('unitree_g1_sensor_reader',),
        ('{build}/unitree_g1_sensor_reader', '--stream', 'pointcloud', '--interface',
         '{interface}', '--duration', '{seconds}'), 'PointCloud2 samples received.'),
    ('g1', 'odometry'): Case('unitree_g1', ('unitree_g1_sensor_reader',),
        ('{build}/unitree_g1_sensor_reader', '--stream', 'odometry', '--interface',
         '{interface}', '--duration', '{seconds}'), 'Odometry samples received.'),
    ('g1', 'motion'): Case('unitree_g1', ('unitree_g1_motion_demo',),
        ('{build}/unitree_g1_motion_demo', '--enable-motion', '--interface', '{interface}'),
        'Keyboard session ended; verify movement and stopping on site.', 'COMPLETED'),
    ('g1', 'offline'): Case('unitree_g1', (),
        ('ctest', '--test-dir', '{build}', '--output-on-failure', '--no-tests=error'),
        'Offline tests only; no robot validation.'),
    ('k2', 'offline'): Case('kepler_k2', (),
        ('ctest', '--test-dir', '{build}', '--output-on-failure', '--no-tests=error'),
        'Simulated K2 and common offline tests; no real motor_api.'),
}
for model in ('a3', 'd3'):
    CASES[(model, 'state')] = Case('cvte_carm', ('cvte_carm_check',),
        ('{build}/cvte_carm_check', '--model', model, '--ip', '{ip}', '--port', '{port}'),
        'Read-only connection, joint count, state and health checks.')
    CASES[(model, 'offline')] = Case('', (),
        ('{python}', 'tests/demo/cvte_carm_examples_test.py'),
        'Python SDK doubles only; no hardware connection.')
for feature, selection in (('joints', '--skip-gripper'), ('gripper', '--only-gripper'),
                           ('joint-gripper', '')):
    CASES[('a3', feature)] = Case('', (),
        ('{python}', 'examples/cvte_carm/joint_gripper_demo.py', '--ip', '{ip}',
         '--port', '{port}', '--amplitude', '0.15', '--move-time', '3') +
        ((selection,) if selection else ()),
        'Motion confirmed by operator; feedback checked against configured tolerances.')
CASES[('d3', 'joints')] = Case('', (),
    ('{python}', 'examples/cvte_carm/dual_joint_demo.py', '--ip', '{ip}',
     '--port', '{port}', '--report', '{output}/joints.json'),
    'Both arms: seven joints each, 0.035 rad timed moves and returns; live feedback checked.')


def positive(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError('must be a positive integer')
    return number


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
        epilog='Examples: g1 camera --interface eth0; a3 joints --ip 10.42.0.101; k2 offline')
    parser.add_argument('robot', nargs='?', choices=('g1', 'k2', 'a3', 'd3'))
    parser.add_argument('feature', nargs='?', help='available cases are listed by --list')
    parser.add_argument('--list', action='store_true', help='list cases without building or running', default=False)
    parser.add_argument('--interface', default='eth0')
    parser.add_argument('--ip', default='10.42.0.101')
    parser.add_argument('--port', type=positive, default=8090)
    parser.add_argument('--seconds', type=positive, default=10)
    parser.add_argument('--jobs', type=positive, default=2)
    parser.add_argument('--sdk-dir', default=os.environ.get('BRIDGE_CVTE_SDK_DIR', str(ROOT / 'arm_control_sdk')))
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--skip-build', action='store_true', help='use an existing verification build')
    parser.add_argument('--dry-run', action='store_true', help='print commands only; do not connect or write files')
    args = parser.parse_args(argv)
    if args.port > 65535 or args.seconds > 86400:
        parser.error('port must be <=65535; seconds must be <=86400')
    if not args.interface or not args.ip:
        parser.error('interface and IP must be non-empty')
    if not args.list and (args.robot, args.feature) not in CASES:
        parser.error('unknown case; use --list to see available features')
    return args


def commands(args, output):
    case = CASES[(args.robot, args.feature)]
    build_name = 'carm' if case.profile == 'cvte_carm' else args.robot
    build = (args.build_dir or ROOT / ('build-verify-' + build_name)).resolve()
    venv_python = ROOT / '.venv/bin/python'
    values = dict(build=str(build), output=str(output), interface=args.interface,
                  ip=args.ip, port=str(args.port), seconds=str(args.seconds),
                  python=str(venv_python) if venv_python.exists() else sys.executable)
    result = []
    if case.profile and not args.skip_build:
        configure = ['cmake', '-S', str(ROOT), '-B', str(build),
                     '-DBRIDGE_ROBOT=' + case.profile, '-DBUILD_GAMEPAD_SDL=OFF',
                     '-DBUILD_TESTING=ON']
        if case.profile == 'unitree_g1':
            hardware = 'ON' if 'unitree_g1_sdk_check' in case.targets else 'OFF'
            configure += ['-DBUILD_UNITREE_G1_MOTION_DEMO=ON',
                          '-DBUILD_UNITREE_G1_HARDWARE_TESTS=' + hardware]
        if case.profile == 'cvte_carm':
            configure += ['-DBUILD_BRIDGE_DDS=OFF', '-DBRIDGE_CVTE_SDK_DIR=' + str(Path(args.sdk_dir).resolve())]
        build_command = ['cmake', '--build', str(build), '--parallel', str(args.jobs)]
        if case.targets:
            build_command += ['--target', *case.targets]
        result += [configure, build_command]
    result.append([part.format_map(values) for part in case.command])
    return result


def execute(command):
    child = subprocess.Popen(command, cwd=ROOT)
    try:
        return child.wait()
    except KeyboardInterrupt:
        # The foreground child receives Ctrl+C too; let its cleanup finish.
        child.wait()
        raise


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.list:
        print('Available verification cases (not executed):')
        for (robot, feature), case in CASES.items():
            print(f'{robot:3} {feature:14} {case.evidence}')
        return 0
    case = CASES[(args.robot, args.feature)]
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = ROOT / 'logs/verification' / f'{args.robot}-{args.feature}-{stamp}-{os.getpid()}'
    planned = commands(args, output)
    if args.dry_run:
        for command in planned:
            print(shlex.join(command))
        return 0
    output.mkdir(parents=True)
    report = dict(robot=args.robot, feature=args.feature, expected_result=case.evidence,
                  commands=planned, steps=[], result='FAIL', exit_code=1)
    code = 0
    try:
        for command in planned:
            print('+ ' + shlex.join(command), flush=True)
            code = execute(command)
            report['steps'].append(dict(command=command, exit_code=code))
            if code:
                break
        report['result'] = case.verdict if code == 0 else 'ABORTED' if code in (130, -2) else 'FAIL'
    except KeyboardInterrupt:
        code = 130
        report['result'] = 'ABORTED'
    except OSError as error:
        print(str(error), file=sys.stderr)
        report['error'] = str(error)
        code = 1
    finally:
        report['exit_code'] = code
        (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f"[{report['result']}] {args.robot}/{args.feature}: {case.evidence if code == 0 else 'see errors above'}")
    print('Report: ' + str(output / 'result.json'))
    return code if code >= 0 else 128 - code


if __name__ == '__main__':
    raise SystemExit(main())
