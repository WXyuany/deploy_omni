"""Offline SDK doubles: verify reads and interruption paths without hardware."""
import contextlib
import copy
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[2]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'examples/cvte_carm' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CarmExamplesTest(unittest.TestCase):
    def fake(self):
        robot = MagicMock()
        robot._impl.state = {'debug': False}
        robot.is_connected.return_value = True
        robot.get_status.return_value = {'arm_dof': 6}
        robot.get_joint_pos.return_value = [0.0] * 6
        robot.get_cart_pose.return_value = [0.0] * 6
        robot.get_config.return_value = {'limit_lower': [-2.0] * 6, 'limit_upper': [2.0] * 6}
        robot.set_ready.return_value = 1
        robot.get_eeff_config.return_value = {
            'eeff_type': 'gripper', 'eeff_lower': [0.0], 'eeff_upper': [0.077]}
        robot.get_eeff_connect.return_value = True
        robot.get_gripper_pos.return_value = 0.0
        robot.set_gripper.return_value = 1
        sdk = types.ModuleType('carm')
        sdk.CArmSingleCol = MagicMock(return_value=robot)
        sdk.CArmDualBot = MagicMock(return_value=robot)
        return robot, sdk

    def test_read_only_models(self):
        for model in ('a3', 'd3'):
            with self.subTest(model=model):
                robot, sdk = self.fake()
                for side in ('left', 'right'):
                    getattr(robot, 'get_' + side + '_status').return_value = {'arm_dof': 7}
                    getattr(robot, 'get_' + side + '_joint_pos').return_value = [0.0] * 7
                    getattr(robot, 'get_' + side + '_cart_pose').return_value = [0.0] * 6
                with patch.dict(sys.modules, {'carm': sdk}), \
                     patch.object(sys, 'argv', ['read_state', '--model', model, '--ip', 'test']), \
                     contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(load('read_state').main(), 0)
                robot.set_ready.assert_not_called()
                robot.set_servo_enable.assert_not_called()
                robot.move_joint.assert_not_called()
                robot.disconnect.assert_called_once()

    def test_cancel_before_ready(self):
        robot, sdk = self.fake()
        with patch.dict(sys.modules, {'carm': sdk}), \
             patch.object(sys, 'argv', ['joint_gripper_demo']), \
             patch('builtins.input', side_effect=KeyboardInterrupt), \
             patch('time.sleep'), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(load('joint_gripper_demo').main(), 130)
        robot.set_ready.assert_not_called()
        robot.move_joint.assert_not_called()
        robot.set_gripper.assert_not_called()
        robot.disconnect.assert_called_once()

    def test_motion_error_does_not_start_return_motion(self):
        robot, sdk = self.fake()
        robot.move_joint.side_effect = RuntimeError('SDK connection lost')
        with patch.dict(sys.modules, {'carm': sdk}), \
             patch.object(sys, 'argv', ['joint_gripper_demo', '--yes']), \
             patch('time.sleep'), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, 'SDK connection lost'):
                load('joint_gripper_demo').main()
        robot.move_joint.assert_called_once()
        robot.set_gripper.assert_not_called()
        robot.emergency_stop.assert_called_once()
        robot.disconnect.assert_called_once()

    def run_motion(self, robot, sdk, *args):
        with patch.dict(sys.modules, {'carm': sdk}), \
             patch.object(sys, 'argv', ['joint_gripper_demo', '--yes', *args]), \
             patch('time.sleep'), contextlib.redirect_stdout(io.StringIO()):
            return load('joint_gripper_demo').main()

    def test_success_code_without_joint_movement_fails(self):
        robot, sdk = self.fake()
        robot.move_joint.return_value = 1
        with self.assertRaisesRegex(RuntimeError, 'feedback did not reach'):
            self.run_motion(robot, sdk, '--skip-gripper')
        robot.move_joint.assert_called_once()
        robot.emergency_stop.assert_called_once()

    def test_six_joint_feedback_success(self):
        robot, sdk = self.fake()
        def move(target, **kwargs):
            robot.get_joint_pos.return_value = list(target)
            return 1
        robot.move_joint.side_effect = move
        self.assertEqual(self.run_motion(robot, sdk, '--skip-gripper'), 0)
        self.assertEqual(robot.move_joint.call_count, 12)
        robot.set_gripper.assert_not_called()
        robot.emergency_stop.assert_not_called()

    def test_only_gripper_does_not_move_joints(self):
        robot, sdk = self.fake()
        def grip(target, **kwargs):
            robot.get_gripper_pos.return_value = target
            return 1
        robot.set_gripper.side_effect = grip
        self.assertEqual(self.run_motion(robot, sdk, '--only-gripper'), 0)
        robot.move_joint.assert_not_called()
        self.assertEqual(robot.set_gripper.call_count, 2)

    def test_gripper_success_code_without_movement_fails(self):
        robot, sdk = self.fake()
        with self.assertRaisesRegex(RuntimeError, 'feedback did not reach'):
            self.run_motion(robot, sdk, '--only-gripper')
        robot.set_gripper.assert_called_once()
        robot.emergency_stop.assert_called_once()

    def test_invalid_targets_rejected_before_ready(self):
        for arguments in (['--amplitude', 'nan'], ['--gripper-open', '1'],
                          ['--amplitude', '0.01'], ['--port', '65536']):
            with self.subTest(arguments=arguments):
                robot, sdk = self.fake()
                with self.assertRaises(RuntimeError):
                    self.run_motion(robot, sdk, *arguments)
                robot.set_ready.assert_not_called()
                robot.move_joint.assert_not_called()


class D3JointTest(unittest.TestCase):
    def setUp(self):
        self.demo = load('dual_joint_demo')
        self.robot = MagicMock()
        self.robot.is_connected.return_value = True
        self.robot.emergency_stop.return_value = 1
        self.states = {}
        for side in ('left', 'right'):
            state = dict(arm_dof=7, arm_name='D3_AC_C_' + side[0], servo=1,
                         fsm_state='POSITION', state=0, vel_per=0.1, task={'exe_flag': False},
                         reality={'pose': [0.0] * 7, 'vel': [0.0] * 7},
                         eeff=dict(eeff_type='gripper', eeff_dof=1, is_connect=True,
                                   eeff_state=1, eeff_pos=[0.0]))
            self.states[side] = dict(state, q=[0.0] * 7, dq=[0.0] * 7)
        self.raw = dict(count='1', debug=False, error=0, errMsg='',
                        arm=[self.states[side] for side in ('left', 'right')])
        for side in ('left', 'right'):
            getattr(self.robot, '_' + side)._impl.state = self.raw
            getattr(self.robot, 'get_' + side + '_config').return_value = {
                'limit_lower': [-2.0] * 7, 'limit_upper': [2.0] * 7}
            getattr(self.robot, 'get_' + side + '_eeff_config').return_value = {
                'eeff_lower': [0.0], 'eeff_upper': [0.077], 'eeff_tau': [100.0]}

    def test_stale_fault_and_simulation_rejected(self):
        feedback = self.demo.Feedback(self.robot)
        with patch.object(self.demo.time, 'monotonic', return_value=10):
            feedback.read(idle=True)
        with patch.object(self.demo.time, 'monotonic', return_value=11):
            with self.assertRaisesRegex(RuntimeError, 'older'):
                feedback.read()
        for key, value in (('debug', True), ('error', 3001)):
            with self.subTest(key=key), patch.dict(self.raw, {key: value}):
                with self.assertRaises(RuntimeError):
                    self.demo.Feedback(self.robot).read()

    def test_plan_respects_limits_and_does_not_mutate_home(self):
        home = dict(left=[0.0] * 7, right=[0.0] * 7)
        limits = {side: {'limit_lower': [-2.0] * 7, 'limit_upper': [2.0] * 7}
                  for side in ('left', 'right')}
        limits['left']['limit_upper'][0] = 0.02
        moves = self.demo.plan_moves(home, limits, ('left',), [1], 0.035)
        self.assertEqual(moves[0]['offset'], -0.035)
        self.assertEqual(home['left'], [0.0] * 7)
        limits['left']['limit_lower'][0] = -0.02
        with self.assertRaises(RuntimeError):
            self.demo.plan_moves(home, limits, ('left',), [1], 0.035)

    def test_other_arm_displacement_rejected(self):
        states = copy.deepcopy(self.states)
        states['right']['q'][3] = 0.02
        with self.assertRaisesRegex(RuntimeError, 'right J4'):
            self.demo.check_envelope(states, dict(left=[0.0] * 7, right=[0.0] * 7),
                                     'left', 1, 0.035)

    def test_success_response_without_movement_times_out(self):
        feedback = MagicMock()
        feedback.read.return_value = self.states
        self.robot.move_left_joint.return_value = 1
        clock = [0.0]
        def sleep(seconds):
            clock[0] += seconds
        move = dict(side='left', joint=1, offset=0.035)
        with patch.object(self.demo.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(self.demo.time, 'sleep', side_effect=sleep):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                self.demo.move_and_wait(self.robot, feedback,
                    dict(left=[0.0] * 7, right=[0.0] * 7), move, [0.035] + [0.0] * 6, 4)
        self.robot.move_left_joint.assert_called_once()
        self.assertFalse(self.robot.move_left_joint.call_args.kwargs['is_sync'])
        self.robot.move_right_joint.assert_not_called()

    def run_demo(self, *args, failure=None):
        import tempfile
        sdk = types.ModuleType('carm')
        sdk.CArmDualBot = MagicMock(return_value=self.robot)
        self.factory = sdk.CArmDualBot
        with tempfile.TemporaryDirectory() as temp, \
             patch.dict(sys.modules, {'carm': sdk}), \
             patch.object(sys, 'argv', ['demo', '--report', str(Path(temp) / 'result.json'), *args]), \
             patch.object(self.demo, 'Feedback') as feedback, \
             patch.object(self.demo, 'move_and_wait', side_effect=failure) as move, \
             patch.object(self.demo.time, 'sleep'), \
             patch('builtins.input', side_effect=KeyboardInterrupt), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            feedback.return_value.read.return_value = self.states
            result = self.demo.main()
        return result, move

    def test_read_only_check_sends_no_control_commands(self):
        result, move = self.run_demo('--check')
        self.assertEqual(result, 0)
        move.assert_not_called()
        self.robot.set_ready.assert_not_called()
        self.robot.emergency_stop.assert_not_called()
        self.robot.disconnect.assert_called()

    def test_motion_modes_block_before_connection(self):
        for args in ([], ['--yes'], ['--test', 'gripper', '--yes'],
                     ['--test', 'all', '--yes'],
                     ['--arm', 'right', '--joints', '2', '--amplitude', '0.20',
                      '--move-time', '5', '--yes']):
            with self.subTest(args=args):
                result, move = self.run_demo(*args)
                self.assertEqual(result, 2)
                self.factory.assert_not_called()
                move.assert_not_called()
                self.robot.emergency_stop.assert_not_called()
                self.robot.set_ready.assert_not_called()
                self.robot.set_debug.assert_not_called()

    def test_large_or_gripper_motion_blocks_before_connection(self):
        for args in (['--amplitude', '0.20', '--move-time', '5'],
                     ['--test', 'gripper'], ['--test', 'all']):
            with self.subTest(args=args):
                result, move = self.run_demo('--enable-motion', '--yes', *args)
                self.assertEqual(result, 2)
                self.factory.assert_not_called()
                move.assert_not_called()

    def test_reference_posture_blocks_motion_without_clearing_stop(self):
        self.states['right']['q'][1] = 0.02
        result, move = self.run_demo('--enable-motion', '--yes')
        self.assertEqual(result, 1)
        move.assert_not_called()
        self.robot.emergency_stop.assert_not_called()
        self.robot.set_ready.assert_not_called()

    def test_small_joint_motion_keeps_failure_stop_and_no_automatic_return(self):
        result, move = self.run_demo('--enable-motion', '--yes', failure=RuntimeError('feedback lost'))
        self.assertEqual(result, 1)
        move.assert_called_once()
        self.robot.emergency_stop.assert_called_once()
        self.robot.set_ready.assert_not_called()

    def test_larger_joint_check_does_not_access_grippers(self):
        result, move = self.run_demo('--check', '--test', 'joints',
                                     '--amplitude', '0.14', '--move-time', '6')
        self.assertEqual(result, 0)
        move.assert_not_called()
        for side in ('left', 'right'):
            getattr(self.robot, 'get_' + side + '_eeff_config').assert_not_called()
            getattr(self.robot, 'set_' + side + '_gripper').assert_not_called()

    def test_larger_offset_requires_bounded_speed_and_range(self):
        for amplitude, seconds in (('0.20', '4'), ('0.21', '6')):
            with patch.object(sys, 'argv', ['demo', '--amplitude', amplitude, '--move-time', seconds]), \
                 contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.demo.parse_args()

    def test_faster_larger_check_sends_no_control_commands(self):
        result, move = self.run_demo('--check', '--test', 'joints', '--arm', 'left',
                                     '--joints', '1', '--amplitude', '0.20',
                                     '--move-time', '5', '--pause', '0')
        self.assertEqual(result, 0)
        move.assert_not_called()
        self.robot.emergency_stop.assert_not_called()
        self.robot.set_ready.assert_not_called()
        for side in ('left', 'right'):
            getattr(self.robot, 'set_' + side + '_gripper').assert_not_called()

    def test_measured_overspeed_is_still_rejected(self):
        self.states['left']['reality']['vel'][0] = 0.1001
        with self.assertRaisesRegex(RuntimeError, 'exceeds 0.1 rad/s'):
            self.demo.Feedback(self.robot).read()

    def test_invalid_speed_blocks_before_motion_or_stop(self):
        for speed in (0, -0.1, None, True, float('nan'), 1.1):
            with self.subTest(speed=speed), patch.dict(self.states['left'], vel_per=speed):
                result, move = self.run_demo('--check', '--arm', 'left', '--joints', '1')
                self.assertEqual(result, 1)
                move.assert_not_called()
                self.robot.emergency_stop.assert_not_called()
                self.robot.set_ready.assert_not_called()

    def test_zero_speed_is_rechecked_before_each_command(self):
        feedback = MagicMock()
        feedback.read.return_value = self.states
        self.states['left']['vel_per'] = 0
        with self.assertRaisesRegex(RuntimeError, 'vel_per=0'):
            self.demo.move_and_wait(self.robot, feedback,
                dict(left=[0.0] * 7, right=[0.0] * 7),
                dict(side='left', joint=1, offset=0.035), [0.035] + [0.0] * 6, 4)
        self.robot.move_left_joint.assert_not_called()

    def test_sdk_rejection_is_preserved_without_recovery_or_retry(self):
        reply = {'recv': 'Task_Reject', 'error': 42, 'errMsg': 'controller stopped'}
        impl = self.robot._left._impl
        original = impl.request
        original.return_value = reply
        trace = self.demo.RequestTrace(self.robot)
        payload = {'command': 'webRecieveTasks', 'task_id': 'TASK_PVT'}
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(impl.request(payload), reply)
        self.assertEqual(trace.entries[0]['request'], payload)
        self.assertEqual(trace.entries[0]['response'], reply)
        self.assertEqual(trace.entries[0]['controller_state'], self.raw)
        original.assert_called_once()
        self.robot.set_ready.assert_not_called()
        impl.recover.assert_not_called()
        trace.close()
        self.assertIs(impl.request, original)

    def test_gripper_plan_rejects_limits_force_and_disconnection(self):
        configs = {side: getattr(self.robot, 'get_' + side + '_eeff_config')()
                   for side in ('left', 'right')}
        moves = self.demo.plan_grippers(self.states, configs, ('left', 'right'), 0.03, 0.005, 5)
        self.assertEqual(len(moves), 4)
        configs['left']['eeff_tau'] = [2.0]
        with self.assertRaisesRegex(RuntimeError, 'force'):
            self.demo.plan_grippers(self.states, configs, ('left',), 0.03, 0.005, 5)
        with self.assertRaisesRegex(RuntimeError, 'outside limits'):
            self.demo.plan_grippers(self.states, configs, ('left',), 0.1, 0.005, 1)
        self.states['left']['eeff']['is_connect'] = False
        with self.assertRaisesRegex(RuntimeError, 'connected'):
            self.demo.plan_grippers(self.states, configs, ('left',), 0.03, 0.005, 1)

    def test_gripper_command_and_actual_feedback(self):
        for moving in (False, True):
            with self.subTest(moving=moving):
                self.setUp()
                feedback = MagicMock()
                feedback.read.return_value = self.states
                def grip(target, **kwargs):
                    if moving:
                        self.states['left']['eeff']['eeff_pos'] = [target]
                    return 1
                self.robot.set_left_gripper.side_effect = grip
                clock = [0.0]
                def sleep(seconds):
                    clock[0] += seconds
                move = dict(side='left', target=0.03, force=5)
                home = dict(left=[0.0] * 7, right=[0.0] * 7)
                with patch.object(self.demo.time, 'monotonic', side_effect=lambda: clock[0]), \
                     patch.object(self.demo.time, 'sleep', side_effect=sleep):
                    if moving:
                        result = self.demo.grip_and_wait(self.robot, feedback, home, move)
                        self.assertAlmostEqual(result['observed'], 0.03)
                        self.robot.set_left_gripper.assert_called_once_with(0.03, tau=5)
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'after timeout'):
                            self.demo.grip_and_wait(self.robot, feedback, home, move)
                        self.robot.set_left_gripper.assert_called_once()
                self.robot.move_left_joint.assert_not_called()
                self.robot.move_right_joint.assert_not_called()
                self.robot.set_right_gripper.assert_not_called()


if __name__ == '__main__':
    unittest.main()
