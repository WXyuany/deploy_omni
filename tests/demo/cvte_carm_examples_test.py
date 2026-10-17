"""Offline SDK doubles: verify reads and interruption paths without hardware."""
import contextlib
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


if __name__ == '__main__':
    unittest.main()
