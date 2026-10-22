"""Check verification routing and failure reports without hardware or compilers."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('verify_robot', ROOT / 'scripts/verify_robot.py')
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


class VerificationTest(unittest.TestCase):
    def test_feature_routing_does_not_run_other_tests(self):
        for feature in ('camera', 'audio', 'led'):
            args = runner.parse_args(['g1', feature, '--interface', 'eth7'])
            plan = runner.commands(args, Path('/tmp/output'))
            command = plan[-1]
            self.assertEqual(command[command.index('--test') + 1], feature)
            self.assertIn('eth7', command)
            self.assertIn('unitree_g1_sdk_check', plan[1])
            self.assertNotIn('adapter_bridge', ' '.join(command))

    def test_motion_keeps_confirmation(self):
        for feature in ('joints', 'gripper', 'joint-gripper'):
            args = runner.parse_args(['a3', feature])
            command = runner.commands(args, Path('/tmp/output'))[-1]
            self.assertNotIn('--yes', command)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            runner.parse_args(['d3', 'joints'])

    def test_dry_run_never_builds_connects_or_writes(self):
        with patch.object(runner, 'execute') as run, \
             patch.object(runner.Path, 'mkdir') as mkdir, \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main(['g1', 'camera', '--dry-run']), 0)
        run.assert_not_called()
        mkdir.assert_not_called()

    def test_build_failure_stops_before_hardware(self):
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(runner, 'ROOT', Path(temp)), \
             patch.object(runner, 'execute', return_value=7) as run, \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main(['g1', 'camera']), 7)
            self.assertEqual(run.call_count, 1)
            report = json.loads(next(Path(temp).rglob('result.json')).read_text())
            self.assertEqual(report['result'], 'FAIL')
            self.assertEqual(report['exit_code'], 7)

    def test_offline_carm_needs_no_cpp_sdk(self):
        args = runner.parse_args(['d3', 'offline'])
        plan = runner.commands(args, Path('/tmp/output'))
        self.assertEqual(len(plan), 1)
        self.assertIn('tests/demo/cvte_carm_examples_test.py', plan[0])

    def test_interrupt_waits_for_child_cleanup(self):
        with patch.object(runner.subprocess, 'Popen') as popen:
            child = popen.return_value
            child.wait.side_effect = [KeyboardInterrupt, 130]
            with self.assertRaises(KeyboardInterrupt):
                runner.execute(['demo'])
            self.assertEqual(child.wait.call_count, 2)
            child.kill.assert_not_called()


if __name__ == '__main__':
    unittest.main()
