"""World insertion must wait for an advancing Gazebo simulation clock."""

import importlib.util
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location(
    'wait_for_gazebo_clock',
    Path(__file__).parents[1] / 'scripts/wait_for_gazebo_clock.py')
readiness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(readiness)


class ClockReadinessTest(unittest.TestCase):
    def test_first_or_repeated_clock_stamp_is_not_ready(self):
        gate = readiness.ClockReadiness()
        self.assertFalse(gate.observe(0, 0))
        self.assertFalse(gate.observe(0, 0))

    def test_forward_progress_opens_gate(self):
        gate = readiness.ClockReadiness()
        self.assertFalse(gate.observe(1, 0))
        self.assertTrue(gate.observe(1, 1_000_000))

    def test_clock_rollback_resets_baseline(self):
        gate = readiness.ClockReadiness()
        self.assertFalse(gate.observe(3, 0))
        self.assertFalse(gate.observe(2, 0))
        self.assertTrue(gate.observe(2, 1))


if __name__ == '__main__':
    unittest.main()
