"""The 20 Hz legacy publisher may outpace /clock timestamp updates."""

import importlib.util
from pathlib import Path
import unittest


script = Path(__file__).parents[1] / 'scripts/cmd_vel_normalizer.py'
spec = importlib.util.spec_from_file_location('cmd_vel_normalizer', script)
normalizer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(normalizer)


class NormalizerStampTest(unittest.TestCase):
    def test_suppresses_identical_twist_with_same_clock_stamp(self):
        self.assertFalse(normalizer.should_emit_command(
            100, (0.12, 0, 0), 100, (0.12, 0, 0)))

    def test_forwards_stop_even_with_same_clock_stamp(self):
        self.assertTrue(normalizer.should_emit_command(
            100, (0, 0, 0), 100, (0.12, 0, 0)))

    def test_forwards_new_clock_stamp(self):
        self.assertTrue(normalizer.should_emit_command(
            101, (0.12, 0, 0), 100, (0.12, 0, 0)))

    def test_forwards_backward_stamp_to_fail_closed_controller(self):
        self.assertTrue(normalizer.should_emit_command(
            99, (0.12, 0, 0), 100, (0.12, 0, 0)))

    def test_first_command_is_emitted(self):
        self.assertTrue(normalizer.should_emit_command(
            100, (0, 0, 0), None, None))


if __name__ == '__main__':
    unittest.main()
