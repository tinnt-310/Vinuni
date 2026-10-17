"""The vehicle must not spawn into a world with a failed station insertion."""

import importlib.util
from pathlib import Path
import unittest

from launch.actions import Shutdown


spec = importlib.util.spec_from_file_location(
    'vehicle_simulation', Path(__file__).parents[1] / 'launch/simulation.launch.py')
simulation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(simulation)


class ProcessExited:
    def __init__(self, returncode):
        self.returncode = returncode


class StationSequenceTest(unittest.TestCase):
    def test_success_starts_exactly_the_next_spawn(self):
        next_spawn = object()
        transition = getattr(simulation, 'station_spawn_transition', None)
        self.assertIsNotNone(transition, 'station transitions must be explicit')
        self.assertEqual(transition(ProcessExited(0), next_spawn), [next_spawn])

    def test_failed_station_stops_launch_before_robot_spawn(self):
        next_spawn = object()
        transition = getattr(simulation, 'station_spawn_transition', None)
        self.assertIsNotNone(transition, 'station transitions must be explicit')
        actions = transition(ProcessExited(1), next_spawn)
        self.assertEqual(len(actions), 1)
        self.assertIsInstance(actions[0], Shutdown)
        self.assertNotIn(next_spawn, actions)


if __name__ == '__main__':
    unittest.main()
