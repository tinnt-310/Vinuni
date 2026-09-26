import importlib.util
from pathlib import Path
import unittest
spec = importlib.util.spec_from_file_location('bringup', Path(__file__).parents[1] / 'launch/bringup.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class SelectionTest(unittest.TestCase):
    def test_supported_routes(self):
        for arm, vehicle, mode, filename in (
            (True, False, 'sim', 'simulation.launch.py'),
            (True, False, 'real', 'demo.launch.py'),
            (False, True, 'sim', 'simulation.launch.py'),
            (True, True, 'sim', 'simulation.launch.py')):
            with self.subTest(arm=arm, vehicle=vehicle, mode=mode):
                self.assertEqual(module.select_backend(arm, vehicle, mode)[1], filename)

    def test_unconfigured_plc_rejected(self):
        for arm in (False, True):
            with self.assertRaisesRegex(RuntimeError, 'missing encoder'):
                module.select_backend(arm, True, 'real')

    def test_empty_and_invalid_rejected(self):
        with self.assertRaises(ValueError):
            module.select_backend(False, False, 'sim')
        with self.assertRaises(ValueError):
            module.select_backend(True, True, 'typo')
