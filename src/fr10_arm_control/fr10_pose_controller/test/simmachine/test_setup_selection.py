"""Ensure FakeSystem tests can target the just-built install overlay."""

import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from common import select_install_setup


class InstallSetupSelectionTest(unittest.TestCase):
    def test_explicit_overlay_is_used(self):
        with TemporaryDirectory() as directory:
            setup = Path(directory) / 'setup.bash'
            setup.touch()
            with patch.dict(os.environ, {'FR10_TEST_SETUP': str(setup)}):
                self.assertEqual(select_install_setup(), setup.resolve())

    def test_default_overlay_uses_workspace_root(self):
        expected = (Path(__file__).resolve().parents[5] / 'install/setup.bash').resolve()
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(select_install_setup(), expected)

    def test_missing_overlay_fails_closed(self):
        with patch.dict(os.environ,
                        {'FR10_TEST_SETUP': '/tmp/fr10_missing_setup.bash'}):
            with self.assertRaises(FileNotFoundError):
                select_install_setup()


if __name__ == '__main__':
    unittest.main()
