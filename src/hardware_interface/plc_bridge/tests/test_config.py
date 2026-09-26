from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from fx5u_controller import ConfigurationError, load_application_config, load_register_mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class ConfigurationTests(unittest.TestCase):
    def test_default_config_preserves_endpoint_and_confirmed_map(self) -> None:
        config = load_application_config(PROJECT_ROOT / "config" / "default.yaml")

        self.assertEqual(config.plc.ip, "192.168.3.250")
        self.assertEqual(config.plc.port, 1025)
        self.assertFalse(config.feedback.enabled)
        self.assertFalse(config.heartbeat.enabled)
        self.assertIsNone(config.heartbeat.device)
        self.assertEqual(config.wheels[1].trigger, "M100")
        self.assertEqual(config.wheels[4].speed, "D1023")
        self.assertEqual(
            [config.wheels[wheel].direction for wheel in (1, 2, 3, 4)],
            ["M60", "M65", "M62", "M63"],
        )
        self.assertEqual(config.speed_control.enable_device, "M58")
        self.assertEqual(config.speed_control.stop_value, 660)
        self.assertEqual(config.speed_control.minimum_value, 100)
        self.assertEqual(config.speed_control.maximum_value, 660)

    def test_nonconsecutive_command_mapping_is_rejected(self) -> None:
        source = (PROJECT_ROOT / "config" / "registers_mapping.yaml").read_text()
        source = source.replace("trigger: M103", "trigger: M105")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mapping.yaml"
            path.write_text(source)
            with self.assertRaisesRegex(ConfigurationError, "consecutive"):
                load_register_mapping(path)

    def test_nonconsecutive_direction_mapping_is_accepted_but_duplicates_are_rejected(self) -> None:
        source = (PROJECT_ROOT / "config" / "registers_mapping.yaml").read_text()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mapping.yaml"
            path.write_text(source)
            mapping = load_register_mapping(path)
            self.assertEqual(
                [mapping[wheel].direction for wheel in (1, 2, 3, 4)],
                ["M60", "M65", "M62", "M63"],
            )

            path.write_text(source.replace("direction: M65", "direction: M60"))
            with self.assertRaisesRegex(ConfigurationError, "assigned more than once"):
                load_register_mapping(path)

    def test_feedback_cannot_be_enabled_without_confirmed_devices(self) -> None:
        self._assert_invalid_default_replacement(
            "feedback:\n  enabled: false",
            "feedback:\n  enabled: true",
            "no confirmed feedback devices",
        )

    def test_heartbeat_cannot_be_enabled_without_confirmed_device(self) -> None:
        self._assert_invalid_default_replacement(
            "heartbeat:\n  # Do not enable until a reviewed PLC watchdog rung uses the same M device and\n"
            "  # safely stops the robot after plc_watchdog_timeout_s without transitions.\n"
            "  enabled: false",
            "heartbeat:\n  # Do not enable until a reviewed PLC watchdog rung uses the same M device and\n"
            "  # safely stops the robot after plc_watchdog_timeout_s without transitions.\n"
            "  enabled: true",
            "no PLC-approved heartbeat.device",
        )

    def test_heartbeat_cannot_overlap_a_command_device(self) -> None:
        self._assert_invalid_default_replacement(
            "device: null\n  interval_s: 0.2",
            "device: M100\n  interval_s: 0.2",
            "overlaps another configured device",
        )

    def _assert_invalid_default_replacement(
        self, old: str, new: str, expected_message: str
    ) -> None:
        source = (PROJECT_ROOT / "config" / "default.yaml").read_text()
        self.assertIn(old, source)
        source = source.replace(old, new, 1)
        with tempfile.TemporaryDirectory() as directory:
            temp_dir = Path(directory)
            (temp_dir / "default.yaml").write_text(source)
            (temp_dir / "registers_mapping.yaml").write_text(
                (PROJECT_ROOT / "config" / "registers_mapping.yaml").read_text()
            )
            with self.assertRaisesRegex(ConfigurationError, expected_message):
                load_application_config(temp_dir / "default.yaml")


if __name__ == "__main__":
    unittest.main()
