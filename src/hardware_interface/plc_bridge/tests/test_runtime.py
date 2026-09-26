from __future__ import annotations

import threading
import unittest
from pathlib import Path
from typing import List, Sequence, Tuple

from fx5u_controller import (
    FeedbackConfig,
    HeartbeatConfig,
    MitsubishiWheelController,
    RuntimeSupervisor,
    load_application_config,
    load_register_mapping,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WHEELS = load_register_mapping(PROJECT_ROOT / "config" / "registers_mapping.yaml")
SPEED_CONTROL = load_application_config(
    PROJECT_ROOT / "config" / "default.yaml"
).speed_control
Call = Tuple[str, str, List[int]]


class RuntimeDriver:
    def __init__(self) -> None:
        self.connected = False
        self.calls: List[Call] = []
        self.feedback_read = threading.Event()
        self.heartbeat_write = threading.Event()
        self.fail_reads = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def write_bits(self, head_device: str, values: Sequence[int]) -> None:
        self.calls.append(("write_bits", head_device, list(values)))
        if head_device == "M300":
            self.heartbeat_write.set()

    def write_words(self, head_device: str, values: Sequence[int]) -> None:
        self.calls.append(("write_words", head_device, list(values)))

    def read_bits(self, head_device: str, count: int) -> Sequence[int]:
        self.calls.append(("read_bits", head_device, [count]))
        self.feedback_read.set()
        if self.fail_reads:
            raise OSError("injected read failure")
        return [1, 1, 0, 0][:count]

    def read_words(self, head_device: str, count: int) -> Sequence[int]:
        self.calls.append(("read_words", head_device, [count]))
        return [10, 20, -10, -20][:count]


def make_controller(driver: RuntimeDriver) -> MitsubishiWheelController:
    return MitsubishiWheelController(
        "192.0.2.10",
        5007,
        0.01,
        wheels=WHEELS,
        speed_control=SPEED_CONTROL,
        driver=driver,
        sleep_fn=lambda _: None,
    )


class RuntimeTests(unittest.TestCase):
    def test_feedback_and_heartbeat_are_serialized_through_controller(self) -> None:
        driver = RuntimeDriver()
        controller = make_controller(driver)
        controller.connect()
        feedback = FeedbackConfig(
            enabled=True,
            poll_interval_s=0.01,
            servo_ready=("M200", "M201", "M202", "M203"),
            actual_angle=("D2000", "D2001", "D2002", "D2003"),
            actual_speed=None,
            alarm_code=None,
        )
        heartbeat = HeartbeatConfig(True, "M300", 0.01, 0.1)
        runtime = RuntimeSupervisor(controller, feedback, heartbeat)

        runtime.start()
        self.assertTrue(driver.feedback_read.wait(timeout=1.0))
        self.assertTrue(driver.heartbeat_write.wait(timeout=1.0))
        runtime.stop()

        snapshot = runtime.latest_feedback
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertEqual(snapshot.servo_ready, (True, True, False, False))
        self.assertEqual(snapshot.actual_angle, (10, 20, -10, -20))
        self.assertIn(("write_bits", "M300", [0]), driver.calls)
        self.assertIsNone(runtime.last_error)

    def test_feedback_failure_latches_controller_and_stops_runtime(self) -> None:
        driver = RuntimeDriver()
        driver.fail_reads = True
        controller = make_controller(driver)
        controller.connect()
        feedback = FeedbackConfig(
            enabled=True,
            poll_interval_s=0.01,
            servo_ready=("M200", "M201", "M202", "M203"),
            actual_angle=None,
            actual_speed=None,
            alarm_code=None,
        )
        runtime = RuntimeSupervisor(
            controller,
            feedback,
            HeartbeatConfig(False, None, 0.2, 0.5),
        )

        runtime.start()
        self.assertTrue(driver.feedback_read.wait(timeout=1.0))
        runtime.stop()

        self.assertTrue(controller.communication_fault)
        self.assertIsNotNone(runtime.last_error)


if __name__ == "__main__":
    unittest.main()
