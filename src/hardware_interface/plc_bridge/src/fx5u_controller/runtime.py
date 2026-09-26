"""Optional feedback polling and PLC watchdog heartbeat services."""

from __future__ import annotations

import logging
import threading
from typing import List, Optional

from .controller import MitsubishiWheelController
from .exceptions import ControllerStateError, SLMPCommunicationError
from .models import FeedbackConfig, HeartbeatConfig, TelemetrySnapshot


LOGGER = logging.getLogger("fx5u_controller.runtime")


class RuntimeSupervisor:
    """Own the optional background threads for one controller connection.

    All actual I/O still goes through the controller's transaction lock. A
    background communication error stops both services and the controller
    latches that failure, preventing subsequent commands until reconnection.
    """

    def __init__(
        self,
        controller: MitsubishiWheelController,
        feedback: FeedbackConfig,
        heartbeat: HeartbeatConfig,
    ) -> None:
        self.controller = controller
        self.feedback_config = feedback
        self.heartbeat_config = heartbeat
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._threads: List[threading.Thread] = []
        self._latest_feedback: Optional[TelemetrySnapshot] = None
        self._last_error: Optional[BaseException] = None

    @property
    def latest_feedback(self) -> Optional[TelemetrySnapshot]:
        with self._state_lock:
            return self._latest_feedback

    @property
    def last_error(self) -> Optional[BaseException]:
        with self._state_lock:
            return self._last_error

    @property
    def running(self) -> bool:
        with self._state_lock:
            return any(thread.is_alive() for thread in self._threads)

    def start(self) -> None:
        if not self.controller.connected:
            raise ControllerStateError("connect the controller before starting runtime services")
        with self._state_lock:
            if any(thread.is_alive() for thread in self._threads):
                raise ControllerStateError("runtime services are already running")
            self._threads = []
            self._latest_feedback = None
            self._last_error = None
            self._stop_event.clear()
            if self.feedback_config.enabled:
                self._threads.append(
                    threading.Thread(
                        target=self._feedback_loop,
                        name="fx5u-feedback",
                        daemon=True,
                    )
                )
            if self.heartbeat_config.enabled:
                self._threads.append(
                    threading.Thread(
                        target=self._heartbeat_loop,
                        name="fx5u-heartbeat",
                        daemon=True,
                    )
                )
            threads = list(self._threads)

        for thread in threads:
            thread.start()
        LOGGER.info(
            "runtime services started feedback=%s heartbeat=%s",
            self.feedback_config.enabled,
            self.heartbeat_config.enabled,
        )
        if self.heartbeat_config.enabled:
            LOGGER.warning(
                "heartbeat only provides stopping protection when matching PLC watchdog "
                "logic is installed and tested; plc_timeout_s=%s",
                self.heartbeat_config.plc_watchdog_timeout_s,
            )

    def stop(self) -> None:
        """Stop threads and clear the configured heartbeat bit."""

        self._stop_event.set()
        with self._state_lock:
            threads = list(self._threads)
        join_timeout = max(
            self.feedback_config.poll_interval_s,
            self.heartbeat_config.interval_s,
            0.1,
        ) + 1.0
        errors = []
        for thread in threads:
            thread.join(timeout=join_timeout)
            if thread.is_alive():
                errors.append(f"thread {thread.name} did not stop")

        device = self.heartbeat_config.device
        if self.heartbeat_config.enabled and device is not None:
            try:
                self.controller.clear_heartbeat(device)
            except Exception as exc:
                errors.append(f"heartbeat OFF cleanup failed: {exc}")

        with self._state_lock:
            self._threads = []
        LOGGER.info("runtime services stopped errors=%s", len(errors))
        if errors:
            raise SLMPCommunicationError("; ".join(errors))

    def _feedback_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                snapshot = self.controller.read_feedback(self.feedback_config)
            except BaseException as exc:
                self._record_failure("feedback polling", exc)
                return
            with self._state_lock:
                self._latest_feedback = snapshot
            if self._stop_event.wait(self.feedback_config.poll_interval_s):
                return

    def _heartbeat_loop(self) -> None:
        device = self.heartbeat_config.device
        if device is None:
            self._record_failure(
                "heartbeat",
                ControllerStateError("heartbeat device is not configured"),
            )
            return
        state = False
        while not self._stop_event.is_set():
            state = not state
            try:
                self.controller.update_heartbeat(device, state)
            except BaseException as exc:
                self._record_failure("heartbeat", exc)
                return
            if self._stop_event.wait(self.heartbeat_config.interval_s):
                return

    def _record_failure(self, service: str, exc: BaseException) -> None:
        with self._state_lock:
            if self._last_error is None:
                self._last_error = exc
        self._stop_event.set()
        LOGGER.critical("runtime service failed service=%s error=%s", service, exc)
