"""Fixed-Ladder control: one transport worker, latest targets, no catch-up queue."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence

import yaml

from .driver import PymcProtocolSLMPDriver, SLMPDriver
from .exceptions import ConfigurationError

LOGGER = logging.getLogger(__name__)
STOP = (660, 660, 660, 660)
DEVICE_MAP = dict(servo_on="M1000", trigger="M1010", brake_release="M1020",
                  speed="D1000", hall="D1010", angle="D1020",
                  actual_angle="D1030", bldc_enable="Y8")


@dataclass(frozen=True)
class CyclicConfig:
    ip: str
    port: int
    plc_type: str
    socket_timeout_s: float
    rate_hz: float
    on_s: float
    off_s: float
    max_scan_s: Optional[float]
    command_timeout_s: float
    servo_wait_s: float
    startup_timeout_s: float
    initial_angles: tuple
    units_per_degree: float
    raw_zero: int
    tolerance_deg: float
    stop_error_deg: float
    enable_mode: str
    feedback_interval_s: float
    feedback_max_age_s: float
    hall_mode: str
    diameter_m: float
    pulses_per_revolution: int
    hall_window_s: float
    hall_max_speed_m_s: float

    def encode(self, angles: Sequence[float]) -> tuple:
        if len(angles) != 4 or any(isinstance(a, bool) or not math.isfinite(a)
                                   or not 0 <= a < 360 for a in angles):
            raise ValueError("angles must contain four finite degrees in [0, 360)")
        turn = round(360 * self.units_per_degree)
        if turn <= 0:
            raise ValueError("steering resolution must represent a full turn")
        raw = tuple(round(a * self.units_per_degree) % turn + self.raw_zero for a in angles)
        if any(not -32768 <= a <= 32767 for a in raw):
            raise ValueError("encoded angle exceeds a signed 16-bit D word")
        return raw

    def decode(self, raw: Sequence[int]) -> tuple:
        if len(raw) != 4:
            raise ValueError("actual_angle response must contain four words")
        return tuple(((a - self.raw_zero) / self.units_per_degree) % 360 for a in raw)


def load_cyclic_config(path: Path) -> CyclicConfig:
    """Reject unknown keys, unsupported devices, and impossible timing budgets."""
    try:
        data = yaml.safe_load(path.read_text())
        fields = {
            "plc": {"ip", "port", "plc_type", "socket_timeout_s"},
            "timing": {"write_rate_hz", "trigger_on_s", "trigger_off_s", "plc_max_scan_s",
                       "scan_margin", "command_timeout_s"},
            "startup": {"servo_ready_wait_s", "angle_timeout_s", "initial_angle_deg"},
            "steering": {"units_per_degree", "raw_zero", "tolerance_deg", "stop_error_deg"},
            "bldc": {"enable_mode"}, "feedback": {"poll_interval_s", "max_age_s"},
            "hall": {"mode", "wheel_diameter_m", "pulses_per_revolution", "window_s",
                     "max_speed_m_s"},
        }
        if not isinstance(data, dict) or set(data) != set(fields) | {"register_mapping_file"}:
            raise ValueError("invalid fixed-Ladder configuration sections")
        for name, keys in fields.items():
            if not isinstance(data[name], dict) or set(data[name]) != keys:
                raise ValueError(f"{name}: missing or unknown keys")
        mapping = yaml.safe_load((path.parent / data["register_mapping_file"]).read_text())
        if mapping != DEVICE_MAP:
            raise ValueError("register map must match the supplied fixed-Ladder interface")

        def number(section, key):
            value = data[section][key]
            if isinstance(value, bool) or not isinstance(value, (float, int)) \
                    or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{section}.{key} must be finite and positive")
            return float(value)

        def integer(section, key, low, high):
            value = data[section][key]
            if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
                raise ValueError(f"{section}.{key} must be an integer in {low}..{high}")
            return value

        scan = data["timing"]["plc_max_scan_s"]
        if scan is not None:
            scan = number("timing", "plc_max_scan_s")
        margin = number("timing", "scan_margin")
        if margin <= 1:
            raise ValueError("scan_margin must exceed 1")
        cfg = CyclicConfig(
            ip=data["plc"]["ip"], port=integer("plc", "port", 1, 65535),
            plc_type=data["plc"]["plc_type"], socket_timeout_s=number("plc", "socket_timeout_s"),
            rate_hz=number("timing", "write_rate_hz"), on_s=number("timing", "trigger_on_s"),
            off_s=number("timing", "trigger_off_s"), max_scan_s=scan,
            command_timeout_s=number("timing", "command_timeout_s"),
            servo_wait_s=number("startup", "servo_ready_wait_s"),
            startup_timeout_s=number("startup", "angle_timeout_s"),
            initial_angles=tuple(data["startup"]["initial_angle_deg"]),
            units_per_degree=number("steering", "units_per_degree"),
            raw_zero=integer("steering", "raw_zero", -32768, 32767),
            tolerance_deg=number("steering", "tolerance_deg"),
            stop_error_deg=number("steering", "stop_error_deg"),
            enable_mode=data["bldc"]["enable_mode"],
            feedback_interval_s=number("feedback", "poll_interval_s"),
            feedback_max_age_s=number("feedback", "max_age_s"),
            hall_mode=data["hall"]["mode"], diameter_m=number("hall", "wheel_diameter_m"),
            pulses_per_revolution=integer("hall", "pulses_per_revolution", 1, 65535),
            hall_window_s=number("hall", "window_s"),
            hall_max_speed_m_s=number("hall", "max_speed_m_s"),
        )
        if not isinstance(cfg.ip, str) or not cfg.ip.strip():
            raise ValueError("plc.ip must be nonempty")
        if cfg.plc_type not in ("Q", "L", "QnA"):
            raise ValueError("unsupported normal 3E PLC type")
        if cfg.enable_mode not in ("external", "slmp"):
            raise ValueError("bldc.enable_mode must be external or slmp")
        if cfg.hall_mode not in ("unknown", "cumulative_u16"):
            raise ValueError("hall.mode must be unknown or cumulative_u16")
        if cfg.on_s + cfg.off_s >= 1 / cfg.rate_hz:
            raise ValueError("trigger ON + OFF leaves no SLMP budget in the requested period")
        if scan is not None and min(cfg.on_s, cfg.off_s) < scan * margin:
            raise ValueError("trigger ON and OFF must cover measured scan time with margin")
        if cfg.feedback_max_age_s <= cfg.feedback_interval_s:
            raise ValueError("feedback.max_age_s must exceed poll_interval_s")
        if not cfg.tolerance_deg < cfg.stop_error_deg <= 90:
            raise ValueError("require tolerance_deg < stop_error_deg <= 90")
        cfg.encode(cfg.initial_angles)
        if cfg.raw_zero + round(360 * cfg.units_per_degree) - 1 > 32767:
            raise ValueError("steering encoding exceeds a signed 16-bit D word")
        return cfg
    except (OSError, yaml.YAMLError, ValueError, TypeError, KeyError) as exc:
        raise ConfigurationError(f"Invalid cyclic profile {path}: {exc}") from exc


class HallEstimator:
    """Unsigned cumulative 16-bit counters; speed magnitude, no inferred direction."""

    def __init__(self, config: CyclicConfig):
        self.config = config
        self.samples = deque()
        self.totals = [0] * 4
        self.last = None

    def update(self, words: Sequence[int], now: float):
        if len(words) != 4:
            raise ValueError("Hall response must contain four words")
        counts = tuple(w & 0xffff for w in words)
        if self.config.hall_mode == "unknown":
            return None
        distance = math.pi * self.config.diameter_m / self.config.pulses_per_revolution
        if self.last is not None:
            previous_time, previous = self.last
            dt = now - previous_time
            delta = tuple((n - p) & 0xffff for n, p in zip(counts, previous))
            if dt <= 0 or any(d * distance > self.config.hall_max_speed_m_s * dt + distance
                              for d in delta):
                self.samples.clear()
                self.totals = [0] * 4
            else:
                self.totals = [t + d for t, d in zip(self.totals, delta)]
        self.last = now, counts
        self.samples.append((now, tuple(self.totals)))
        while len(self.samples) > 2 and self.samples[1][0] <= now - self.config.hall_window_s:
            self.samples.popleft()
        start, old = self.samples[0]
        elapsed = now - start
        if elapsed < self.config.hall_window_s:
            return None
        return tuple((t - p) * distance / elapsed for t, p in zip(self.totals, old))


@dataclass(frozen=True)
class Target:
    angles: tuple
    speeds: tuple
    received_s: float
    steer: bool = True
    alignment_angles: Optional[tuple] = None


class FixedLadderSession:
    """Synchronous transactions, used only by the transport worker."""

    def __init__(self, config: CyclicConfig, driver: SLMPDriver,
                 clock: Callable = time.monotonic, sleep: Callable = time.sleep,
                 cancel: Optional[threading.Event] = None):
        self.config, self.driver = config, driver
        self.clock, self.sleep = clock, sleep
        self.last_off_s = float("-inf")
        self.feedback = None
        self.hall = HallEstimator(config)
        self.cancel = cancel

    def startup_wait(self, seconds):
        if self.cancel is None:
            self.sleep(seconds)
        elif self.cancel.wait(seconds):
            raise RuntimeError("startup cancelled")

    def pulse(self):
        self.sleep(max(0.0, self.last_off_s + self.config.off_s - self.clock()))
        try:
            self.driver.write_bits("M1010", [1] * 4)
            self.sleep(self.config.on_s)
        finally:
            # ON may have been applied even if its acknowledgement was lost.
            self.driver.write_bits("M1010", [0] * 4)
            self.last_off_s = self.clock()

    def read_feedback(self):
        raw_angles = self.driver.read_words("D1030", 4)
        angle_time = self.clock()
        angles = self.config.decode(raw_angles)
        words = self.driver.read_words("D1010", 4)
        now = self.clock()
        speeds = self.hall.update(words, now)
        self.feedback = dict(actual_angle_deg=angles, angle_received_s=angle_time,
                             hall_raw=tuple(w & 0xffff for w in words),
                             hall_speed_m_s=speeds, received_s=now)
        return self.feedback

    def startup(self):
        self.startup_wait(0)
        self.driver.write_words("D1000", STOP)
        if self.config.enable_mode == "slmp":
            self.driver.write_bits("Y8", [0])
        self.driver.write_bits("M1010", [0] * 4)
        self.last_off_s = self.clock()
        self.driver.write_bits("M1000", [1] * 4)
        self.startup_wait(self.config.servo_wait_s)
        self.driver.write_bits("M1020", [1])
        self.driver.write_words("D1020", self.config.encode(self.config.initial_angles))
        self.pulse()
        deadline = self.clock() + self.config.startup_timeout_s
        while True:
            self.startup_wait(0)
            feedback = self.read_feedback()
            if max(self.angle_errors(self.config.initial_angles, feedback)) <= self.config.tolerance_deg:
                break
            if self.clock() >= deadline:
                raise TimeoutError("steering did not reach startup angles")
            self.startup_wait(self.config.feedback_interval_s)
        if self.config.enable_mode == "slmp":
            self.driver.write_bits("Y8", [1])

    @staticmethod
    def angle_errors(angles, feedback):
        return tuple(abs(math.remainder(target - actual, 360))
                     for target, actual in zip(angles, feedback["actual_angle_deg"]))

    def cycle(self, target: Target):
        cfg, now = self.config, self.clock()
        feedback = self.feedback
        expired = now - target.received_s > cfg.command_timeout_s
        stale = feedback is None or now - feedback["angle_received_s"] > cfg.feedback_max_age_s
        speeds = STOP if expired or stale else target.speeds
        if feedback is not None:
            errors = self.angle_errors(target.alignment_angles or target.angles, feedback)
            speeds = tuple(
                660 if error >= cfg.stop_error_deg else
                round(660 - (660 - raw) * max(0.0, math.cos(math.radians(error))))
                for raw, error in zip(speeds, errors)
            )
        self.driver.write_words("D1000", speeds)
        if not expired and target.steer:
            self.driver.write_words("D1020", cfg.encode(target.angles))
            self.pulse()
        if feedback is None or self.clock() - feedback["received_s"] >= cfg.feedback_interval_s:
            self.read_feedback()
        return dict(applied_raw=speeds, command_expired=expired, feedback_stale=stale,
                    feedback=self.feedback)

    def shutdown(self):
        errors = []
        operations = [lambda: self.driver.write_words("D1000", STOP),
                      lambda: self.driver.write_bits("M1010", [0] * 4)]
        if self.config.enable_mode == "slmp":
            operations.append(lambda: self.driver.write_bits("Y8", [0]))
        operations += [lambda: self.driver.write_bits("M1020", [0]),
                       lambda: self.driver.write_bits("M1000", [0] * 4)]
        for operation in operations:
            try:
                operation()
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError("Incomplete software stop: " + "; ".join(errors))


class CyclicWorker:
    """Latest-value mailbox and monotonic cadence with no overlapping cycles."""

    def __init__(self, config: CyclicConfig, driver: Optional[SLMPDriver] = None):
        self.config = config
        self.driver = driver or PymcProtocolSLMPDriver(
            config.ip, config.port, plc_type=config.plc_type,
            socket_timeout_s=config.socket_timeout_s)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._target = Target(config.initial_angles, STOP, float("-inf"), False)
        self._error = None
        self._status = {}

    @property
    def status(self):
        with self._lock:
            result = dict(self._status, error=self._error)
        feedback = result.get("feedback")
        if feedback is not None:
            result["feedback_age_s"] = time.monotonic() - feedback["angle_received_s"]
        return result

    def submit(self, angles, speeds, alignment_angles=None):
        self.config.encode(angles)
        if alignment_angles is not None:
            self.config.encode(alignment_angles)
        speeds = tuple(speeds)
        if len(speeds) != 4 or any(isinstance(v, bool) or not isinstance(v, int)
                                   or not 0 <= v <= 660 for v in speeds):
            raise ValueError("speeds must contain four integers in 0..660")
        with self._lock:
            if self._error is not None or self._stop.is_set():
                raise RuntimeError(self._error or "SLMP worker stopped")
            self._target = Target(tuple(angles), speeds, time.monotonic(), True,
                                  None if alignment_angles is None else tuple(alignment_angles))

    def stop_drive(self):
        with self._lock:
            self._target = Target(self._target.angles, STOP, time.monotonic(), False)

    def emergency_stop(self):
        self._stop.set()

    def start(self):
        if self._thread is not None:
            raise RuntimeError("SLMP worker may only be started once")
        self._thread = threading.Thread(target=self._run, name="fx5u-slmp", daemon=True)
        self._thread.start()
        timeout = self.config.startup_timeout_s + self.config.servo_wait_s + 20 * self.config.socket_timeout_s
        if not self._ready.wait(timeout):
            self._stop.set()
            raise TimeoutError("SLMP startup did not finish")
        if self.status["error"]:
            raise RuntimeError(self.status["error"])

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=10 + 10 * self.config.socket_timeout_s)
            if self._thread.is_alive():
                raise RuntimeError("SLMP transport worker did not stop")
        if self.status["error"]:
            raise RuntimeError(self.status["error"])

    def _run(self):
        session = FixedLadderSession(self.config, self.driver, cancel=self._stop)
        overruns = 0
        previous_started = None
        try:
            self.driver.connect()
            session.startup()
            self._ready.set()
            while not self._stop.is_set():
                started = time.monotonic()
                with self._lock:
                    target = self._target
                status = session.cycle(target)
                elapsed = time.monotonic() - started
                overruns += int(elapsed > 1 / self.config.rate_hz)
                with self._lock:
                    self._status = dict(status, cycle_s=elapsed, overruns=overruns,
                                        period_s=None if previous_started is None else started - previous_started,
                                        target_rate_hz=self.config.rate_hz,
                                        trigger_timing_verified=self.config.max_scan_s is not None,
                                        enable_mode=self.config.enable_mode)
                previous_started = started
                self._stop.wait(max(0.0, 1 / self.config.rate_hz - elapsed))
        except BaseException as exc:
            LOGGER.exception("SLMP worker failed")
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            self._stop.set()
        finally:
            for action in (session.shutdown, self.driver.disconnect):
                try:
                    action()
                except Exception as exc:
                    LOGGER.error("SLMP shutdown failed: %s", exc)
                    with self._lock:
                        self._error = self._error or f"{type(exc).__name__}: {exc}"
            self._ready.set()
