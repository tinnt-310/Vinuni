# FX5U four-wheel controller

Production-oriented Python controller for the confirmed Mitsubishi FX5U wheel
interface. It uses SLMP over TCP, 3E Binary frames, and
`pymcprotocol.Type3E`.

Quy trình cài đặt, test offline và bảng lệnh vận hành đầy đủ bằng tiếng Việt:
[`docs/PROTOCOL_HUONG_DAN_CHAY.md`](docs/PROTOCOL_HUONG_DAN_CHAY.md).

 ## Lệnh chạy chính thức

Sau khi đã cài đặt một lần và kết nối PLC:

```bash
cd /home/tin/plc
./run.sh
```

`run.sh` tự chọn Python trong `.venv`, đọc `config/default.yaml` và kết nối PLC
thật tại `192.168.3.250:1025`. Không cần chạy `source .venv/bin/activate`.

## Safety boundary

The software writes only the configured and validated command map:

| Function | Wheel 1 (right top) | Wheel 2 (right bottom) | Wheel 3 (left top) | Wheel 4 (left bottom) |
|---|---:|---:|---:|---:|
| FB Absolute trigger | M100 | M101 | M102 | M103 |
| Servo command | M110 | M111 | M112 | M113 |
| Fault reset | M120 | M121 | M122 | M123 |
| Steering angle (degree) | D1000 | D1001 | D1002 | D1003 |
| Raw speed command | D1020 | D1021 | D1022 | D1023 |

No angle or speed scaling is performed. Angles are whole degrees and are sent
unchanged. Speed values are raw signed 16-bit PLC words; their engineering unit
must be confirmed from the PLC/servo project.

`emergency_stop()` clears the confirmed trigger bits and commands the four
servos OFF. It is a best-effort software stop—not a replacement for a
hard-wired, safety-rated emergency-stop circuit.

## Project layout

```text
config/
  default.yaml
  registers_mapping.yaml
logs/
run.sh
scripts/
  example.py
  interactive_cli.py
src/fx5u_controller/
  config.py
  controller.py
  driver.py
  exceptions.py
  logging_setup.py
  models.py
  runtime.py
tests/
pyproject.toml
requirements.txt
setup.cfg
```

`pyproject.toml` selects the PEP 517 build backend; `setup.cfg` is the single
package/dependency definition and remains compatible with Ubuntu 22.04's
setuptools. `requirements.txt` only delegates to the local package for
environments that still use `pip install -r`.
Generated caches, runtime logs, build output, and package metadata are ignored.

## Install

```bash
cd /home/tin/plc
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip setuptools
.venv/bin/python -m pip install -e .
```

Các lệnh cài đặt trên chỉ chạy một lần. Những lần vận hành sau chỉ dùng
`cd /home/tin/plc` và `./run.sh`.

In GX Works3, configure the CPU Ethernet port as an SLMP server using
QnA-compatible 3E, Binary, TCP, the chosen port, and permit the required device
writes. The supplied endpoint in `config/default.yaml` is
`192.168.3.250:1025`; confirm the same values in GX Works3 before connecting.

## Configuration

- `config/default.yaml` contains the connection, timing, rotating-log,
  feedback-polling, and heartbeat settings.
- `config/registers_mapping.yaml` is the reviewed command-device map.
- Unknown keys, malformed devices, duplicate assignments, incorrect wheel
  numbering, and nonconsecutive command blocks are rejected at startup.
- CLI `--ip`, `--port`, and `--pulse` options can override deployment values
  without modifying source code.

The default trigger/reset pulse is 50 ms. This is a configurable software
assumption, not a confirmed Mitsubishi or PLC-program minimum. Do not shorten
it until the PLC task/scan behavior has been measured.

For a control update containing both steering and speed, use
`move_angles_and_speeds(angles, speeds)` or the interactive `motion` command.
It writes D1000..D1003 and D1020..D1023 under one lock, then pulses M100..M103
once. This removes the second 50 ms wait used by consecutive `move_angles()`
and `move_speeds()` calls. The minimum lock-held pulse time is therefore 50 ms,
plus SLMP round trips, rather than 100 ms. Before live driving, verify that the
PLC function block reads both command blocks on the same trigger event.

### Feedback

Feedback polling is implemented but disabled. Before setting
`feedback.enabled: true`, supply four confirmed devices in wheel order for at
least one of these groups:

- `servo_ready` — four M devices
- `actual_angle` — four D devices
- `actual_speed` — four D devices
- `alarm_code` — four D devices

Groups may be nonconsecutive; consecutive groups are automatically read in one
batch. A read failure latches the communication fault and stops background
services.

### Heartbeat/watchdog

Heartbeat transmission is implemented but disabled, and its device is `null`.
It must not be enabled until all of the following are complete:

1. A dedicated M device is assigned and reviewed.
2. PLC watchdog logic detects missing transitions within the configured timeout.
3. That PLC logic brings the robot to a verified safe stop.
4. Cable-disconnect and IPC-power-loss tests pass on isolated machinery.

The Python heartbeat alone cannot stop a robot after the IPC loses power; the
PLC watchdog rung is the safety action. Runtime heartbeat failures latch the
controller communication fault.

## Logging

`configure_logging()` creates a UTC-timestamped rotating black-box log. The
default location is `logs/fx5u-controller.log`, with ten 10 MiB backups.
Commands, values, connection state, shutdown operations, communication faults,
runtime service failures, and nonzero configured alarm feedback are recorded.

## Tests

All tests are offline and use fake SLMP drivers:

```bash
python3 -m unittest discover -s tests -v
```

They cover the original motion safety behavior plus configuration validation,
log rotation, telemetry polling, heartbeat cleanup, and background-failure
latching.

## Run

Keep the robot isolated/lifted and a hardware emergency stop within reach.
The configured endpoint is ready to use; `--ip` and `--port` remain available
for temporary overrides:

```bash
cd /home/tin/plc
./run.sh
```

In the interactive shell, send one combined update with:

```text
motion 30 30 -30 -30 100 100 100 100
```

`run.sh` enables real hardware access and the controller independently attempts
runtime shutdown, software emergency stop, and controlled disconnect on exit.

## Model-name check

Mitsubishi's published model list contains `FX5UJ-60MT/ES` and
`FX5U-64MT/ES`, rather than `FX5U-60MT/ES`. Verify the complete CPU nameplate.
The configured M/D interface does not depend on I/O-point count, but the exact
CPU model and Ethernet settings still require commissioning confirmation.
