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

The low-level API accepts whole-degree PLC angles in 0..359, with 90 degrees
as the straight-travel steering position. The ROS bridge maps logical steering
through that offset and wraps into 0..359. Fractional degrees require a revised
PLC register protocol; the current mapping uses one 16-bit D word per wheel.
Direct wheel speed values are limited to raw 0..660: raw 660 is stop at
0.8 V and raw 0
corresponds to 3.8 V. Direction is controlled by separate M bits.

At startup the bridge commands raw 660, turns Y8/M58 OFF, pulses M222 ON then
OFF for 50 ms, and confirms the stopped command again. For a speed target it
preloads raw 660 before enabling M58 and Y8; stopping writes raw 660 before
turning Y8 and M58 OFF. These are SLMP acknowledgements, not drive feedback.

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

The open-loop ROS bridge subscribes to Twist targets on `/cmd_vel` from
keyboard teleop or Nav2. Its fixed PLC write loop ramps linear velocity
(0.40 m/s²), angular velocity (0.15 rad/s²), commanded steering angles
(30°/s), and raw speed (15 counts per write). These limits can be tuned in
`config/open_loop_bridge.yaml`. M100..M103 pulse only when an integer steering
angle changes. After the initial steering settle, angle and speed updates can
continue together without disabling Y8/M58. A drive direction bit is reversed
only after raw speed reaches 660; Y8/M58 stay enabled during that transition.
These are command-side limits, not measured wheel acceleration or angle.

Provisional wheel-speed calibration: `raw = clamp(660 - 260 * abs(v_wheel), 0, 660)`,
with wheel speed in m/s. `limits.raw_at_one_m_s: 400` sets the reference;
0.1 m/s maps to 634, 0.2 to 608, and 1.0 to 400 before ramping.
This is an assumed mapping, not measured velocity. The existing 0.20 m/s
chassis command limit is unchanged and is separate from this calibration.
Steering still sends integer-degree targets and 50 ms trigger pulses at 10 Hz.
Physical smoothness requires checking PLC positioning/retrigger behavior and
servo acceleration settings; host-side interpolation alone does not guarantee it.

In terminal teleop, press `r` once for a manual drive reset: command raw 660,
disable Y8/M58, pulse M222 ON/OFF, and remain stopped. Press a movement key
again to resume. Restart both bridge and keyboard after updating to enable
this reset topic. Reset is rejected while a communication fault is latched;
reconnect first. Normal speed/steering updates never pulse M222.

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

## ROS 2 open-loop keyboard and joystick control

The bridge writes commands but does not claim measured steering, measured
speed, joint states, or odometry. Joystick has priority only while its deadman
button is held. A source timeout of 0.75 seconds commands raw 660 and disables
M58 and Y8.

Keep all wheels lifted, keep a hardware emergency stop within reach, and verify
the wheel mapping, steering signs/offsets, drive directions, joystick axes, and
deadman button in `config/open_loop_bridge.yaml`.

Terminal 1 -- connect to the real PLC:

```bash
cd /home/tin/Vinuni/src/hardware_interface/plc_bridge
./run_bridge.sh --execute
```

Terminal 2 -- keyboard with live speed changes:

```bash
cd /home/tin/Vinuni/src/hardware_interface/plc_bridge
./run_keyboard.sh
```

Hold `I/U/O/J/L/M/,/.` to command motion; release the key to stop after
an 80 ms transition grace. Hold `q/z` to raise/lower linear and angular
speed, `w/x` for linear speed, or `e/c` for angular speed. Speed and motion
keys may be held together. The keyboard publishes targets at 50 Hz; the PLC
write loop defaults to 10 Hz because each steering trigger pulse lasts 50 ms.
`Space` or `K` commands an immediate stop. The terminal must support the
Kitty keyboard protocol's key-release and report-all-keys modes for true key-up.
By default the script detects support and falls back to standard terminal input.
Use `./run_keyboard.sh --input-mode terminal` to skip detection. This fallback
publishes at 50 Hz but infers release from a 0.75-second input timeout; it depends
on terminal repeat while a key is held. Speed keys preserve the active movement
and adjust by 1.1 per character. Space/K stop immediately. Use
`--input-mode kitty` to require true key-up, or `--key-timeout 0.75` to tune
the fallback timeout.

Alternatively, use ROS teleop_twist_keyboard:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.15
```

The movement layout matches `teleop_twist_keyboard`: `U I O / J K L / M , .`.
Hold Shift with a motion key for holonomic translation. A new key changes the
target immediately; the bridge ramps toward it and continues updating steering
and wheel speed. The terminal input does not depend on OS keyboard repeat.
Losing terminal focus commands zero, and the bridge's 0.75-second watchdog
stops the drive if Twist messages disappear. A constant held Twist eventually
produces a constant steering angle; the robot continues turning by wheel speed.

Terminal 2 -- joystick instead:

```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
```

The default deadman is joystick button index 4. Check the actual axes/buttons:

```bash
ros2 topic echo /joy
```

Inspect command acknowledgement (not physical feedback):

```bash
ros2 topic echo /plc_bridge/status
```

Normal timeout stops wheel drive but holds the last steering angle. Ctrl+C on
the bridge attempts the full software emergency stop and disconnect sequence.
If communication is already lost, only PLC watchdog logic or the hard-wired
emergency stop can guarantee stopping.

## Model-name check

Mitsubishi's published model list contains `FX5UJ-60MT/ES` and
`FX5U-64MT/ES`, rather than `FX5U-60MT/ES`. Verify the complete CPU nameplate.
The configured M/D interface does not depend on I/O-point count, but the exact
CPU model and Ethernet settings still require commissioning confirmation.
