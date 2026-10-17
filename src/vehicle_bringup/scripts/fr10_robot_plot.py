#!/usr/bin/python3
"""One read-only window: force, torque, TCP X/Y/Z and wall-time TCP speed."""
import argparse
from collections import deque
import csv
from datetime import datetime
import math
from pathlib import Path
import time


class Stream:
    """Keep frame-consistent data; repeated source stamps do not imply freshness."""

    def __init__(self, dimensions, window):
        self.dimensions = dimensions
        self.window = window
        self.rows = deque(maxlen=math.ceil(window * 20) + 10)
        self.frame = ''
        self.stamp = None
        self.received = None
        self.values = None
        self.motion = deque(maxlen=2000)
        self.speed_rows = deque(maxlen=math.ceil(window * 20) + 10)

    def push(self, stamp_ns, frame, values, now):
        if len(values) != self.dimensions or not all(math.isfinite(v) for v in values):
            self.values = None
            self.motion.clear()
            return
        changed_frame = frame != self.frame
        reset_clock = self.stamp is not None and stamp_ns < self.stamp - 1_000_000_000
        if changed_frame or reset_clock:
            self.rows.clear()
            self.speed_rows.clear()
            self.motion.clear()
            self.stamp = None
        if self.stamp is not None and stamp_ns <= self.stamp:
            return
        if self.received is not None and now - self.received > 1.:
            self.motion.clear()
        self.frame, self.stamp = frame, stamp_ns
        self.values, self.received = tuple(values), now
        if self.dimensions == 3:
            self.motion.append((now, tuple(values)))
            while len(self.motion) > 2 and self.motion[1][0] < now - .5:
                self.motion.popleft()

    def path_speed(self, now):
        if self.values is None or self.received is None or now - self.received > 1. or len(self.motion) < 2:
            return float('nan')
        duration = self.motion[-1][0] - self.motion[0][0]
        if duration <= 0.:
            return float('nan')
        rows = list(self.motion)
        return sum(math.dist(a[1], b[1]) for a, b in zip(rows, rows[1:])) / duration

    def sample(self, elapsed, now):
        fresh = self.values is not None and self.received is not None and now - self.received <= 1.
        values = self.values if fresh else (float('nan'),) * self.dimensions
        self.rows.append((elapsed, *values))
        while self.rows and self.rows[0][0] < elapsed - self.window:
            self.rows.popleft()
        if self.dimensions == 3:
            self.speed_rows.append((elapsed, self.path_speed(now)))
            while self.speed_rows and self.speed_rows[0][0] < elapsed - self.window:
                self.speed_rows.popleft()
        return [elapsed, self.stamp * 1e-9 if self.stamp is not None else '', self.frame, *values]

    def caption(self, now):
        if self.received is None:
            return 'Waiting for data', False
        age = now - self.received
        if self.values is None or age > 1.:
            return f'Frame: {self.frame or "unspecified"} | STALE / invalid — last fresh sample {age:.1f} s ago', False
        return f'Frame: {self.frame or "unspecified"} | sample age {age:.2f} s', True


class Figures:
    def __init__(self, plt, wrench_topic, tcp_topic):
        self.plt = plt
        fig, axes = plt.subplots(3, 2, sharex=True, figsize=(12, 9))
        fig.canvas.manager.set_window_title('FR10 — force, torque, TCP and speed')
        fig.suptitle('FR10 — Live force, torque, TCP position and speed', fontsize=12)
        self.wrench_title = fig.text(.06, .94, '', fontsize=9)
        self.tcp_title = fig.text(.06, .915, '', fontsize=9)
        force_axis, torque_axis, x_axis, y_axis, z_axis, self.speed_axis = axes.flat
        self.wrench_axes = (force_axis,) * 3 + (torque_axis,) * 3
        self.wrench_lines = []
        for axis, label in zip(self.wrench_axes, ('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz')):
            line, = axis.plot([], [], label=label, linewidth=1.)
            self.wrench_lines.append(line)
        force_axis.set_title('Fx / Fy / Fz')
        force_axis.set_ylabel('Force [N]')
        torque_axis.set_title('Tx / Ty / Tz')
        torque_axis.set_ylabel('Torque [Nm]')
        for axis in (force_axis, torque_axis):
            axis.legend(loc='upper right', ncol=3, fontsize=8)
        self.figures = {'dashboard': fig}
        self.position_axes = (x_axis, y_axis, z_axis)
        self.position_lines = []
        for name, axis in zip(('X', 'Y', 'Z'), self.position_axes):
            line, = axis.plot([], [], label=f'TCP {name.upper()}', linewidth=1.)
            axis.set_title(f'TCP {name}')
            axis.set_ylabel(f'{name.upper()} [mm]')
            axis.ticklabel_format(axis='y', style='plain', useOffset=False)
            self.position_lines.append(line)
        self.speed_line, = self.speed_axis.plot([], [], label='TCP path speed', color='tab:purple')
        self.speed_axis.set_title('TCP speed (0.5 s wall-time window)')
        self.speed_axis.set_ylabel('Speed [mm/s]')
        self.speed_axis.set_ylim(bottom=0.)
        for axis in axes.flat:
            axis.grid(alpha=.3)
        for axis in axes[-1]:
            axis.set_xlabel('Wall time [s]')
        fig.tight_layout(rect=(0, 0, 1, .9))
        self.wrench_topic, self.tcp_topic = wrench_topic, tcp_topic

    def update(self, wrench, tcp, now):
        for stream, lines, axes in ((wrench, self.wrench_lines, self.wrench_axes),
                                    (tcp, self.position_lines, self.position_axes)):
            if stream.rows:
                columns = tuple(zip(*stream.rows))
                for line, values, axis in zip(lines, columns[1:], axes):
                    line.set_data(columns[0], values)
                    axis.relim()
                    axis.autoscale_view()
                    end = max(.1, stream.rows[-1][0])
                    axis.set_xlim(max(0., end - stream.window), end)
        caption, fresh = wrench.caption(now)
        self.wrench_title.set_text(f'Wrench: {self.wrench_topic} | {caption}')
        self.wrench_title.set_color('black' if fresh else 'red')
        caption, fresh = tcp.caption(now)
        self.tcp_title.set_text(f'TCP: {self.tcp_topic} | {caption}')
        self.tcp_title.set_color('black' if fresh else 'red')
        if tcp.speed_rows:
            columns = tuple(zip(*tcp.speed_rows))
            self.speed_line.set_data(*columns)
            self.speed_axis.relim()
            self.speed_axis.autoscale(enable=True, axis='y')
            self.speed_axis.set_ylim(bottom=0.)

    def is_open(self):
        return any(self.plt.fignum_exists(fig.number) for fig in self.figures.values())

    def draw(self):
        for fig in self.figures.values():
            if self.plt.fignum_exists(fig.number):
                fig.canvas.draw_idle()
                fig.canvas.flush_events()

    def save(self, output):
        for name, fig in self.figures.items():
            fig.savefig(output / f'{name}.png', dpi=140)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wrench-topic', default='/ft_sensor/wrench')
    parser.add_argument('--tcp-topic', default='/fr10_motion/tcp_pose')
    parser.add_argument('--window', type=float, default=120., help='Visible wall-time window [s]')
    parser.add_argument('--duration', type=float, default=0., help='Wall seconds; 0 until the window closes')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--output', type=Path, default=None, help='Directory for CSV and dashboard.png')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.window) or not 1. <= args.window <= 3600.:
        parser.error('--window must be in 1..3600 seconds')
    if not math.isfinite(args.duration) or args.duration < 0 or (args.headless and args.duration <= 0):
        parser.error('--duration must be finite and nonnegative; --headless needs a positive duration')
    output = args.output or Path('test_results') / ('fr10_live_plot_' + datetime.now().strftime('%Y%m%dT%H%M%S_%f'))
    output.mkdir(parents=True, exist_ok=False)

    import matplotlib
    if args.headless:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, WrenchStamped

    wrench, tcp = Stream(6, args.window), Stream(3, args.window)
    started = time.monotonic()

    def stamp(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def on_wrench(message):
        f, t = message.wrench.force, message.wrench.torque
        wrench.push(stamp(message), message.header.frame_id,
                    (f.x, f.y, f.z, t.x, t.y, t.z), time.monotonic())

    def on_tcp(message):
        p = message.pose.position
        tcp.push(stamp(message), message.header.frame_id,
                 (p.x * 1000., p.y * 1000., p.z * 1000.), time.monotonic())

    rclpy.init(args=ros_args)
    node = Node('fr10_robot_plot')
    node.create_subscription(WrenchStamped, args.wrench_topic, on_wrench, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, args.tcp_topic, on_tcp, qos_profile_sensor_data)
    figures = Figures(plt, args.wrench_topic, args.tcp_topic)
    if not args.headless:
        plt.show(block=False)
    print(f'One read-only window with six panels. Output: {output.resolve()}', flush=True)
    print('Close the window or press Ctrl+C to save and exit.', flush=True)
    try:
        with (output / 'wrench.csv').open('w', newline='', buffering=1) as wf, \
                (output / 'tcp.csv').open('w', newline='', buffering=1) as pf:
            writers = csv.writer(wf), csv.writer(pf)
            writers[0].writerow(['wall_s', 'source_s', 'frame', 'fx_n', 'fy_n', 'fz_n', 'tx_nm', 'ty_nm', 'tz_nm'])
            writers[1].writerow(['wall_s', 'source_s', 'frame', 'x_mm', 'y_mm', 'z_mm', 'speed_wall_mm_s'])
            next_sample = next_draw = started
            while rclpy.ok() and figures.is_open():
                rclpy.spin_once(node, timeout_sec=.01)
                now = time.monotonic()
                if now >= next_sample:
                    for stream, writer in zip((wrench, tcp), writers):
                        row = stream.sample(now - started, now)
                        if stream is tcp:
                            row.append(tcp.speed_rows[-1][1])
                        writer.writerow(row)
                    next_sample = now + .05
                if now >= next_draw:
                    figures.update(wrench, tcp, now)
                    if not args.headless:
                        figures.draw()
                    next_draw = now + .5
                if args.duration and now - started >= args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        figures.update(wrench, tcp, time.monotonic())
        figures.save(output)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for fig in figures.figures.values():
            plt.close(fig)
        print(f'Saved dashboard.png and two CSV files: {output.resolve()}')


if __name__ == '__main__':
    main()
