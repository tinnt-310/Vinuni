#!/usr/bin/python3
"""Read-only force/depth/clock dashboard, sampled at 5 Hz of wall time."""
import argparse
from collections import deque
import csv
from pathlib import Path
import time

import rclpy

from fr10_force_status import ForceStatus


def slope(rows, now):
    recent = [row for row in rows if row[0] >= now - 5.]
    if len(recent) < 2 or recent[-1][0] <= recent[0][0]:
        return float('nan')
    return (recent[-1][1] - recent[0][1]) / (recent[-1][0] - recent[0][0])


class PlotStatus(ForceStatus):
    def report(self):
        # The figure displays status; keep the terminal quiet.
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0.,
                        help='Stop after this many wall seconds; 0 runs until closed')
    parser.add_argument('--headless', action='store_true', help='Save without opening a window')
    parser.add_argument('--output', type=Path, default=None,
                        help='Save PNG and CSV at this file prefix')
    args, ros_args = parser.parse_known_args()
    if args.duration < 0 or (args.headless and (args.duration <= 0 or args.output is None)):
        parser.error('--headless requires --duration > 0 and --output; duration must be >= 0')

    import matplotlib
    if args.headless:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
    fig.canvas.manager.set_window_title('FR10 force monitor (read only)')
    title = fig.suptitle('Waiting for /fr10_force/feedback ...')
    force_lines = [axes[0].plot([], [], label=label, **style)[0]
                   for label, style in [('Fx', {}), ('Fy', {}), ('Fz', {}),
                                        ('Fz reference (active phase)', {'linestyle': '--'})]]
    depth_line, = axes[1].plot([], [], label='TCP depth')
    axes[1].axhline(0., color='gray', linestyle=':', label='Port mouth')
    rtf_line, = axes[2].plot([], [], label='Gazebo RTF (5 s window)')
    speed_line, = axes[3].plot([], [], label='TCP speed along measured 3D path (5 s window)')
    for ax, ylabel in zip(axes, ['Force [N]', 'Depth [mm]', 'Sim / wall time', 'TCP speed [mm/s wall]']):
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=.3)
        ax.legend(loc='upper left', fontsize=8)
    axes[3].set_xlabel('Wall time since monitor started [s]')
    fig.tight_layout(rect=(0, 0, 1, .90))
    if not args.headless:
        plt.show(block=False)

    output_file = None
    writer = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        output_file = args.output.with_suffix('.csv').open('w', newline='', buffering=1)
        writer = csv.writer(output_file)
        writer.writerow(['wall_s', 'generation', 'phase', 'feedback_age_s', 'fx_n', 'fy_n',
                         'fz_n', 'fz_reference_n', 'depth_mm', 'rtf', 'tcp_speed_wall_mm_s',
                         'axial_speed_wall_mm_s'])
    rclpy.init(args=ros_args)
    node = PlotStatus()
    rows = deque(maxlen=1500)  # Five minutes on screen; CSV retains every sample.
    started = time.monotonic()
    next_sample = started
    next_draw = started
    generation = None
    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            elapsed = now - started
            if now >= next_sample and node.feedback is not None:
                feedback = node.feedback
                age = now - node.received
                if generation != feedback.generation:
                    rows.clear()
                    generation = feedback.generation
                preparing = feedback.state in (feedback.IDLE, feedback.MOVE_PREINSERT,
                                               feedback.HOLD_PREINSERT, feedback.FT_ZERO)
                force = feedback.measured_wrench.force
                reference = float('nan') if preparing else force.z + feedback.force_error.force.z
                rtf = slope(node.clocks, now)
                speed = slope(node.depths, now) * 1000.
                tcp_speed = node.tcp_speed_wall_mm_s(now)
                values = [force.x, force.y, force.z, reference, feedback.depth * 1000., rtf, tcp_speed]
                if age > 1.:
                    values = [float('nan')] * len(values)
                row = [elapsed, *values]
                rows.append(row)
                if writer:
                    writer.writerow([elapsed, generation, feedback.detail, age, *values, speed])
                title.set_text(
                    f'FORCE {feedback.detail}\n'
                    f'Depth {feedback.depth*1000:.2f} mm | Fz {force.z:.3f} N | '
                    f'RTF {rtf:.3f} | TCP {tcp_speed:.3f} mm/s wall | feedback age {age:.1f} s')
                title.set_color('red' if age > 1. else 'black')
                next_sample = now + .2
            # Render only twice per wall second to limit CPU use alongside Gazebo.
            if now >= next_draw:
                if rows:
                    columns = list(zip(*rows))
                    for line, values in zip(force_lines + [depth_line, rtf_line, speed_line], columns[1:]):
                        line.set_data(columns[0], values)
                    for ax in axes:
                        ax.relim()
                        ax.autoscale_view()
                if not args.headless:
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                next_draw = now + .5
            if args.duration > 0 and elapsed >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if args.output:
            # Include the last sample even when closing between screen refreshes.
            if rows:
                columns = list(zip(*rows))
                for line, values in zip(force_lines + [depth_line, rtf_line, speed_line], columns[1:]):
                    line.set_data(columns[0], values)
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view()
            fig.savefig(args.output.with_suffix('.png'), dpi=140)
            output_file.close()
            print(f'Saved {args.output.with_suffix(".png")} and {args.output.with_suffix(".csv")}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        plt.close(fig)


if __name__ == '__main__':
    main()
