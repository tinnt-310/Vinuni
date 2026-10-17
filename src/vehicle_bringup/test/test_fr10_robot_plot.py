"""A read-only plot must not present frozen or mixed-frame data as live."""
import importlib.util
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


spec = importlib.util.spec_from_file_location('robot_plot',
    Path(__file__).parents[1] / 'scripts/fr10_robot_plot.py')
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)


def test_duplicate_source_samples_expire_and_out_of_order_is_ignored():
    stream = plot.Stream(3, 2.)
    stream.push(2_000_000_000, 'base', (1., 2., 3.), 10.)
    stream.push(2_000_000_000, 'base', (9., 9., 9.), 10.9)
    stream.push(1_999_000_000, 'base', (8., 8., 8.), 10.95)
    assert stream.sample(1., 10.99)[3:] == [1., 2., 3.]
    assert all(math.isnan(v) for v in stream.sample(1.1, 11.1)[3:])
    assert 'STALE' in stream.caption(11.1)[0]
    stream.push(2_010_000_000, 'base', (4., 5., 6.), 11.2)
    assert stream.sample(1.2, 11.2)[3:] == [4., 5., 6.]


def test_frame_change_and_clock_restart_do_not_connect_different_traces():
    stream = plot.Stream(3, 2.)
    stream.push(10_000_000_000, 'base', (1., 2., 3.), 10.)
    stream.sample(0., 10.)
    stream.push(11_000_000_000, 'world', (4., 5., 6.), 11.)
    assert not stream.rows
    stream.sample(1., 11.)
    stream.push(0, 'world', (7., 8., 9.), 12.)
    assert not stream.rows
    assert stream.sample(2., 12.)[1:] == [0., 'world', 7., 8., 9.]
    stream.push(1, 'world', (float('nan'), 0., 0.), 12.1)
    assert all(math.isnan(v) for v in stream.sample(2.1, 12.1)[3:])


def test_path_speed_preserves_round_trip_motion_and_expires_with_frozen_source():
    tcp = plot.Stream(3, 2.)
    for stamp, position, wall in [(1, (0., 0., 0.), 10.),
                                  (2, (5., 0., 0.), 10.25),
                                  (3, (0., 0., 0.), 10.5)]:
        tcp.push(stamp, 'base', position, wall)
    assert tcp.path_speed(10.5) == 20.
    tcp.push(3, 'base', (0., 0., 0.), 11.4)
    assert math.isnan(tcp.path_speed(11.6))
    tcp.push(4, 'base', (100., 0., 0.), 12.)
    assert math.isnan(tcp.path_speed(12.))


def test_single_figure_groups_wrench_channels_and_preserves_position_and_speed(tmp_path):
    wrench, tcp = plot.Stream(6, 2.), plot.Stream(3, 2.)
    wrench.push(1, 'sensor', (1., 2., 3., 4., 5., 6.), 10.)
    tcp.push(1, 'base', (101., 202., 303.), 10.)
    wrench.sample(0., 10.)
    tcp.sample(0., 10.)
    tcp.push(2, 'base', (106., 202., 303.), 10.5)
    tcp.sample(.5, 10.5)
    figures = plot.Figures(plt, '/wrench', '/tcp')
    try:
        figures.update(wrench, tcp, 10.5)
        assert len(figures.figures) == 1
        assert len(figures.figures['dashboard'].axes) == 6
        assert [list(line.get_ydata()) for line in figures.wrench_lines] == [[v] for v in range(1, 7)]
        assert [list(line.get_ydata()) for line in figures.position_lines] == [[101., 106.], [202., 202.], [303., 303.]]
        assert len({line.axes for line in figures.wrench_lines[:3]}) == 1
        assert len({line.axes for line in figures.wrench_lines[3:]}) == 1
        assert figures.wrench_lines[0].axes is not figures.wrench_lines[3].axes
        assert list(figures.speed_line.get_ydata())[-1] == 10.
        assert figures.speed_axis.get_ylim()[1] >= 10.
        figures.save(tmp_path)
        assert (tmp_path / 'dashboard.png').stat().st_size > 1000
    finally:
        for fig in figures.figures.values():
            plt.close(fig)
