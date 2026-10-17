import numpy as np
import pytest
import importlib.util
import json
from pathlib import Path
from scipy.spatial.transform import Rotation
from fr10_pose_uncertainty import input_socket_pose


def test_only_pose_input_changes_and_translation_is_in_port_frame():
    position = [.5, -.1, .3]
    q = Rotation.from_rotvec([.4, -.5, .7]).as_quat().tolist()
    original_position = list(position)
    original_q = list(q)
    p, corrected = input_socket_pose(position, q, (1.2, 1.6), (1.2, 1.6, 0.))
    delta = Rotation.from_quat(q).inv().apply(np.array(p) - position)
    assert np.allclose(delta, [.0012, .0016, 0.], atol=1e-12)
    error = (Rotation.from_quat(q).inv() * Rotation.from_quat(corrected)).as_rotvec()
    assert np.allclose(error, np.deg2rad([1.2, 1.6, 0.]), atol=1e-12)
    assert position == original_position and q == original_q


@pytest.mark.parametrize('xy,angle', [((2., 2.), (0., 0., 0.)),
    ((0., 0.), (2., 2., 0.)), ((float('nan'), 0.), (0., 0., 0.))])
def test_invalid_matrix_is_rejected_before_launch(xy, angle):
    with pytest.raises(ValueError):
        input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], xy, angle)


def test_explicit_five_mm_five_degree_envelope_preserves_default_limit():
    with pytest.raises(ValueError):
        input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], (-5., 0.), (0., -5., 0.))
    p, q = input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], (-5., 0.), (0., -5., 0.),
                            max_position_mm=5., max_rotation_deg=5.)
    assert np.allclose(p, [-.005, 0., 0.])
    assert np.allclose(Rotation.from_quat(q).as_rotvec(), np.deg2rad([0., -5., 0.]))
    for value in (0., 6., float('nan'), True):
        with pytest.raises(ValueError):
            input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], max_position_mm=value)


def test_spatial_translation_error_uses_a_ball_not_an_axis_aligned_box():
    p, _ = input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], (3., 0.),
                             axial_mm=4., max_position_mm=5.)
    assert np.allclose(p, [.003, 0., .004])
    with pytest.raises(ValueError):
        input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], (3., 0.),
                          axial_mm=5., max_position_mm=5.)
    with pytest.raises(ValueError):
        input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], axial_mm=float('nan'))


def test_extended_envelope_cannot_launch_gazebo_or_create_output(tmp_path):
    from run_matched_station_gazebo import run
    output = tmp_path / 'must_not_launch'
    with pytest.raises(ValueError, match='requires MuJoCo'):
        run(output, 212, 11996, 'front_mid', False, None, True,
            backend='gazebo', pose_input=True, pose_error_limit=5.)
    assert not output.exists()


def test_keyed_yaw_clearance_matches_independent_rotated_pin_displacement():
    root = Path(__file__).resolve().parents[4]
    spec = importlib.util.spec_from_file_location('charging_station',
        root/'src/vehicle_bringup/launch/charging_station.py')
    station = importlib.util.module_from_spec(spec); spec.loader.exec_module(station)
    geometry = json.loads((root/'src/vehicle_description/models/matched_socket_v2/geometry.json').read_text())
    angle = station.keyed_yaw_half_span(geometry)
    points = np.asarray([pin['center_mm'] for pin in geometry['pins']])
    positions = np.c_[points, np.zeros(len(points))]
    displacement = np.linalg.norm(Rotation.from_rotvec([0., 0., angle]).apply(positions)-positions, axis=1)
    assert max(displacement) == pytest.approx(geometry['simulation_design']['radial_clearance_mm'], abs=1e-12)
    assert 0 < angle < np.deg2rad(2)
    for clearance in (0., float('nan'), 1000.):
        geometry['simulation_design']['radial_clearance_mm'] = clearance
        with pytest.raises(ValueError):
            station.keyed_yaw_half_span(geometry)
