"""Rigid socket triangles preserve CAD holes and measured force conventions."""
import importlib.util
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from fr10_mujoco_bridge import Fr10MujocoPlant, ROOT, DESCRIPTION, HOME
from fr10_mating_geometry import load_stl
import fr10_matched_mujoco as matched


@pytest.fixture(scope='module')
def plant(tmp_path_factory):
    spec=importlib.util.spec_from_file_location('charging_station',ROOT/'src/vehicle_bringup/launch/charging_station.py')
    station_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(station_module)
    model=DESCRIPTION/'models/matched_socket_v2'
    station=station_module.select_station(ROOT/'src/vehicle_bringup/config/charging_stations.yaml',
        model/'geometry.json','front_mid','<robot/>',False)
    station['model_path']=str(model)
    path=tmp_path_factory.mktemp('station')/'station.json';path.write_text(json.dumps(station))
    return Fr10MujocoPlant(station_file=path)


def triangle_rows(triangles):
    rows=[]
    for triangle in triangles:
        order=np.lexsort((triangle[:,2],triangle[:,1],triangle[:,0]))
        rows.append(tuple(triangle[order].flatten()))
    return np.asarray(sorted(rows))


def test_original_nonconvex_triangles_no_added_dofs_or_surface_radius(plant):
    model=plant.model
    assert model.nq==6 and model.nv==6 and model.nflex==2
    assert np.all(model.flex_rigid) and np.all(model.flex_radius==0.)
    for flex,filename in enumerate(('socket_contact_surface.stl','socket_protected_rear.stl')):
        start=model.flex_elemdataadr[flex];count=model.flex_elemnum[flex]
        indices=model.flex_elem[start:start+3*count].reshape(-1,3)
        triangles=model.flex_vert[model.flex_vertadr[flex]+indices]
        original=load_stl(DESCRIPTION/'models/matched_socket_v2'/filename)
        assert triangles.shape==original.shape
        assert np.max(np.abs(triangle_rows(triangles)-triangle_rows(original))) < 1e-12


def test_original_visuals_preserve_physics_and_measured_tcp(plant, tmp_path):
    from view_fr10_force_mujoco import configure_rendering
    path = tmp_path / 'station.json'
    path.write_text(json.dumps(plant.matched_station))
    view = Fr10MujocoPlant(station_file=path, include_visuals=True)
    visuals = {view.model.geom(i).name: i for i in range(view.model.ngeom)
               if view.model.geom(i).name.startswith('visual__')}
    for link in ('fr10_base_link', 'shoulder_link', 'upperarm_link', 'forearm_link',
                 'wrist1_link', 'wrist2_link', 'wrist3_link', 'charging_connector_link',
                 'charging_mating_nose', 'charging_mating_shoulder', 'camera_link'):
        geom = visuals[f'visual__{link}__0']
        assert view.model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH
        assert view.model.geom_group[geom] == 2
        assert view.model.geom_contype[geom] == view.model.geom_conaffinity[geom] == 0
    for name in ('body_mass', 'body_inertia', 'body_pos', 'body_quat', 'jnt_range',
                 'actuator_gainprm', 'actuator_biasprm'):
        np.testing.assert_allclose(getattr(view.model, name), getattr(plant.model, name), atol=1e-12, rtol=1e-12)
    # Geometry names survive import; adding visual CAD must not add or change
    # any contact surface, measured transform, constraint or seating force.
    physical = {plant.model.geom(i).name for i in range(plant.model.ngeom)}
    assert {view.model.geom(i).name for i in range(view.model.ngeom)} - set(visuals) == physical
    for name in physical:
        first, second = plant.model.geom(name).id, view.model.geom(name).id
        for field in ('geom_contype', 'geom_conaffinity', 'geom_pos', 'geom_quat', 'geom_size'):
            np.testing.assert_allclose(getattr(plant.model, field)[first],
                                       getattr(view.model, field)[second], atol=1e-12, rtol=1e-12)
    for depth in (-.001, .007, .01502):
        place_tcp(plant, depth)
        view.data.qpos[:] = plant.data.qpos
        view.data.qvel[:] = 0.
        view.data.ctrl[:] = plant.data.ctrl
        mujoco.mj_forward(view.model, view.data)
        view.data.qfrc_applied[:] = view.data.qfrc_bias
        mujoco.mj_forward(view.model, view.data)
        np.testing.assert_allclose(view.tcp_pose()[0], plant.tcp_pose()[0], atol=1e-12)
        assert view.data.ncon == plant.data.ncon
        np.testing.assert_allclose(view.contact_wrench_tcp(), plant.contact_wrench_tcp(), atol=1e-7, rtol=1e-7)
    option = mujoco.MjvOption()
    configure_rendering(option, True)
    assert not option.geomgroup[0] and option.geomgroup[1] and option.geomgroup[2]
    assert not option.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL]


def place_tcp(plant,depth):
    target=plant.contact_tcp_position+plant.insertion_axis_world*depth
    target_rotation=Rotation.from_quat(plant.matched_station['orientation_arm_xyzw']).as_matrix()
    def residual(q):
        plant.data.qpos[:6]=q;mujoco.mj_kinematics(plant.model,plant.data)
        position,rotation=plant.tcp_pose()
        return np.r_[position-target,Rotation.from_matrix(target_rotation.T@rotation).as_rotvec()]
    solution=least_squares(residual,HOME,ftol=1e-12,xtol=1e-12,gtol=1e-12)
    assert np.max(np.abs(solution.fun))<1e-9
    plant.data.qpos[:6]=solution.x;plant.data.qvel[:]=0.;plant.data.ctrl[:]=solution.x
    mujoco.mj_forward(plant.model,plant.data)
    # Runtime compensates gravity at every split step. Measure contact under
    # that same load convention, without counting gravity support as seating.
    plant.data.qfrc_applied[:6] = plant.data.qfrc_bias[:6]
    mujoco.mj_forward(plant.model,plant.data)


def test_open_bores_and_actual_seat_contact_wrench(plant):
    for depth in (-.001,.007,.0145):
        place_tcp(plant,depth)
        assert plant.data.ncon==0  # No convex hull filling the original bores.
        assert np.linalg.norm(plant.contact_wrench_tcp())<1e-9
    place_tcp(plant,.01502)
    assert plant.data.ncon>0
    pairs=[plant.contact_names(c) for c in plant.data.contact[:plant.data.ncon]]
    assert all('socket_surface_exact' in pair for pair in pairs)
    assert all(any(name.startswith('charging_mating_shoulder__') for name in pair) for pair in pairs)
    wrench=plant.contact_wrench_tcp()
    assert np.all(np.isfinite(wrench)) and wrench[2]>1.


def test_batched_contact_wrench_matches_independent_per_contact_sum(plant):
    for depth in (-.001, .015005, .01502, .01504):
        place_tcp(plant, depth)
        position, rotation = plant.tcp_pose()
        force, moment = np.zeros(3), np.zeros(3)
        for index, contact in enumerate(plant.data.contact):
            first, second = map(int, contact.geom)
            fixture = [int(g) in plant.fixture_geom_ids or int(f) in plant.fixture_flex_ids
                       for g, f in zip(contact.geom, contact.flex)]
            if not ((first in plant.tool_geom_ids and fixture[1]) or
                    (second in plant.tool_geom_ids and fixture[0])):
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(plant.model, plant.data, index, local)
            frame = np.asarray(contact.frame).reshape(3, 3)
            sign = 1. if first in plant.tool_geom_ids else -1.
            applied = sign * frame.T @ local[:3]
            force += applied
            moment += np.cross(contact.pos - position, applied) + sign * frame.T @ local[3:]
        assert np.allclose(plant.contact_wrench_tcp(),
                           np.r_[rotation.T @ force, rotation.T @ moment], rtol=1e-12, atol=1e-12)


def test_smooth_contact_onset_still_carries_seated_load_within_penetration_gate(plant):
    forces = []
    for penetration in (0., .0000001, .000001, .00001, .00004):
        place_tcp(plant, .015 + penetration)
        wrench = plant.contact_wrench_tcp()
        assert np.all(np.isfinite(wrench))
        assert max([-c.dist for c in plant.data.contact[:plant.data.ncon]], default=0.) <= .00005
        forces.append(wrench[2])
    assert abs(forces[0]) < .001 and forces[1] < .001
    assert np.all(np.diff(forces) >= 0.)
    assert forces[-1] > 5.  # Positive seating load, not a contact-free model.


def test_invalid_contact_calibration_fails_closed(tmp_path, monkeypatch):
    import yaml
    config = tmp_path/'config'; config.mkdir()
    monkeypatch.setattr(matched, 'DESCRIPTION', tmp_path)
    valid = dict(solref=[-20000., -300.], solimp=[0., .99, .00005, .5, 2.],
                 ccd_tolerance=1e-10, ccd_iterations=200)
    for key, invalid in (
            ('solref', [-20000.]), ('solref', [20000., 300.]),
            ('solref', [float('nan'), -300.]),
            ('solimp', [0., .99, .00005]),
            ('solimp', [0., 1., .00005, .5, 2.]),
            ('solimp', [0., .99, .000051, .5, 2.]),
            ('solimp', [0., .99, .00005, 0., 2.]),
            ('solimp', [0., .99, .00005, .5, float('nan')])):
        values = {**valid, key: invalid}
        (config/'fr10_simulation_actuators.yaml').write_text(
            yaml.safe_dump(dict(matched_mujoco_contact=values)))
        with pytest.raises(ValueError, match='contact calibration'):
            matched.matched_contact_parameters()


def test_collision_precision_configuration_fails_closed(tmp_path, monkeypatch):
    import yaml
    config = tmp_path/'config'; config.mkdir()
    monkeypatch.setattr(matched, 'DESCRIPTION', tmp_path)
    valid = dict(solref=[-20000., -300.], solimp=[0., .99, .00005, .5, 2.],
                 ccd_tolerance=1e-10, ccd_iterations=200)
    for key, invalid in (('ccd_tolerance', 1e-9), ('ccd_tolerance', 0.),
            ('ccd_tolerance', float('nan')), ('ccd_iterations', 100),
            ('ccd_iterations', True), ('ccd_iterations', 100.5), ('ccd_iterations', 1001)):
        (config/'fr10_simulation_actuators.yaml').write_text(yaml.safe_dump(
            dict(matched_mujoco_contact={**valid, key: invalid})))
        with pytest.raises(ValueError, match='collision precision'):
            matched.matched_contact_parameters()


def test_tilted_bore_contact_distance_converges_without_false_impulse(plant):
    # Measured q immediately before the 164 um pulse in stage12/Rx+2 deg.
    # Adjacent poses differed by submicrometres. The default convex tolerance
    # jumped between almost zero and 164 um on an unchanged surface pair.
    q = np.array([-1.087831500502551e-7, -1.5175056310972548,
                  2.108144358049546, -2.126531049296966,
                  -1.57079612073515, -4.722411068696397e-7])
    model = plant.model
    original = model.opt.ccd_iterations, model.opt.ccd_tolerance
    try:
        for perturbation in (-2e-8, -1e-8, 0., 1e-8, 2e-8):
            measured = q.copy(); measured[3] += perturbation
            peaks = []
            for iterations, tolerance in ((100, 1e-9), (200, 1e-10)):
                model.opt.ccd_iterations, model.opt.ccd_tolerance = iterations, tolerance
                plant.observe_configuration(measured)
                peaks.append(max([-c.dist for c in plant.data.contact[:plant.data.ncon]], default=0.))
            assert max(peaks) < 1e-6
            assert abs(peaks[0]-peaks[1]) < 1e-8
        # Refining the distance solver must retain real seating loads.
        place_tcp(plant, .01504)
        assert plant.contact_wrench_tcp()[2] > 5.
    finally:
        model.opt.ccd_iterations, model.opt.ccd_tolerance = original


@pytest.mark.parametrize('q', [
    [-.00027387869372695557, -1.5166490154564745, 2.1333830825743907,
     -2.1874653221430935, -1.5696902543562652, .00045790083916037223],
    [-.000267537591722315, -1.516628327119846, 2.1333606686199693,
     -2.18746350847308, -1.5696896937840925, .0005126476046116616],
])
def test_combined_error_shoulder_distance_converges_without_four_mm_phantom(plant, q):
    # Stage22 combined X+2 mm/Ry+2 deg, before the false impulse. The same
    # q has 4.0189 mm penetration at100/1e-9, but8.0815 um at200/1e-10,
    # 400/1e-12 and1000/1e-14. Subsequent real penetration came from that impulse.
    q = np.array(q)
    model = plant.model
    original = model.opt.ccd_iterations, model.opt.ccd_tolerance
    try:
        shoulder = model.geom('charging_mating_shoulder__mating_surface_0')
        assert shoulder.type[0] == mujoco.mjtGeom.mjGEOM_BOX
        for perturbation in (-2e-9, -1e-9, 0., 1e-9, 2e-9):
            measured = q.copy(); measured[3] += perturbation
            peaks = []
            for iterations, tolerance in ((200, 1e-10), (400, 1e-12)):
                model.opt.ccd_iterations, model.opt.ccd_tolerance = iterations, tolerance
                wrench = plant.observe_configuration(measured)
                peaks.append(max([-c.dist for c in plant.data.contact[:plant.data.ncon]], default=0.))
                assert np.isfinite(wrench).all() and np.linalg.norm(wrench[3:]) < 1.
            assert max(peaks) < 1e-5
            assert abs(peaks[0]-peaks[1]) < 1e-8
    finally:
        model.opt.ccd_iterations, model.opt.ccd_tolerance = original


def test_native_cad_observation_uses_measured_pose_without_integrating(plant, monkeypatch):
    place_tcp(plant, .01504)
    measured = plant.data.qpos.copy()
    expected_pose = plant.tcp_pose()[0].copy()
    expected_force = plant.contact_wrench_tcp().copy()
    initial_time = plant.data.time
    def integration_forbidden(*args):
        raise AssertionError('A native geometry observer must not advance a second servo')
    monkeypatch.setattr(mujoco, 'mj_step2', integration_forbidden)
    monkeypatch.setattr(mujoco, 'mj_step', integration_forbidden)
    wrench = plant.observe_configuration(measured)
    assert np.array_equal(plant.data.qpos, measured)
    assert np.array_equal(plant.data.qvel, np.zeros(6))
    assert plant.data.time == initial_time
    assert np.max(np.abs(plant.tcp_pose()[0]-expected_pose)) < 1e-12
    assert np.max(np.abs(wrench-expected_force)) < .001 and wrench[2] > 5.
    for invalid in (np.zeros(5), np.full(6, float('nan')), np.full(6, 100.)):
        with pytest.raises(ValueError, match='measured joints'):
            plant.observe_configuration(invalid)
        assert np.array_equal(plant.data.qpos, measured) and plant.data.time == initial_time
