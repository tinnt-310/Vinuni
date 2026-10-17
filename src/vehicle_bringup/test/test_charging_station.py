import importlib.util
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('charging_station', ROOT/'launch/charging_station.py')
station = importlib.util.module_from_spec(spec); spec.loader.exec_module(station)


def test_station_faces_and_single_model_datum():
    registry = ROOT/'config/charging_stations.yaml'
    model = ROOT.parents[1]/'src/vehicle_description/models/matched_socket_v2'
    geometry = model/'geometry.json'
    xml = '<robot><joint name="mount" type="fixed"><parent link="base_link"/><child link="fr10_base_link"/><origin xyz=".5 0 1.2" rpy="0 0 3.141592653589793"/></joint></robot>'
    for mounted in (False, True):
        for name in yaml.safe_load(registry.read_text())['stations']:
            selected = station.select_station(registry, geometry, name, xml, mounted)
            assert selected['acceptance'] == 'PENDING'
            rotation = station.rpy_matrix(selected['world_mouth_rpy'])
            assert station.rotate(rotation, [0,0,-1])[2] >= -1e-9
            assert selected['insertion_depth_m'] == json.loads(geometry.read_text())['simulation_design']['nose_length_mm']*.001
            assert len(selected['initial_joint_rad']) == 4
            assert all(len(q) == 6 for q in selected['initial_joint_rad'].values())
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'world.sdf'
                station.write_world(selected, model, path)
                physics = ET.parse(path).find('.//physics')
                step = float(physics.find('max_step_size').text)
                assert 0.00005 <= step <= 0.001
                assert abs(step*float(physics.find('real_time_update_rate').text)-1.) < 1e-12
                collisions = ET.parse(path).findall(".//model[@name='matched_charging_station']/link/collision")
                assert {c.get('name') for c in collisions} == {'mating_surface', 'protected_rear'}
                assert all(c.find('geometry/mesh') is not None for c in collisions)
                sensors = ET.parse(path).findall('.//sensor/contact/collision')
                assert {item.text for item in sensors} == {'mating_surface', 'protected_rear'}
                assert ET.parse(path).findall('.//include') == []
                assert len(selected['world_primitives']) == 4
                assert selected['world_primitives'][0]['position_world_m'][2]+.01 == 0.
                assert all(p['size_m'][2] > 0 for p in selected['world_primitives'])


def test_invalid_integration_step_fails_before_world_generation(tmp_path):
    model = tmp_path/'models/matched_socket_v2'; model.mkdir(parents=True)
    config = tmp_path/'config'; config.mkdir()
    path = tmp_path/'world.sdf'
    for invalid in (False, 0., -.001, .00101, .00001, float('nan'), '0.0005'):
        (config/'fr10_simulation_actuators.yaml').write_text(yaml.safe_dump(
            dict(matched_gazebo_physics=dict(max_step_size_s=invalid))))
        with pytest.raises(ValueError, match='integration step'):
            station.write_world({}, model, path)
        assert not path.exists()


def test_explicit_convergence_step_preserves_default_and_rejects_unsafe_values(tmp_path):
    model = ROOT.parents[1]/'src/vehicle_description/models/matched_socket_v2'
    selected = station.select_station(ROOT/'config/charging_stations.yaml',
                                      model/'geometry.json', 'front_mid', '<robot/>', False)
    path = tmp_path/'world.sdf'
    for step in (.00005, .00025, .001):
        station.write_world(selected, model, path, step_s=step)
        assert float(ET.parse(path).find('.//max_step_size').text) == step
    path.unlink()
    for invalid in (False, 0., .00101, .00001, float('nan'), float('inf'), '.001'):
        with pytest.raises(ValueError, match='integration step'):
            station.write_world(selected, model, path, step_s=invalid)
        assert not path.exists()
    station.write_world(selected, model, path)
    expected = yaml.safe_load((model.parents[1]/'config/fr10_simulation_actuators.yaml').read_text())
    assert float(ET.parse(path).find('.//max_step_size').text) == expected['matched_gazebo_physics']['max_step_size_s']


def test_near_preinsert_uses_one_clearance_for_target_and_frames():
    model = ROOT.parents[1]/'src/vehicle_description/models/matched_socket_v2'
    args = (ROOT/'config/charging_stations.yaml', model/'geometry.json',
            'front_mid', '<robot/>', False)
    baseline = station.select_station(*args)
    near = station.select_station(*args, preinsert_distance_m=.005)
    assert baseline['preinsert_distance_m'] == .03
    assert near['preinsert_distance_m'] == .005
    assert near['preinsert_position_arm_m'] == pytest.approx([-.72, -.159, .325])
    for key in ('world_mouth_position_m', 'world_mouth_rpy', 'world_primitives',
                'seated_position_arm_m', 'insertion_depth_m', 'canonical_sha256'):
        assert near[key] == baseline[key]
    for invalid in (False, 0., .004, .401, float('nan'), float('inf'), '.005'):
        with pytest.raises(ValueError, match='clearance'):
            station.select_station(*args, preinsert_distance_m=invalid)


def test_solver_candidate_changes_only_iterations_and_rejects_invalid_values(tmp_path):
    model = ROOT.parents[1]/'src/vehicle_description/models/matched_socket_v2'
    selected = station.select_station(ROOT/'config/charging_stations.yaml',
                                     model/'geometry.json', 'front_mid', '<robot/>', False)
    baseline = tmp_path/'baseline.sdf'; candidate = tmp_path/'candidate.sdf'
    station.write_world(selected, model, baseline)
    station.write_world(selected, model, candidate, solver_iterations=1000)
    expected = ET.parse(baseline); observed = ET.parse(candidate)
    assert observed.findtext('.//solver/iters') == '1000'
    observed.find('.//solver/iters').text = '2000'
    assert ET.tostring(observed.getroot()) == ET.tostring(expected.getroot())
    for invalid in (False, 0, 49, 2001, 500.5, float('nan'), '500'):
        path = tmp_path/'invalid.sdf'
        with pytest.raises(ValueError, match='solver iterations'):
            station.write_world(selected, model, path, solver_iterations=invalid)
        assert not path.exists()


def test_invalid_station_and_nonfixed_mount_rejected():
    registry = ROOT/'config/charging_stations.yaml'
    geometry = ROOT.parents[1]/'src/vehicle_description/models/matched_socket_v2/geometry.json'
    with pytest.raises(ValueError, match='Unknown'):
        station.select_station(registry, geometry, 'unknown', '<robot/>', False)
    with pytest.raises(ValueError, match='fixed chain'):
        station.arm_mount('<robot><joint type="revolute"><parent link="base"/><child link="fr10_base_link"/></joint></robot>', True)


def test_initial_configuration_updates_only_six_arm_states():
    registry = yaml.safe_load((ROOT/'config/charging_stations.yaml').read_text())
    xml = '<robot><ros2_control name="arm">'+''.join(
        f'<joint name="j{i}"><command_interface name="position"/><state_interface name="position"><param name="initial_value">0</param></state_interface></joint>'
        for i in range(1,7))+'</ros2_control><ros2_control name="car"><joint name="wheel"/></ros2_control></robot>'
    for name, values in registry['initial_joint_rad'].items():
        root = ET.fromstring(station.initialize_arm_description(xml, registry, name))
        observed = [float(v.text) for v in root.findall('.//state_interface/param')]
        assert observed == values
        assert root.find(".//joint[@name='wheel']/state_interface") is None
    with pytest.raises(ValueError, match='Unknown initial'):
        station.initial_configuration(registry, 'unknown')
    with pytest.raises(ValueError, match='exactly six'):
        station.initialize_arm_description('<robot/>', registry, 'home')
    registry['initial_joint_rad']['home'][0] = float('nan')
    with pytest.raises(ValueError, match='finite'):
        station.initial_configuration(registry, 'home')
