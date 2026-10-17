"""Numerical contact recovery must carry the seating load before depth expiry."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT=Path(__file__).resolve().parents[4]
DESCRIPTION=ROOT/'src/vehicle_description'
SPEC=importlib.util.spec_from_file_location('station_contact',ROOT/'src/vehicle_bringup/launch/charging_station.py')
STATION=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(STATION)


def test_contact_velocity_cap_can_carry_five_newtons_before_depth_gate(tmp_path):
    model=DESCRIPTION/'models/matched_socket_v2'
    station=STATION.select_station(ROOT/'src/vehicle_bringup/config/charging_stations.yaml',
                                  model/'geometry.json','front_mid','<robot/>',False)
    world=tmp_path/'contact.world';STATION.write_world(station,model,world)
    root=ET.parse(world)
    cap=float(root.find('.//physics/ode/constraints/contact_max_correcting_vel').text)
    surfaces=root.findall(".//model[@name='matched_charging_station']/link/collision/surface/contact/ode")
    calibration=yaml.safe_load((DESCRIPTION/'config/fr10_simulation_actuators.yaml').read_text())
    # Gazebo adds the two100 Ns/m dampers. At rest the cap must not turn
    # the5 N seating objective into an impossible force-regulation request.
    assert cap*200. > 5.
    assert cap==calibration['matched_gazebo_contact']['max_correcting_velocity_m_s']
    for surface in surfaces:
        assert float(surface.find('max_vel').text)==cap
        assert float(surface.find('kp').text)==20000.
        assert float(surface.find('kd').text)==100.
