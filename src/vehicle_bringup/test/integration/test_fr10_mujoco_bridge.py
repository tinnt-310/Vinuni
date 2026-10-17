#!/usr/bin/env python3
"""Deterministic tests for the MuJoCo adapter's canonical-model contract."""

import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("fr10_mujoco_bridge.py")
SPEC = importlib.util.spec_from_file_location("fr10_mujoco_bridge", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

def test_compiled_pd_gains_follow_shared_calibration():
    plant=MODULE.Fr10MujocoPlant()
    kp,kd=MODULE.simulation_pd_gains()
    assert np.array_equal(plant.kp,kp)
    assert np.array_equal(plant.kd,kd)
    assert np.array_equal(plant.model.actuator_gainprm[:,0],kp)

def test_invalid_shared_gain_profile_fails_closed(tmp_path,monkeypatch):
    import yaml
    config=tmp_path/'config';config.mkdir()
    monkeypatch.setattr(MODULE,'DESCRIPTION',tmp_path)
    for invalid in ({'kp':[1]*5,'kd':[1]*6,'ki':[0]*6},
                    {'kp':[1]*6,'kd':[1]*6,'ki':[1]*6},
                    {'kp':[float('nan')]*6,'kd':[1]*6,'ki':[0]*6}):
        (config/'fr10_simulation_actuators.yaml').write_text(yaml.safe_dump(invalid))
        try: MODULE.simulation_pd_gains()
        except ValueError: pass
        else: raise AssertionError('Malformed/shared PID profile must not be ignored')


def test_canonical_model_hold_and_contact():
    plant = MODULE.Fr10MujocoPlant()
    # Six arm DOFs plus one test-fixture compliance DOF.
    assert plant.model.nq == 7
    assert plant.model.nv == 7
    assert np.all(np.isfinite(plant.home_tcp_position))
    port_rotation = np.array(
        plant.data.xmat[plant.port_body_id]).reshape(3, 3)
    port_origin = np.array(plant.data.xpos[plant.port_body_id])
    assert np.allclose(
        port_origin + port_rotation @ MODULE.PORT_TCP_IN_MESH,
        plant.contact_tcp_position,
        atol=1e-9,
    )
    assert np.allclose(
        port_rotation @ np.array([0.0, 1.0, 0.0]),
        plant.insertion_axis_world,
        atol=1e-9,
    )
    assert plant.model.geom_contype[plant.port_visual_geom_id] == 0
    assert plant.model.geom_conaffinity[plant.port_visual_geom_id] == 0

    for _ in range(250):
        wrench = plant.step()
        assert np.all(np.isfinite(wrench))
    assert np.max(np.abs(plant.data.qpos[:6] - MODULE.HOME)) < 0.02

    plant.set_command(plant.cartesian_push_command(0.008))
    peak_axis_force = 0.0
    for _ in range(500):
        wrench = plant.step()
        assert np.all(np.isfinite(wrench))
        peak_axis_force = max(peak_axis_force, abs(float(wrench[2])))
    assert peak_axis_force > 1.0
    assert np.max(np.abs(plant.data.qpos[:6] - plant.command)) < 0.05


def test_invalid_commands_fail_closed():
    plant = MODULE.Fr10MujocoPlant()
    for command in (
            np.zeros(5),
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, np.nan])):
        try:
            plant.set_command(command)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid MuJoCo command was accepted")


def test_split_step_matches_reference_free_space_dynamics():
    plant = MODULE.Fr10MujocoPlant()
    reference = MODULE.mujoco.MjData(plant.model)
    reference.qpos[:] = plant.data.qpos
    reference.ctrl[:] = plant.data.ctrl
    for _ in range(20):
        for _ in range(8):
            MODULE.mujoco.mj_forward(plant.model, reference)
            reference.qfrc_applied[:] = 0.
            reference.qfrc_applied[:6] = reference.qfrc_bias[:6]
            MODULE.mujoco.mj_step(plant.model, reference)
        plant.step()
        assert np.max(np.abs(reference.qpos-plant.data.qpos)) < 1e-10
        assert abs(reference.time-plant.data.time) < 1e-12
