import numpy as np
import pytest
from fr10_wrench_calibration import quaternion_rotation, compose_rotation, check_known_load


def test_known_load_and_contact_not_conflated():
    q = compose_rotation([0,0,0,1], 0, .4)
    force = quaternion_rotation(q).T @ [0,0,-29.41995]
    cog = [.01,-.02,.03]
    raw = np.r_[force, np.cross(cog, force)]
    assert check_known_load(raw, q, 3., cog)['pass']
    raw[0] += 2.
    assert not check_known_load(raw, q, 3., cog)['pass']


def test_bad_calibration_fails_closed():
    with pytest.raises(ValueError): check_known_load([0]*6, [0]*4, 3., [0]*3)
    with pytest.raises(ValueError): check_known_load([np.nan]*6, [0,0,0,1], 3., [0]*3)


def test_local_rotation_composition():
    q = compose_rotation([0,0,0,1], 1, np.pi/2)
    assert np.allclose(quaternion_rotation(q) @ [0,0,1], [1,0,0], atol=1e-12)
