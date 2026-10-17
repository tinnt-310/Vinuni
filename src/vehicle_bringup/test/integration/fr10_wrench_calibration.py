"""Known rigid-load checks, separate from controller and simulator ground truth."""
import numpy as np


def quaternion_rotation(xyzw):
    q = np.asarray(xyzw, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Invalid calibration orientation')
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def compose_rotation(q, axis, radians):
    x, y, z, w = q
    vector = np.zeros(3); vector[axis] = np.sin(radians/2)
    value = np.asarray([x,y,z])*np.cos(radians/2) + w*vector + np.cross([x,y,z], vector)
    return [*value, w*np.cos(radians/2)-np.dot([x,y,z], vector)]


def check_known_load(raw, measured_tcp_xyzw, mass, cog_sensor, force_tolerance=.1, torque_tolerance=.005):
    raw = np.asarray(raw, dtype=float)
    if raw.shape != (6,) or not np.isfinite(raw).all() or not np.isfinite(mass) or mass <= 0:
        raise ValueError('Invalid calibration sample')
    cog = np.asarray(cog_sensor, dtype=float)
    if cog.shape != (3,) or not np.isfinite(cog).all(): raise ValueError('Invalid tool CoG')
    # This model's sensor and TCP axes coincide; translation affects torque
    # origin, not gravity direction. All CoG coordinates are at sensor origin.
    force = quaternion_rotation(measured_tcp_xyzw).T @ [0,0,-9.80665*mass]
    expected = np.r_[force, np.cross(cog, force)]
    error = raw-expected
    force_error, torque_error = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
    return {'raw_native': raw.tolist(), 'predicted_native': expected.tolist(),
            'residual': error.tolist(), 'force_error_norm_n': force_error,
            'torque_error_norm_nm': torque_error,
            'pass': force_error <= force_tolerance and torque_error <= torque_tolerance}
