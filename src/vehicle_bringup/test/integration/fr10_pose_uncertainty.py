"""Inject perception error into one input pose; never change physical geometry."""
import numpy as np
from scipy.spatial.transform import Rotation


def input_socket_pose(position, orientation_xyzw, lateral_mm=(0., 0.), rotation_deg=(0., 0., 0.),
                      *, axial_mm=0., max_position_mm=2., max_rotation_deg=2.):
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(orientation_xyzw, dtype=float)
    lateral = np.asarray(lateral_mm, dtype=float)
    rotation = np.asarray(rotation_deg, dtype=float)
    if position.shape != (3,) or quaternion.shape != (4,) or lateral.shape != (2,) or rotation.shape != (3,):
        raise ValueError('Invalid pose uncertainty dimensions')
    if not all(np.isfinite(value).all() for value in (position, quaternion, lateral, rotation)):
        raise ValueError('Pose uncertainty must be finite')
    if not np.isfinite(axial_mm):
        raise ValueError('Axial pose uncertainty must be finite')
    if any(isinstance(v, bool) or not np.isfinite(v) or not 0 < v <= 5.
           for v in (max_position_mm, max_rotation_deg)):
        raise ValueError('Pose-error envelope must be finite, positive and at most 5 mm / 5 degrees')
    translation = np.r_[lateral, axial_mm]
    if np.linalg.norm(translation) > max_position_mm + 1e-12 or np.linalg.norm(rotation) > max_rotation_deg + 1e-12:
        raise ValueError(f'Requested matrix bounds are translation norm <={max_position_mm:g} mm '
                         f'and rotation norm <={max_rotation_deg:g} degrees')
    if np.linalg.norm(quaternion) < 1e-6:
        raise ValueError('Invalid input quaternion')
    frame = Rotation.from_quat(quaternion)
    corrected_position = position + frame.apply(translation * .001)
    corrected_orientation = (frame * Rotation.from_rotvec(np.deg2rad(rotation))).as_quat()
    return corrected_position.tolist(), corrected_orientation.tolist()
