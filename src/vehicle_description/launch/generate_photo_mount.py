"""Generate nominal photo-reference bracket meshes; no measured CAD is changed."""
from pathlib import Path
import math
import struct

import numpy as np


def rounded_rectangle(hx, hy, radius, count=96):
    points = []
    for i in range(count):
        angle = 2 * math.pi * i / count
        # Radial intersection with a rounded rectangle, using a monotone SDF.
        direction = np.array([math.cos(angle), math.sin(angle)])
        low, high = 0., math.hypot(hx, hy)
        for _ in range(50):
            mid = (low + high) / 2
            q = abs(direction * mid) - [hx - radius, hy - radius]
            sdf = np.linalg.norm(np.maximum(q, 0)) + min(max(q), 0) - radius
            if sdf > 0: high = mid
            else: low = mid
        points.append(direction * ((low + high) / 2))
    return np.asarray(points)


def prism(points, low, high):
    n = len(points)
    bottom = np.column_stack((points, np.full(n, low)))
    top = np.column_stack((points, np.full(n, high)))
    triangles = []
    for i in range(n):
        j = (i + 1) % n
        triangles.extend(([bottom[i], bottom[j], top[j]], [bottom[i], top[j], top[i]]))
        triangles.extend(([[0, 0, low], bottom[j], bottom[i]], [[0, 0, high], top[i], top[j]]))
    return np.asarray(triangles)


def collar_cells(inner, outer, low, high):
    cells = []
    for i in range(len(inner)):
        j = (i + 1) % len(inner)
        a = np.array([[*inner[i], low], [*outer[i], low], [*outer[j], low], [*inner[j], low],
                      [*inner[i], high], [*outer[i], high], [*outer[j], high], [*inner[j], high]])
        faces = ((0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
                 (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
                 (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7))
        cells.append(a[np.array(faces)])
    return cells


def save(path, triangles):
    with path.open('wb') as output:
        output.write(b'Nominal FR10 CCS2 DC / D455 photo-reference mount, metres'.ljust(80, b'\0'))
        output.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
            norm = np.linalg.norm(normal)
            if norm: normal /= norm
            output.write(struct.pack('<12fH', *normal, *triangle.flatten(), 0))


def generate(directory):
    directory.mkdir(parents=True, exist_ok=True)
    save(directory/'dc_housing.stl', prism(rounded_rectangle(.045, .0675, .020), .075, .175))
    # Split white clamp: original body remains visible through the opening.
    inner = rounded_rectangle(.046, .0685, .021)
    outer = rounded_rectangle(.056, .0785, .031)
    cells = collar_cells(inner, outer, .080, .115)
    for side, sign in (('left', -1), ('right', 1)):
        selected = [cell for i, cell in enumerate(cells) if sign * outer[i, 0] >= -1e-12]
        save(directory/f'clamp_{side}.stl', np.concatenate(selected))
    save(directory/'rear_pad.stl', prism(rounded_rectangle(.056, .0785, .031), .070, .080))
    save(directory/'d455_fascia.stl', prism(rounded_rectangle(.0605, .0125, .012), .04105, .04135))


if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[1]/'meshes/fr10_photo_mount')
