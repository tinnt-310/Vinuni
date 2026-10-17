#!/usr/bin/env python3
"""Deterministic section measurements and matched female nose for supplied CAD.

The socket asset is read-only. Generated models are simulation mechanical
geometry, not electrical contacts or manufacturing drawings. Units: metres in
generated meshes, millimetres only for CAD measurements and JSON inputs.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

WORKSPACE = Path(__file__).resolve().parents[4]
DESCRIPTION = WORKSPACE / 'src/vehicle_description'
CANONICAL = DESCRIPTION / 'meshes/cong_sac/meshes/cong_sac.stl'
LEGACY = DESCRIPTION / 'meshes/jlr_ccs2_centered.stl'


def load_stl(path):
    with path.open('rb') as stream:
        stream.read(80)
        count = struct.unpack('<I', stream.read(4))[0]
        records = np.fromfile(stream, dtype=np.dtype([
            ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]), count=count)
    if len(records) != count:
        raise ValueError('Incomplete binary STL')
    return records['vertices'].astype(float)


def append_exact_shoulder_collision(collision, mesh_path):
    """Represent the existing rectangular shoulder exactly as a box.

    This avoids degenerate convex mesh/triangle distance solves; it does not
    approximate, inflate or change the mechanical CAD surface.
    """
    triangles = load_stl(mesh_path)
    vertices = np.unique(triangles.reshape(-1, 3), axis=0)
    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    if triangles.shape != (12, 3, 3) or vertices.shape != (8, 3) or np.any(upper <= lower):
        raise ValueError('Shoulder CAD is not a rectangular box')
    expected = np.array([[x,y,z] for x in (lower[0],upper[0])
                         for y in (lower[1],upper[1]) for z in (lower[2],upper[2])])
    if not np.array_equal(vertices, expected):
        raise ValueError('Shoulder CAD corners do not match the exact box')
    areas = np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],
                                   triangles[:,2]-triangles[:,0]), axis=1)*.5
    dimensions = upper-lower
    surface_area = 2*(dimensions[0]*dimensions[1]+dimensions[1]*dimensions[2]+dimensions[0]*dimensions[2])
    if not np.isclose(areas.sum(), surface_area, rtol=1e-12, atol=0.):
        raise ValueError('Shoulder CAD faces do not cover the box surface')
    ET.SubElement(collision, 'origin', xyz=' '.join(map(str, (lower+upper)*.5)))
    ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=' '.join(map(str, dimensions)))


def socket_in_mating_mm():
    raw = load_stl(CANONICAL)
    return raw[:, :, [0, 2, 1]] * [-1, 1, 1] + [0, -767.5, 6.5]


def section(triangles, depth):
    cut = triangles[(triangles[:, :, 2].min(axis=1) < depth) &
                    (triangles[:, :, 2].max(axis=1) > depth)]
    intersections = np.full((len(cut), 3, 2), np.nan)
    for edge, (first, second) in enumerate(((0, 1), (1, 2), (2, 0))):
        a, b = cut[:, first], cut[:, second]
        crossing = (a[:, 2] - depth) * (b[:, 2] - depth) < 0
        alpha = (depth - a[crossing, 2]) / (b[crossing, 2] - a[crossing, 2])
        intersections[crossing, edge] = (a[crossing] + alpha[:, None] * (b[crossing] - a[crossing]))[:, :2]
    return np.asarray([row[np.isfinite(row[:, 0])] for row in intersections
                       if np.isfinite(row[:, 0]).sum() == 2])


def components(segments):
    # Join exactly exported edges, at 0.001 mm numerical tolerance.
    parents, groups = {}, {}

    def root(key):
        parents.setdefault(key, key)
        if parents[key] != key:
            parents[key] = root(parents[key])
        return parents[key]

    for pair in segments:
        a, b = [tuple(np.round(point * 1000).astype(int)) for point in pair]
        parents[root(b)] = root(a)
    for pair in segments:
        key = root(tuple(np.round(pair[0] * 1000).astype(int)))
        groups.setdefault(key, []).append(pair)
    return [np.asarray(group) for group in groups.values()]


def circle_fit(segments):
    points = segments.reshape(-1, 2)
    center, _, _, _ = np.linalg.lstsq(
        np.column_stack((2 * points, np.ones(len(points)))), np.sum(points ** 2, axis=1), rcond=None)
    radius = math.sqrt(max(0, center[2] + center[:2] @ center[:2]))
    residual = np.max(np.abs(np.linalg.norm(points - center[:2], axis=1) - radius))
    return center[:2], radius, residual


def point_segment_distance(center, segments):
    a, b = segments[:, 0], segments[:, 1]
    delta = b - a
    alpha = np.clip(np.sum((center - a) * delta, axis=1) /
                    np.maximum(np.sum(delta * delta, axis=1), 1e-20), 0, 1)
    return float(np.min(np.linalg.norm(a + alpha[:, None] * delta - center, axis=1)))


def measure_socket():
    mesh = socket_in_mating_mm()
    groups = components(section(mesh, 20.0))
    pins, other = [], []
    for group in groups:
        center, radius, residual = circle_fit(group)
        if 1.3 < radius < 4.2 and residual < .05:
            pins.append({'center_mm': np.round(center, 4).tolist(),
                         'pin_radius_mm': float(np.max(np.linalg.norm(group.reshape(-1, 2) - center, axis=1))),
                         'fit_residual_mm': float(residual)})
        else:
            other.append(group)
    if len(pins) != 9:
        raise ValueError(f'Expected nine circular pins at 20 mm, measured {len(pins)}')
    for pin in pins:
        center = np.asarray(pin['center_mm'])
        pin['bore_inscribed_radius_mm'] = min(point_segment_distance(center, group) for group in other)
    # Verify every nose section against measured bores, not just one station.
    for depth in (1.0, 3.0, 5.0, 8.0, 12.0, 16.0, 20.0):
        for group in components(section(mesh, depth)):
            center, radius, residual = circle_fit(group)
            if 1.3 < radius < 4.2 and residual < .05:
                continue
            for pin in pins:
                pin['bore_inscribed_radius_mm'] = min(pin['bore_inscribed_radius_mm'],
                    point_segment_distance(np.asarray(pin['center_mm']), group))
    return {
        'schema': 2,
        'socket_sha256': hashlib.sha256(CANONICAL.read_bytes()).hexdigest(),
        'canonical_asset': str(CANONICAL.relative_to(WORKSPACE)),
        'raw_units': 'mm',
        'raw_to_mating_mm': {'matrix': [[-1, 0, 0], [0, 0, 1], [0, 1, 0]],
                             'translation': [0, -767.5, 6.5]},
        'mating_bounds_mm': [mesh.min(axis=(0, 1)).tolist(), mesh.max(axis=(0, 1)).tolist()],
        'pins': sorted(pins, key=lambda pin: (pin['center_mm'][1], pin['center_mm'][0])),
        'simulation_design': {'radial_clearance_mm': .25, 'nose_length_mm': 15.,
                              'lead_in_mm': 3., 'lead_in_angle_deg': 45.,
                              'pilot_recess_mm': 3., 'pilot_lead_in_mm': 1.5,
                              'tcp_connector_z_m': .039, 'body_cut_z_m': .020,
                              'seat_face_connector_z_m': .024,
                              'seat_width_mm': 80., 'seat_height_mm': 112., 'seat_thickness_mm': 4.},
        'limitations': ['simulation-only mass/contact calibration pending',
                        '15 mm seating is the redesigned shoulder datum, not legacy 5 mm completion'],
    }


def write_stl(path, triangles):
    path.parent.mkdir(parents=True, exist_ok=True)
    triangles = np.asarray(triangles, dtype=float)
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normal = cross / np.maximum(np.linalg.norm(cross, axis=1)[:, None], 1e-20)
    records = np.zeros(len(triangles), dtype=np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
    records['normal'], records['vertices'] = normal, triangles
    with path.open('wb') as stream:
        stream.write(b'FR10 matched simulation geometry; SI metres'.ljust(80, b' '))
        stream.write(struct.pack('<I', len(triangles)))
        records.tofile(stream)


def convex_triangles(vertices):
    vertices = np.asarray(vertices)
    hull = ConvexHull(vertices)
    triangles = []
    for face, equation in zip(hull.simplices, hull.equations):
        triangle = vertices[face]
        if np.dot(np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0]), equation[:3]) < 0:
            triangle = triangle[[0, 2, 1]]
        triangles.append(triangle)
    return np.asarray(triangles)


def annular_wedge(center, inner_radii, radii, levels, angle, next_angle):
    # Lofted convex cells preserve the hole, rather than convexifying the tube.
    points = []
    for inner, radius, height in zip(inner_radii, radii, levels):
        for r in (inner, radius):
            for theta in (angle, next_angle):
                points.append([center[0] + r * math.cos(theta), center[1] + r * math.sin(theta), height])
    return convex_triangles(points)


def clip_rear_body(path, cutoff):
    # vtkClipClosedSurface caps the retained manufacturer's body at the new
    # mechanical interface. This preserves wrist-side CAD at original scale.
    import vtk
    from vtk.util.numpy_support import vtk_to_numpy
    reader = vtk.vtkSTLReader(); reader.SetFileName(str(path)); reader.Update()
    plane = vtk.vtkPlane(); plane.SetOrigin(0, 0, cutoff); plane.SetNormal(0, 0, -1)
    planes = vtk.vtkPlaneCollection(); planes.AddItem(plane)
    clip = vtk.vtkClipClosedSurface(); clip.SetInputConnection(reader.GetOutputPort())
    clip.SetClippingPlanes(planes); clip.GenerateFacesOn(); clip.Update()
    triangular = vtk.vtkTriangleFilter(); triangular.SetInputConnection(clip.GetOutputPort()); triangular.Update()
    poly = triangular.GetOutput()
    points = vtk_to_numpy(poly.GetPoints().GetData())
    cells = vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    body = points[cells]
    if not len(body) or body[:, :, 2].max() > cutoff + 1e-6:
        raise ValueError('Wrist-side CAD clipping failed')
    return body


def generate(output, measurements):
    design = measurements['simulation_design']
    pieces, descriptions = [], []
    base = design['seat_face_connector_z_m']
    for index, pin in enumerate(measurements['pins']):
        required_inner = (pin['pin_radius_mm'] + design['radial_clearance_mm']) * .001
        outer = (pin['bore_inscribed_radius_mm'] - design['radial_clearance_mm']) * .001
        pilot = pin['pin_radius_mm'] < 2.
        length = (design['nose_length_mm'] - (design['pilot_recess_mm'] if pilot else 0)) * .001
        chamfer = (design['pilot_lead_in_mm'] if pilot else design['lead_in_mm']) * .001
        facets = max(16, math.ceil(math.pi / math.acos(1 - .000095 / outer)))
        inner = required_inner / math.cos(math.pi/facets)
        # The socket has male pins AND surrounding bores. Both sides need a
        # lead-in. Limit the requested 3 mm chamfer by the actual available
        # wall, preserving at least 0.15 mm at the tip. Recessed pilots engage
        # only after the larger collars have guided the initial capture.
        chamfer = min(chamfer, (outer-inner-.00015)/2)
        pin['actual_lead_in_mm'] = chamfer*1000
        pin['minimum_tip_wall_mm'] = (outer-inner-2*chamfer)*1000
        pin['collar_facets'] = facets
        pin['inner_vertex_radius_mm'] = inner * 1000
        if chamfer <= 0 or outer - chamfer <= inner + chamfer + .00010:
            raise ValueError(f'Lead-in leaves inadequate wall on pin {index}: {outer-inner-chamfer:.6g}')
        center = np.asarray(pin['center_mm']) * .001
        for segment in range(facets):
            angles = [2 * math.pi * segment/facets, 2 * math.pi * (segment+1)/facets]
            for band, (inner_radii, radii, levels) in enumerate((
                    ([inner, inner], [outer, outer], [base, base+length-chamfer]),
                    ([inner, inner+chamfer], [outer, outer-chamfer], [base+length-chamfer, base+length]))):
                triangle = annular_wedge(center, inner_radii, radii, levels, *angles)
                name = f'collar_{index}_{segment}_{band}.stl'
                write_stl(output / 'collision' / name, triangle)
                descriptions.append(name); pieces.append(triangle)
    nose = np.concatenate(pieces)
    write_stl(output / 'nose.stl', nose)
    body = clip_rear_body(LEGACY, design['body_cut_z_m'])
    write_stl(output / 'body.stl', body)
    # A separate shoulder stops on the socket's front rim, not a plate hidden
    # in the cavity. Keep it separate from non-contact body/camera collisions.
    dims = np.asarray([design['seat_width_mm'], design['seat_height_mm'], design['seat_thickness_mm']]) * .001
    vertices = [[x*dims[0]/2, y*dims[1]/2, base+z*dims[2]] for x in (-1, 1) for y in (-1, 1) for z in (-1, 0)]
    write_stl(output / 'seat.stl', convex_triangles(vertices))
    write_stl(output / 'socket_mating.stl', socket_in_mating_mm() * .001)
    measured = dict(measurements)
    measured['generated_collision_cells'] = descriptions
    measured['generated_assets'] = {
        str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(output.rglob('*.stl'))}
    measured['max_collar_facet_sagitta_mm'] = max(
        (pin['bore_inscribed_radius_mm']-.25)*(1-math.cos(math.pi/pin['collar_facets'])) for pin in measurements['pins'])
    def mass_properties(triangles, mass):
        triangles = np.asarray(triangles)
        volumes = np.einsum('ij,ij->i', triangles[:, 0],
                  np.cross(triangles[:, 1], triangles[:, 2]))/6
        volume = volumes.sum()
        cog = np.sum(volumes[:, None]*triangles.sum(axis=1)/4, axis=0)/volume
        size = triangles.max(axis=(0, 1))-triangles.min(axis=(0, 1))
        inertia = [mass*(size[(axis+1)%3]**2+size[(axis+2)%3]**2)/12 for axis in range(3)]
        return {'mass_kg': mass, 'cog_m': cog.tolist(), 'box_inertia_kg_m2': inertia,
                'signed_mesh_volume_m3': float(volume), 'inertia_method': 'bounding box approximation'}
    measured['simulation_mass'] = {
        'charging_connector_link': mass_properties(body, 2.8),
        'charging_mating_nose': mass_properties(nose, .15),
        'charging_mating_shoulder': mass_properties(convex_triangles(vertices), .05)}
    measured['tool_mass_kg'] = 3.0
    measured['tool_cog_sensor_m'] = (sum(
        prop['mass_kg']*np.asarray(prop['cog_m']) for prop in measured['simulation_mass'].values())/3.).tolist()
    # Generated XML is a mechanical model artifact, just like the STL cells.
    # Simulators and PlanningScene consume the same exact collision surfaces.
    # The rectangular shoulder uses its identical box, not a degenerate mesh.
    robot = ET.Element('robot', name='matched_charging_tool')
    prefix = 'package://vehicle_description/models/matched_socket_v2/'
    for link_name, visual, collisions in (
            ('charging_connector_link', 'body.stl', []),
            ('charging_mating_nose', 'nose.stl', ['collision/'+name for name in descriptions]),
            ('charging_mating_shoulder', 'seat.stl', ['seat.stl'])):
        link = ET.SubElement(robot, 'link', name=link_name)
        element = ET.SubElement(link, 'visual')
        ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh', filename=prefix+visual)
        material = ET.SubElement(element, 'material', name='matched_tool_dark')
        ET.SubElement(material, 'color', rgba='.14 .16 .18 1')
        if link_name == 'charging_connector_link':
            collision = ET.SubElement(link, 'collision', name='protected_rear_body')
            lower, upper = body.min(axis=(0, 1)), body.max(axis=(0, 1))
            upper[2] = min(upper[2], base - .001)
            ET.SubElement(collision, 'origin', xyz=' '.join(map(str, (lower+upper)/2)))
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=' '.join(map(str, upper-lower)))
        for index, mesh in enumerate(collisions):
            collision = ET.SubElement(link, 'collision', name=f'mating_surface_{index}')
            if link_name == 'charging_mating_shoulder':
                append_exact_shoulder_collision(collision, output/mesh)
            else:
                ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=prefix+mesh)
        properties = measured['simulation_mass'][link_name]
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', value=str(properties['mass_kg']))
        ET.SubElement(inertial, 'origin', xyz=' '.join(map(str, properties['cog_m'])))
        diagonal = properties['box_inertia_kg_m2']
        ET.SubElement(inertial, 'inertia', ixx=str(diagonal[0]), iyy=str(diagonal[1]), izz=str(diagonal[2]),
                      ixy='0', ixz='0', iyz='0')
        if link_name != 'charging_connector_link':
            joint = ET.SubElement(robot, 'joint', name='connector_to_'+link_name, type='fixed')
            ET.SubElement(joint, 'parent', link='charging_connector_link')
            ET.SubElement(joint, 'child', link=link_name)
        gazebo = ET.SubElement(robot, 'gazebo', reference=link_name)
        for tag, value in (('mu1', '.20'), ('mu2', '.20'),
                           ('kp', "${matched_ode_contact['stiffness_n_m']}"),
                           ('kd', "${matched_ode_contact['damping_n_s_m']}"),
                           ('maxVel', "${matched_ode_contact['max_correcting_velocity_m_s']}"), ('minDepth', '0')):
            ET.SubElement(gazebo, tag).text = value
    ET.indent(robot)
    ET.ElementTree(robot).write(output/'matched_connector.urdf', encoding='unicode', xml_declaration=True)
    (output / 'geometry.json').write_text(json.dumps(measured, indent=2) + '\n')
    return measured


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=DESCRIPTION/'models/matched_socket_v2')
    parser.add_argument('--measure-only', action='store_true')
    args = parser.parse_args()
    measurements = measure_socket()
    if args.measure_only:
        print(json.dumps(measurements, indent=2)); return
    args.output.mkdir(parents=True, exist_ok=True)
    result = generate(args.output, measurements)
    print(json.dumps({'output': str(args.output), 'pins': result['pins'],
                      'collision_cells': len(result['generated_collision_cells']),
                      'max_sagitta_mm': result['max_collar_facet_sagitta_mm']}, indent=2))


if __name__ == '__main__':
    main()
