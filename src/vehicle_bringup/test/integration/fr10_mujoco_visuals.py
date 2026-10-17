"""Retain original URDF visuals for the read-only MuJoCo viewer."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


def write_triangle_obj(triangles, destination):
    """Lossless indexed triangles; avoids MuJoCo's 200k-face STL limit."""
    vertices, indices = np.unique(np.asarray(triangles).reshape(-1, 3), axis=0, return_inverse=True)
    with Path(destination).open('w') as output:
        output.write('# Original CAD triangles; no decimation or convexification\n')
        np.savetxt(output, vertices, fmt='v %.17g %.17g %.17g')
        np.savetxt(output, indices.reshape(-1, 3) + 1, fmt='f %d %d %d')


def camera_dae_to_obj(source, destination):
    """Convert the supplied D435's untransformed triangle mesh, in metres."""
    root = ET.parse(source).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    unit = root.find('c:asset/c:unit', ns)
    axis = root.find('c:asset/c:up_axis', ns)
    if unit is None or float(unit.get('meter')) != 1. or axis is None or axis.text != 'Z_UP':
        raise ValueError('Unsupported camera CAD units or up axis')
    for kind in ('matrix', 'translate', 'rotate', 'scale', 'instance_node', 'instance_controller'):
        if root.findall('.//c:library_visual_scenes//c:' + kind, ns):
            raise ValueError('Camera CAD requires scene transforms; refusing an inaccurate conversion')
    triangles = []
    geometries = {g.get('id'): g for g in root.findall('c:library_geometries/c:geometry', ns)}
    for instance in root.findall('.//c:library_visual_scenes//c:instance_geometry', ns):
        mesh = geometries[instance.get('url').lstrip('#')].find('c:mesh', ns)
        if any(mesh.findall('c:' + kind, ns) for kind in ('polylist', 'polygons', 'trifans', 'tristrips')):
            raise ValueError('Camera CAD must contain triangles')
        for group in mesh.findall('c:triangles', ns):
            inputs = group.findall('c:input', ns)
            vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            vertices = mesh.find("c:vertices[@id='" + vertex.get('source')[1:] + "']", ns)
            position = next(i for i in vertices.findall('c:input', ns) if i.get('semantic') == 'POSITION')
            source_node = mesh.find("c:source[@id='" + position.get('source')[1:] + "']", ns)
            points = np.fromstring(source_node.find('c:float_array', ns).text, sep=' ').reshape(-1, 3)
            stride = max(int(i.get('offset', '0')) for i in inputs) + 1
            indices = np.fromstring(group.find('c:p', ns).text, sep=' ', dtype=int).reshape(-1, stride)
            indices = indices[:, int(vertex.get('offset', '0'))].reshape(-1, 3)
            if len(indices) != int(group.get('count')) or not np.isfinite(points).all():
                raise ValueError('Invalid camera CAD triangles')
            triangles.extend(points[indices])
    if not triangles:
        raise ValueError('Camera CAD has no visible triangles')
    write_triangle_obj(triangles, destination)


def retain_urdf_visuals(robot, directory):
    from ament_index_python.packages import get_package_share_directory
    extension = robot.find('mujoco')
    if extension is None:
        extension = ET.SubElement(robot, 'mujoco')
    compiler = extension.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(extension, 'compiler')
    compiler.set('discardvisual', 'false')
    for link in robot.findall('link'):
        for index, visual in enumerate(link.findall('visual')):
            visual.set('name', f'visual__{link.get("name")}__{index}')
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            uri = mesh.get('filename')
            if uri.startswith('package://'):
                package, relative = uri[len('package://'):].split('/', 1)
                uri = str(Path(get_package_share_directory(package)) / relative)
                mesh.set('filename', uri)
            if Path(uri).suffix.lower() == '.dae':
                destination = Path(directory) / (link.get('name') + '_visual.obj')
                camera_dae_to_obj(uri, destination)
                mesh.set('filename', str(destination))
            elif Path(uri).suffix.lower() == '.stl':
                with Path(uri).open('rb') as source:
                    source.seek(80)
                    count = struct.unpack('<I', source.read(4))[0]
                if count <= 200000:
                    continue
                from fr10_mating_geometry import load_stl
                destination = Path(directory) / (link.get('name') + '_visual.obj')
                write_triangle_obj(load_stl(Path(uri)), destination)
                mesh.set('filename', str(destination))
