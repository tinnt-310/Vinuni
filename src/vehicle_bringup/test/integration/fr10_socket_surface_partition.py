#!/usr/bin/env python3
"""Exact CAD triangle surfaces for Gazebo ODE/FCL; no convex-filled cavities.

The moving connector uses small convex cells. The stationary socket uses ODE's
native triangle mesh. The contact depth band and protected rear are separate
collision objects, allowing the planner to protect the rear independently.
MuJoCo convex conversion is deliberately not claimed by this exporter.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy
from fr10_mating_geometry import DESCRIPTION, CANONICAL, load_stl, write_stl


def partition(model):
    source = model/'socket_mating.stl'
    reader = vtk.vtkSTLReader(); reader.SetFileName(str(source)); reader.Update()
    result = {}
    for name, sign in [('socket_contact_surface.stl', -1), ('socket_protected_rear.stl', 1)]:
        plane = vtk.vtkPlane(); plane.SetOrigin(0, 0, .018); plane.SetNormal(0, 0, sign)
        clip = vtk.vtkClipPolyData(); clip.SetInputConnection(reader.GetOutputPort())
        clip.SetClipFunction(plane); clip.Update()
        triangles = vtk.vtkTriangleFilter(); triangles.SetInputConnection(clip.GetOutputPort()); triangles.Update()
        poly = triangles.GetOutput()
        vertices = vtk_to_numpy(poly.GetPoints().GetData())
        faces = vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
        write_stl(model/name, vertices[faces])
        result[name] = {'triangles': len(faces), 'sha256': hashlib.sha256((model/name).read_bytes()).hexdigest()}
    original = load_stl(source)
    exported = np.concatenate([load_stl(model/name) for name in result])
    def area(mesh): return .5*np.linalg.norm(np.cross(mesh[:,1]-mesh[:,0], mesh[:,2]-mesh[:,0]), axis=1).sum()
    area_error = float(abs(area(exported)-area(original)))
    if area_error > 1e-8: raise ValueError(f'CAD surface partition lost/added area: {area_error}')
    manifest = {'canonical_sha256': hashlib.sha256(CANONICAL.read_bytes()).hexdigest(),
                'algorithm': 'exact triangle clipping at mating z=18 mm, without artificial caps',
                'gazebo_collision': 'ODE native trimesh', 'planning_collision': 'FCL triangle BVH',
                'mujoco_collision': 'PENDING verified convex geometry; ODE mesh is not equivalent',
                'max_added_functional_surface_error_m': 0., 'surface_area_error_m2': area_error,
                'assets': result, 'acceptance': 'GEOMETRY_ONLY; physical contact validation pending'}
    (model/'socket_collision_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--model', type=Path,
        default=DESCRIPTION/'models/matched_socket_v2')
    partition(parser.parse_args().model)
