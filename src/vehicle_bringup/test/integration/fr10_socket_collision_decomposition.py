#!/usr/bin/env python3
"""Convex socket mating surfaces from canonical CAD, not a filled proxy.

Only the contact/braking depth band is decomposed. The original mesh stays
unchanged; back-of-socket CAD is protected outside the insertion envelope.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path

import coacd
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy

from fr10_mating_geometry import DESCRIPTION, CANONICAL, write_stl


def decompose(model, output):
    reader = vtk.vtkSTLReader(); reader.SetFileName(str(model/'socket_mating.stl')); reader.Update()
    # VTK's contour predicates operate more reliably in CAD millimetres.
    scale = vtk.vtkTransform(); scale.Scale(1000, 1000, 1000)
    scaled = vtk.vtkTransformPolyDataFilter(); scaled.SetInputConnection(reader.GetOutputPort())
    scaled.SetTransform(scale); scaled.Update()
    plane = vtk.vtkPlane(); plane.SetOrigin(0, 0, 18); plane.SetNormal(0, 0, -1)
    planes = vtk.vtkPlaneCollection(); planes.AddItem(plane)
    clip = vtk.vtkClipClosedSurface(); clip.SetInputConnection(scaled.GetOutputPort())
    clip.SetClippingPlanes(planes); clip.GenerateFacesOn(); clip.Update()
    boundary = vtk.vtkFeatureEdges(); boundary.SetInputConnection(clip.GetOutputPort())
    boundary.BoundaryEdgesOn(); boundary.NonManifoldEdgesOff(); boundary.FeatureEdgesOff(); boundary.ManifoldEdgesOff()
    boundary.Update()
    if boundary.GetOutput().GetNumberOfPoints():
        points = vtk_to_numpy(boundary.GetOutput().GetPoints().GetData())
        if np.max(np.abs(points[:, 2]-18)) > 1e-5:
            raise ValueError('Open edge on a functional CAD surface: refuse to fill it')
    # Complete only new planar rear caps, outside the validated mating band.
    fill = vtk.vtkFillHolesFilter(); fill.SetInputConnection(clip.GetOutputPort()); fill.SetHoleSize(100); fill.Update()
    triangle = vtk.vtkTriangleFilter(); triangle.SetInputConnection(fill.GetOutputPort()); triangle.Update()
    clean = vtk.vtkCleanPolyData(); clean.SetInputConnection(triangle.GetOutputPort())
    clean.ToleranceIsAbsoluteOn(); clean.SetAbsoluteTolerance(1e-5); clean.Update()
    edges = vtk.vtkFeatureEdges(); edges.SetInputConnection(clean.GetOutputPort())
    edges.BoundaryEdgesOn(); edges.NonManifoldEdgesOn(); edges.FeatureEdgesOff(); edges.ManifoldEdgesOff(); edges.Update()
    if edges.GetOutput().GetNumberOfCells():
        raise ValueError('Mating depth CAD is non-manifold: refuse voxel remeshing at unverified accuracy')
    poly = clean.GetOutput()
    vertices = vtk_to_numpy(poly.GetPoints().GetData()).astype(float) * .001
    faces = vtk_to_numpy(poly.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    # Separate disconnected solids to make hole-preserving decomposition smaller
    # and deterministic. Pins are already convex; housing/core bores are not.
    parents = np.arange(len(vertices))
    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]; index = parents[index]
        return index
    for face in faces:
        for index in face[1:]: parents[root(index)] = root(face[0])
    groups = {}
    for face in faces: groups.setdefault(root(face[0]), []).append(face)
    params = dict(threshold=.00005, real_metric=True, preprocess_mode='off',
                  resolution=5000, mcts_nodes=10, mcts_iterations=100,
                  mcts_max_depth=3, merge=False, decimate=False, seed=0)
    output.mkdir(parents=True, exist_ok=True)
    cells = []
    for component, group in enumerate(groups.values()):
        group = np.asarray(group)
        indices, inverse = np.unique(group, return_inverse=True)
        print(f'Component {component+1}/{len(groups)}: {len(group)} CAD triangles', flush=True)
        parts = coacd.run_coacd(coacd.Mesh(vertices[indices], inverse.reshape(-1, 3)), **params)
        for points, part_faces in parts:
            name = f'socket_cell_{len(cells):04d}.stl'
            write_stl(output/name, np.asarray(points)[np.asarray(part_faces)])
            cells.append({'file': name, 'component': component,
                          'vertices': len(points), 'faces': len(part_faces),
                          'sha256': hashlib.sha256((output/name).read_bytes()).hexdigest()})
        # Preserve expensive component work and expose resource/part counts.
        (output/'progress.json').write_text(json.dumps({
            'completed_components': component+1, 'total_components': len(groups),
            'parameters': params, 'cells': cells}, indent=2)+'\n')
    manifest = {'algorithm': 'CoACD', 'version': importlib.metadata.version('coacd'),
                'canonical_sha256': hashlib.sha256(CANONICAL.read_bytes()).hexdigest(),
                'parameters': params, 'mating_depth_band_m': [0, .018],
                'repair': 'clip/cap only on z=18 mm rear plane; vertex merge 10 nm; no voxel remeshing',
                'cells': cells, 'acceptance': 'PENDING independent clearance/swept contact validation'}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({'parts': len(cells), 'components': len(groups), 'output': str(output)}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, default=DESCRIPTION/'models/matched_socket_v2')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    decompose(args.model, args.output or args.model/'socket_collision')
