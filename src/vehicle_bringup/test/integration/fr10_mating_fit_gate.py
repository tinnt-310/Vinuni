#!/usr/bin/env python3
"""CAD swept-fit gate using the original socket triangles, without physics."""
import argparse
import json
from pathlib import Path
import math

import numpy as np
import vtk
from fr10_mating_geometry import DESCRIPTION, CANONICAL, load_stl


def reader(path):
    source = vtk.vtkSTLReader(); source.SetFileName(str(path)); source.Update()
    return source


def collision_checker(socket, tool):
    checker = vtk.vtkCollisionDetectionFilter()
    checker.SetInputData(0, socket.GetOutput()); checker.SetInputData(1, tool.GetOutput())
    identity = vtk.vtkTransform(); moving = vtk.vtkTransform()
    checker.SetTransform(0, identity); checker.SetTransform(1, moving)
    checker.SetCollisionModeToFirstContact(); checker.GenerateScalarsOff()
    checker.SetBoxTolerance(0); checker.SetCellTolerance(0)
    return checker, moving


def intersections(pair, depth, tcp_z):
    checker, moving = pair
    socket_bounds = checker.GetInputDataObject(0, 0).GetBounds()
    tool_bounds = checker.GetInputDataObject(1, 0).GetBounds()
    if tool_bounds[5] + depth - tcp_z < socket_bounds[4] or tool_bounds[4] + depth - tcp_z > socket_bounds[5]:
        return 0
    moving.Identity(); moving.Translate(0, 0, depth - tcp_z)
    checker.Update()
    return checker.GetNumberOfContacts()


def run(model, output):
    geometry = json.loads((model / 'geometry.json').read_text())
    import hashlib
    canonical_hash = hashlib.sha256(CANONICAL.read_bytes()).hexdigest()
    if canonical_hash != geometry['socket_sha256']:
        raise ValueError('Canonical mesh hash changed since generation')
    design = geometry['simulation_design']
    tcp_z = design['tcp_connector_z_m']
    seated = design['nose_length_mm'] * .001
    socket = reader(model/'socket_mating.stl')
    nose, shoulder, body = [reader(model/name) for name in ('nose.stl', 'seat.stl', 'body.stl')]
    nose_check, seat_check, body_check = [collision_checker(socket, tool)
                                         for tool in (nose, shoulder, body)]
    rows = []
    for depth in np.linspace(-.020, seated-.0001, 141):
        contacts = intersections(nose_check, depth, tcp_z)
        protected = intersections(body_check, depth, tcp_z)
        seat = intersections(seat_check, depth, tcp_z)
        rows.append({'depth_m': float(depth), 'nose_intersections': contacts,
                     'shoulder_intersections': seat, 'body_intersections': protected})
    seat_contacts = intersections(seat_check, seated + .00005, tcp_z)
    cells = [load_stl(model/'collision'/name) for name in geometry['generated_collision_cells']]
    # Every convex cell must retain outward face winding and positive volume.
    volumes = [float(np.einsum('ij,ij->i', cell[:, 0],
                    np.cross(cell[:, 1], cell[:, 2])).sum()/6) for cell in cells]
    volume_valid = all(volume > 0 for volume in volumes)
    nose_bounds = load_stl(model/'nose.stl').reshape(-1, 3)
    result = {'canonical_sha256': canonical_hash,
              'seated_depth_m': seated, 'sweep': rows,
              'collision_cells': len(cells), 'positive_volume_cells': volume_valid,
              'nose_volume_m3': sum(volumes),
              'nose_bounds_m': [nose_bounds.min(axis=0).tolist(), nose_bounds.max(axis=0).tolist()],
              'shoulder_contacts_at_50um_preload': seat_contacts,
              'max_collar_facet_sagitta_mm': geometry['max_collar_facet_sagitta_mm'],
              'pass': bool(volume_valid and seat_contacts > 0 and
                           geometry['max_collar_facet_sagitta_mm'] <= .1 and
                           all(not any(row[key] for key in ('nose_intersections', 'shoulder_intersections',
                                                           'body_intersections')) for row in rows)),
              'limitations': ['surface intersection sweep, not force/dynamics acceptance',
                              'camera and mount swept validation remains in the complete robot PlanningScene']}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'sweep'}, indent=2))
    if not result['pass']:
        print('First interference:', next((row for row in rows if any(row[key] for key in
              ('nose_intersections', 'shoulder_intersections', 'body_intersections'))), None))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--model', type=Path, default=DESCRIPTION/'models/matched_socket_v2')
    args = parser.parse_args()
    raise SystemExit(run(args.model, args.output))
