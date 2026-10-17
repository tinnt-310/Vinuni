#!/usr/bin/env python3
"""Hole-preserving convex contact shells from original CAD surface facets.

Merge only coplanar triangles, triangulate polygons with their holes, and back
each remaining convex patch into the material by 50 um. This is a bounded
surface-contact representation, not a claim of full volumetric decomposition.
The original STL and the Gazebo triangle surface remain unchanged.
"""
import argparse
import hashlib
import heapq
import json
from pathlib import Path

import numpy as np
import shapely
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon

from fr10_mating_geometry import load_stl, write_stl, DESCRIPTION


def convex_partition(polygon):
    """Merge adjacent constrained triangles only when their union is convex.

    Shared-edge topology preserves bores; no convex hull across a hole is
    accepted. Larger planar patches also avoid duplicate contact constraints.
    """
    pieces = {i: triangle for i, triangle in enumerate(
        shapely.constrained_delaunay_triangles(polygon).geoms)}
    edges, adjacent, pending = {}, {i:set() for i in pieces}, []
    for index, piece in pieces.items():
        if not polygon.covers(piece): raise ValueError('Triangulation crossed a CAD hole')
        points = list(piece.exterior.coords)
        for a,b in zip(points,points[1:]):
            edge = tuple(sorted((a,b)))
            if edge in edges:
                other = edges[edge]
                adjacent[index].add(other); adjacent[other].add(index)
                heapq.heappush(pending,(-float(np.linalg.norm(np.asarray(a)-b)),other,index))
            else: edges[edge] = index
    next_id = len(pieces)
    while pending:
        _,a,b = heapq.heappop(pending)
        if a not in pieces or b not in pieces: continue
        joined = pieces[a].union(pieces[b])
        if joined.geom_type != 'Polygon' or joined.interiors: continue
        hull = joined.convex_hull
        if hull.area-joined.area > 1e-15: continue
        neighbors = (adjacent[a] | adjacent[b]) - {a,b}
        del pieces[a], pieces[b]
        pieces[next_id] = hull; adjacent[next_id] = neighbors
        for neighbor in neighbors:
            adjacent[neighbor].difference_update((a,b)); adjacent[neighbor].add(next_id)
            length = hull.boundary.intersection(pieces[neighbor].boundary).length
            if length > 0:
                heapq.heappush(pending,(-length,neighbor,next_id))
        del adjacent[a],adjacent[b]
        next_id += 1
    return list(pieces.values())


def convex_surface_cells(triangles, backing_m=0.00005, tolerance_m=0.0000002):
    triangles = np.asarray(triangles, dtype=float)
    if (triangles.ndim != 3 or triangles.shape[1:] != (3,3) or
            not np.all(np.isfinite(triangles)) or
            not np.isfinite(backing_m) or backing_m <= 0 or
            not np.isfinite(tolerance_m) or tolerance_m <= 0):
        raise ValueError('Finite triangles and positive backing/tolerance are required')
    cross = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    lengths = np.linalg.norm(cross, axis=1)
    usable = lengths > 1e-16
    triangles, cross, lengths = triangles[usable], cross[usable], lengths[usable]
    if not len(triangles): raise ValueError('No nondegenerate CAD facets')
    normals = cross/lengths[:, None]
    offsets = np.einsum('ij,ij->i', normals, triangles[:, 0])
    # Include winding in the key so opposite faces can never be merged.
    keys = np.column_stack((np.round(normals/1e-6), np.round(offsets/tolerance_m))).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    cells, max_plane_error, area_error = [], 0., 0.
    for group in range(inverse.max()+1):
        indices = np.flatnonzero(inverse == group)
        faces = triangles[indices]
        normal = normals[np.flatnonzero(inverse == group)[0]]
        origin = faces[0, 0]
        direction = faces[0, 1]-origin
        direction /= np.linalg.norm(direction)
        second = np.cross(normal, direction)
        basis = np.column_stack((direction, second))
        plane_error = float(np.max(np.abs((faces-origin) @ normal)))
        max_plane_error = max(max_plane_error, plane_error)
        if plane_error > tolerance_m:
            # Retain original facets if numerical plane grouping is ambiguous.
            surfaces = [(face, normals[index]) for face,index in zip(faces,indices)]
        else:
            polygons = [Polygon((face-origin) @ basis) for face in faces]
            joined = shapely.union_all(polygons)
            pieces = list(joined.geoms) if hasattr(joined, 'geoms') else [joined]
            patches = []
            for piece in pieces:
                if piece.is_empty or piece.area < 1e-16: continue
                simplified = piece.simplify(tolerance_m, preserve_topology=True)
                area_error += piece.symmetric_difference(simplified).area
                if not simplified.interiors and simplified.convex_hull.area-simplified.area < 1e-15:
                    patches.append(np.asarray(simplified.exterior.coords)[:-1])
                else:
                    # GEOS constrained Delaunay respects all original bores.
                    for patch in convex_partition(simplified):
                        patches.append(np.asarray(patch.exterior.coords)[:-1])
            surfaces = [(origin + patch @ basis.T, normal) for patch in patches]
        for surface, surface_normal in surfaces:
            vertices = np.concatenate((surface, surface-backing_m*surface_normal))
            if len(vertices) < 6: continue
            hull = ConvexHull(vertices)
            if hull.volume <= 0: raise ValueError('Degenerate contact shell')
            hull_faces = []
            for face, equation in zip(hull.simplices, hull.equations):
                tri = vertices[face]
                if np.dot(np.cross(tri[1]-tri[0], tri[2]-tri[0]), equation[:3]) < 0:
                    tri = tri[[0, 2, 1]]
                hull_faces.append(tri)
            cells.append(np.asarray(hull_faces))
    return cells, {'input_triangles': len(triangles), 'convex_cells': len(cells),
        'max_plane_group_error_m': max_plane_error, 'area_simplification_error_m2': float(area_error),
        'backing_m': backing_m, 'surface_tolerance_m': tolerance_m}


def generate(source, output):
    cells, metrics = convex_surface_cells(load_stl(source))
    output.mkdir(parents=True, exist_ok=True)
    items = []
    for index, cell in enumerate(cells):
        path = output/f'surface_{index:04d}.stl'
        write_stl(path, cell)
        items.append({'file': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    manifest = dict(metrics, algorithm='coplanar constrained convex partition surface shells',
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source=str(source.resolve()), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        shapely_version=shapely.__version__, cells=items,
        acceptance='PENDING independent swept fit and physical contact validation',
        limitations=['contact surface backing only; not full solid decomposition',
                     'maximum penetration must remain below the backing thickness'])
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({key:value for key,value in manifest.items() if key!='cells'}, indent=2), flush=True)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=DESCRIPTION/'models/matched_socket_v2/socket_contact_surface.stl')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    generate(args.source, args.output)
