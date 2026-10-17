"""Check CAD hole preservation and the ambiguous coplanarity fallback."""
import numpy as np
from shapely.geometry import Polygon, Point
from shapely import union_all
from fr10_socket_convex_surfaces import convex_surface_cells


def test_ring_keeps_open_mouth_and_backs_into_material():
    outer = np.array([[-2,-2,0],[2,-2,0],[2,2,0],[-2,2,0]],dtype=float)*.01
    inner = outer*.5
    faces = []
    for i in range(4):
        j=(i+1)%4
        faces.extend([[outer[i],outer[j],inner[j]],[outer[i],inner[j],inner[i]]])
    cells, _ = convex_surface_cells(faces)
    assert len(cells) < len(faces)  # Fewer contact constraints, unchanged holes.
    footprints=[]
    for cell in cells:
        vertices=np.unique(cell.reshape(-1,3),axis=0)
        assert vertices[:,2].min() >= -.000050000001
        assert vertices[:,2].max() <= 1e-12
        footprint=Polygon(vertices[:,:2]).convex_hull
        assert not footprint.covers(Point(0,0))
        footprints.append(footprint)
    expected=Polygon(outer[:,:2],holes=[inner[:,:2]])
    assert union_all(footprints).symmetric_difference(expected).area < 1e-14


def test_ambiguous_plane_group_preserves_original_facet_vertices():
    first=np.array([[0,0,0],[10,0,0],[0,10,0]],dtype=float)
    second=np.array([[0,0,0],[10,0,0],[0,10,1e-6]],dtype=float)
    cells,metrics=convex_surface_cells([first,second])
    assert metrics['max_plane_group_error_m'] > metrics['surface_tolerance_m']
    assert len(cells)==2
    for original,cell in zip([first,second],cells):
        for vertex in original:
            assert np.min(np.linalg.norm(cell.reshape(-1,3)-vertex,axis=1)) < 1e-12
