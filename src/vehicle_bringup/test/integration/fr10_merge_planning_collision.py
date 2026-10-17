#!/usr/bin/env python3
"""Concatenate cell facets for FCL; no hull, welding or simplification."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

PREFIX = 'package://vehicle_description/models/matched_socket_v2/'


def generate(directory):
    directory = Path(directory)
    urdf = directory/'matched_connector.urdf'
    link = ET.parse(urdf).find("link[@name='charging_mating_nose']")
    records, sources = [], []
    for collision in link.findall('collision'):
        mesh = collision.find('geometry/mesh')
        if collision.find('origin') is not None or mesh is None or mesh.get('scale'):
            raise ValueError('Only identity cell transforms are supported')
        uri = mesh.get('filename')
        if not uri.startswith(PREFIX): raise ValueError('Unexpected cell URI')
        data = (directory/uri[len(PREFIX):]).read_bytes()
        count = struct.unpack_from('<I',data,80)[0]
        if len(data)!=84+50*count: raise ValueError('Expected binary STL facets')
        records.append(data[84:])
        sources.append(dict(uri=uri,sha256=hashlib.sha256(data).hexdigest(),facets=count))
    facets = sum(s['facets'] for s in sources)
    data = b'FCL exact concatenated collision cell facets'.ljust(80,b'\0')+struct.pack('<I',facets)+b''.join(records)
    target = directory/'collision'/'mating_nose_bvh.stl'
    target.write_bytes(data)
    manifest = dict(schema=1,operation='concatenate_binary_facets_without_geometry_changes',
        link='charging_mating_nose',sources=sources,facets=facets,
        mesh_uri=PREFIX+'collision/mating_nose_bvh.stl',sha256=hashlib.sha256(data).hexdigest())
    (directory/'planning_collision.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return dict(cells=len(sources),facets=facets,sha256=manifest['sha256'])


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    print(json.dumps(generate(parser.parse_args().directory)))
