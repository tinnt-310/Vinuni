"""Coalesce identical cell facets for FCL; physical description stays canonical."""
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

PREFIX = 'package://vehicle_description/models/matched_socket_v2/'


def planning_collision_description(description, model_path):
    robot = ET.fromstring(description)
    link = robot.find("link[@name='charging_mating_nose']")
    if link is None: return description
    directory = Path(model_path)
    manifest = json.loads((directory/'planning_collision.json').read_text())
    if manifest['schema']!=1 or manifest['operation']!='concatenate_binary_facets_without_geometry_changes':
        raise ValueError('Unsupported planning collision manifest')
    collisions = link.findall('collision')
    if len(collisions)!=len(manifest['sources']) or not collisions:
        raise ValueError('Physical/planning collision cell count differs')
    records=[]
    for collision,source in zip(collisions,manifest['sources']):
        mesh=collision.find('geometry/mesh')
        filename=mesh.get('filename','') if mesh is not None else ''
        canonical=directory/source['uri'][len(PREFIX):]
        same_file=(filename.startswith('file://') and
                   Path(filename[len('file://'):]).resolve()==canonical.resolve())
        if (mesh is None or collision.find('origin') is not None or mesh.get('scale') or
            (filename!=source['uri'] and not same_file) or not source['uri'].startswith(PREFIX)):
            raise ValueError('Physical/planning collision cell transform differs')
        data=(directory/source['uri'][len(PREFIX):]).read_bytes()
        if (hashlib.sha256(data).hexdigest()!=source['sha256'] or
            struct.unpack_from('<I',data,80)[0]!=source['facets'] or
            len(data)!=84+50*source['facets']):
            raise ValueError('Physical collision cell changed; regenerate planning asset')
        records.append(data[84:])
    if not manifest['mesh_uri'].startswith(PREFIX): raise ValueError('Unexpected planning URI')
    merged=(directory/manifest['mesh_uri'][len(PREFIX):]).read_bytes()
    if (hashlib.sha256(merged).hexdigest()!=manifest['sha256'] or
        len(merged)!=84+50*manifest['facets'] or merged[84:]!=b''.join(records) or
        struct.unpack_from('<I',merged,80)[0]!=sum(s['facets'] for s in manifest['sources'])):
        raise ValueError('Planning mesh facets differ from canonical collision cells')
    for collision in collisions: link.remove(collision)
    collision=ET.SubElement(link,'collision',name='exact_cell_facets_bvh')
    ET.SubElement(ET.SubElement(collision,'geometry'),'mesh',filename=manifest['mesh_uri'])
    return ET.tostring(robot,encoding='unicode')
