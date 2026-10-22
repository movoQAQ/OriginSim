#!/usr/bin/env python3
"""Sample the static collision surfaces of the hxy RMUC SDF into a world-frame PCD.

This supplies the missing 2026 static reference. It never rewrites a PGM/YAML.
Supports the binary STL meshes and collision boxes used by this scene; rejects
other geometries and relative frames rather than silently producing a partial map.
Requires numpy and scipy (already used by the workspace's map tools).
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def pose(element):
    transform = np.eye(4)
    tag = element.find('pose')
    if tag is None:
        return transform
    if tag.get('relative_to') or tag.get('frame'):
        raise ValueError('Relative SDF frames require explicit resolution')
    values = np.fromstring(tag.text, sep=' ')
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError('Expected six finite pose values')
    transform[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=0.04)
    args = parser.parse_args()
    if not 0.01 <= args.spacing <= 0.1:
        raise ValueError('Reference spacing must be between 0.01 and 0.1 m')
    resources, triangles = {}, []

    def remember(path):
        resources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()

    def resource(uri):
        if not uri.startswith('model://'):
            raise ValueError('Expected model:// resource: ' + uri)
        path = (args.models / uri[len('model://'):]).resolve()
        if not path.is_relative_to(args.models.resolve()):
            raise ValueError('Resource escapes the models directory')
        return path

    def collect(model, parent):
        transform = parent @ pose(model)
        include = model.find('include')
        if include is not None:
            model_file = resource(include.findtext('uri')) / 'model.sdf'
            remember(model_file)
            collect(ET.parse(model_file).getroot().find('model'), transform @ pose(include))
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                world_from_shape = transform @ pose(link) @ pose(collision)
                geometry = collision.find('geometry')
                mesh, box = geometry.find('mesh'), geometry.find('box')
                if mesh is not None:
                    path = resource(mesh.findtext('uri'))
                    remember(path)
                    data = path.read_bytes()
                    count = struct.unpack_from('<I', data, 80)[0]
                    if len(data) != 84 + count * 50:
                        raise ValueError('Expected binary STL: ' + str(path))
                    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
                    vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].astype(float)
                    scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
                    if len(scale) != 3 or not np.isfinite(scale).all() or np.any(scale <= 0):
                        raise ValueError('Invalid mesh scale')
                    vertices *= scale
                elif box is not None:
                    size = np.fromstring(box.findtext('size'), sep=' ')
                    if len(size) != 3 or not np.isfinite(size).all() or np.any(size <= 0):
                        raise ValueError('Invalid collision box')
                    corners = np.array([[x, y, z] for x in [-.5, .5] for y in [-.5, .5] for z in [-.5, .5]]) * size
                    faces = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
                             [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                             [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
                    vertices = corners[faces]
                else:
                    raise ValueError('Unsupported collision geometry in ' + model.get('name'))
                triangles.append(vertices @ world_from_shape[:3, :3].T + world_from_shape[:3, 3])

    remember(args.world)
    world = ET.parse(args.world).getroot().find('world')
    for model in world.findall('model'):
        if model.findtext('static') != 'true':
            raise ValueError('Reference must contain static world models only')
        collect(model, np.eye(4))
    faces = np.concatenate(triangles)
    if not np.isfinite(faces).all():
        raise ValueError('Nonfinite collision mesh')
    areas = np.linalg.norm(np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0]), axis=1) / 2
    total_area = areas.sum()
    rng = np.random.default_rng(2026)
    points = [faces.reshape(-1, 3)]
    remaining = int(np.ceil(3 * total_area / args.spacing**2))
    while remaining:
        count = min(remaining, 200000)
        selected = faces[rng.choice(len(faces), count, p=areas / total_area)]
        u, v = rng.random((2, count))
        root = np.sqrt(u)
        points.append((1 - root[:, None]) * selected[:, 0] +
                      (root * (1 - v))[:, None] * selected[:, 1] +
                      (root * v)[:, None] * selected[:, 2])
        remaining -= count
    xyz = np.concatenate(points)
    # Keep the complete field mesh and the relevant lower part of tall perimeter walls.
    xyz = xyz[(xyz[:, 2] >= -0.1) & (xyz[:, 2] <= 4.0)]
    _, indices = np.unique(np.floor(xyz / args.spacing).astype(np.int32), axis=0, return_index=True)
    xyz = xyz[np.sort(indices)].astype('<f4')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('wb') as stream:
        stream.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                      f'COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                      f'POINTS {len(xyz)}\nDATA binary\n').encode())
        stream.write(xyz.tobytes())
    metadata = dict(frame='map (Gazebo world)', source='static SDF collision surfaces',
                    sources_sha256=resources, spacing_m=args.spacing, seed=2026,
                    z_range_m=[-0.1, 4.0], points=len(xyz),
                    bounds_m=[xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()],
                    pcd_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                    modifies_occupancy_map=False)
    args.output.with_suffix('.metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
