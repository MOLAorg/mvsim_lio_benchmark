#!/usr/bin/env python3
"""Clean reference mesh of the Newer College Quad for ray casting.

The 1 cm Quad mesh of the dataset is a surface reconstruction of the TLS
cloud: besides the scanned surfaces it closes holes and extrapolates where
nothing was scanned. Only faces whose three vertices lie within --max-dist of
a TLS point, and whose edges are shorter than --max-edge, are kept.

Usage: prep_mesh.py <mesh.ply> <tls_cloud.ply> <out.ply>
"""
import argparse
import re
import sys

import numpy as np
from scipy.spatial import cKDTree

PLY_TYPES = {'char': 'i1', 'uchar': 'u1', 'short': 'i2', 'ushort': 'u2', 'int': 'i4',
             'uint': 'u4', 'float': 'f4', 'double': 'f8'}


def read_ply(path):
    """Vertices (N, 3) float64 and triangles (M, 3) int64 of a binary little-endian PLY."""
    with open(path, 'rb') as fh:
        header = []
        while True:
            line = fh.readline().decode('ascii').strip()
            header.append(line)
            if line == 'end_header':
                break
        offset = fh.tell()
    elements = []
    for line in header:
        if line.startswith('element'):
            _, name, count = line.split()
            elements.append([name, int(count), []])
        elif line.startswith('property list'):
            _, _, ct, it, name = line.split()
            elements[-1][2].append(('list', ct, it, name))
        elif line.startswith('property'):
            _, t, name = line.split()
            elements[-1][2].append((t, name))
    raw = np.memmap(path, dtype='u1', mode='r', offset=offset)
    pos = 0
    verts = None
    faces = None
    for name, count, props in elements:
        if name == 'vertex':
            dt = np.dtype([(p[1], '<' + PLY_TYPES[p[0]]) for p in props])
            v = np.frombuffer(raw[pos:pos + count * dt.itemsize], dtype=dt, count=count)
            verts = np.stack([v['x'], v['y'], v['z']], 1).astype(np.float64)
            pos += count * dt.itemsize
        elif name == 'face':
            if count == 0:
                faces = np.zeros((0, 3), np.int64)
                continue
            p = props[0]
            if p[0] != 'list' or len(props) != 1:
                sys.exit('unsupported face layout')
            dt = np.dtype([('n', '<' + PLY_TYPES[p[1]]), ('i', '<' + PLY_TYPES[p[2]], 3)])
            f = np.frombuffer(raw[pos:pos + count * dt.itemsize], dtype=dt, count=count)
            if not np.all(f['n'] == 3):
                sys.exit('only triangle faces are supported')
            faces = f['i'].astype(np.int64)
            pos += count * dt.itemsize
    return verts, faces


def write_ply(path, verts, faces):
    with open(path, 'wb') as fh:
        fh.write((f'ply\nformat binary_little_endian 1.0\nelement vertex {len(verts)}\n'
                  'property float x\nproperty float y\nproperty float z\n'
                  f'element face {len(faces)}\nproperty list uchar int vertex_indices\n'
                  'end_header\n').encode('ascii'))
        fh.write(verts.astype('<f4').tobytes())
        f = np.zeros(len(faces), dtype=[('n', 'u1'), ('i', '<i4', 3)])
        f['n'] = 3
        f['i'] = faces
        fh.write(f.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mesh')
    ap.add_argument('cloud')
    ap.add_argument('out')
    ap.add_argument('--max-dist', type=float, default=0.02)
    ap.add_argument('--max-edge', type=float, default=0.10)
    a = ap.parse_args()
    V, F = read_ply(a.mesh)
    P, _ = read_ply(a.cloud)
    print(f'mesh: {len(V)} vertices, {len(F)} faces; TLS cloud: {len(P)} points', flush=True)
    d, _ = cKDTree(P).query(V, k=1, distance_upper_bound=1.0, workers=8)
    near = d < a.max_dist
    e = np.maximum.reduce([np.linalg.norm(V[F[:, i]] - V[F[:, (i + 1) % 3]], axis=1) for i in range(3)])
    keep = near[F].all(axis=1) & (e < a.max_edge)
    print(f'vertices within {a.max_dist} m of the TLS cloud: {near.mean():.1%}')
    print(f'faces kept: {keep.sum()} of {len(F)} ({keep.mean():.1%})')
    F = F[keep]
    used = np.unique(F)
    remap = -np.ones(len(V), np.int64)
    remap[used] = np.arange(len(used))
    write_ply(a.out, V[used], remap[F])
    lo = V[used].min(0)
    hi = V[used].max(0)
    print(f'bounds: {np.round(lo, 1)} .. {np.round(hi, 1)}')
    print(a.out)


if __name__ == '__main__':
    main()
