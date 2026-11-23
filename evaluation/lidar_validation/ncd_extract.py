#!/usr/bin/env python3
"""Organized OS1-64 sweeps of Newer College 01_short where the device is nearly still.

Selects the ground-truth poses inside the bounds of the reference mesh with an
angular rate below --max-rate and a small change of the rotation increment,
and saves the matching sweeps (range, reflectivity, per-point time, xyz) with
their ground-truth pose.

Usage: ncd_extract.py <sequence_dir> <mesh_bounds: x0 x1 y0 y1> <out.npz>
"""
import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation as R

TOPIC = '/os1_cloud_node/points'


def field(buf, f, dtype):
    n = np.dtype(dtype).itemsize
    return buf[:, f.offset:f.offset + n].copy().view(dtype)[:, 0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seq')
    ap.add_argument('bounds', type=float, nargs=4)
    ap.add_argument('out')
    ap.add_argument('--max-rate', type=float, default=3.0, help='deg/s')
    ap.add_argument('--max-inc-change', type=float, default=0.2, help='deg')
    ap.add_argument('--max-dt', type=float, default=0.03, help='scan to GT stamp [s]')
    a = ap.parse_args()
    seq = Path(a.seq)
    g = np.loadtxt(seq / 'ground_truth' / 'gt.tum')
    t = g[:, 0]
    p = g[:, 1:4]
    rot = R.from_quat(g[:, 4:8])
    x0, x1, y0, y1 = a.bounds
    inside = (p[:, 0] > x0) & (p[:, 0] < x1) & (p[:, 1] > y0) & (p[:, 1] < y1)
    inc = (rot[1:] * rot[:-1].inv()).as_rotvec()
    w = np.r_[0, np.degrees(np.linalg.norm(inc, axis=1)) / np.diff(t)]
    dinc = np.r_[0, 0, np.degrees(np.linalg.norm(np.diff(inc, axis=0), axis=1))]
    sel = np.flatnonzero(inside & (w < a.max_rate) & (dinc < a.max_inc_change))
    print(f'{len(sel)} ground-truth poses selected', flush=True)
    want = t[sel]
    bags = sorted((seq / 'rosbag').glob('*_lio.bag'))
    out = {k: [] for k in ('stamp', 'gt', 'range', 'refl', 'tns', 'xyz')}
    with AnyReader(bags) as rd:
        con = [c for c in rd.connections if c.topic == TOPIC]
        lo = int((want.min() - 1) * 1e9)
        hi = int((want.max() + 1) * 1e9)
        for c, tr, raw in rd.messages(connections=con, start=lo, stop=hi):
            m = rd.deserialize(raw, c.msgtype)
            st = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            k = np.searchsorted(want, st)
            best = min((abs(want[j] - st), j) for j in (k - 1, k) if 0 <= j < len(want))
            if best[0] > a.max_dt:
                continue
            j = sel[best[1]]
            fs = {f.name: f for f in m.fields}
            buf = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
            H = m.height
            W = m.width
            # The 2020 driver stores the points column by column (index =
            # column * H + ring) despite height = rings, width = columns.
            ring = field(buf, fs['ring'], '<u1')
            col_major = np.array_equal(ring[:H], np.arange(H))

            def img(x):
                if col_major:
                    return np.swapaxes(x.reshape((W, H) + x.shape[1:]), 0, 1)
                return x.reshape((H, W) + x.shape[1:])

            out['stamp'].append(st)
            out['gt'].append(g[j])
            out['range'].append(img(field(buf, fs['range'], '<u4')))
            out['refl'].append(img(field(buf, fs['reflectivity'], '<u2')))
            out['tns'].append(img(field(buf, fs['t'], '<u4')))
            xyz = np.stack([field(buf, fs[k], '<f4') for k in 'xyz'], 1)
            out['xyz'].append(img(xyz))
            if len(out['stamp']) % 25 == 0:
                print(f'  {len(out["stamp"])} sweeps', flush=True)
    np.savez_compressed(a.out, **{k: np.array(v) for k, v in out.items()})
    print(f'{len(out["stamp"])} sweeps -> {a.out}')


if __name__ == '__main__':
    main()
