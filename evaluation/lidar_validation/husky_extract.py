#!/usr/bin/env python3
"""Organized Ouster sweeps of a Husky bag while the robot stands still.

Stationary intervals come from the wheel odometry (|v| < 1 cm/s and
|w| < 0.3 deg/s for at least --min-len s, trimmed by --margin s at both ends),
on the bag's receive clock, the same clock used to select the sweeps.

Usage: husky_extract.py <bag.mcap> <out_dir>
Writes one npz per interval: range [mm] and reflectivity images, the
xyz of the sweep with most returns (for the pixel directions), and stamps.
"""
import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

NS = '/a200_0812'
ODOM = f'{NS}/platform/odom'
POINTS = f'{NS}/ouster/points'


def open_reader(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r, types


def field(buf, f, dtype):
    n = np.dtype(dtype).itemsize
    return buf[:, f.offset:f.offset + n].copy().view(dtype)[:, 0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--min-len', type=float, default=10.0)
    ap.add_argument('--margin', type=float, default=1.0)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    r, types = open_reader(a.bag, [ODOM])
    od = []
    while r.has_next():
        _, data, t_rec = r.read_next()
        m = deserialize_message(data, get_message(types[ODOM]))
        od.append((t_rec * 1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z))
    od = np.array(od)
    still = (np.abs(od[:, 1]) < 0.01) & (np.abs(od[:, 2]) < np.radians(0.3))
    edges = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0]))
    iv = [(od[s, 0] + a.margin, od[e - 1, 0] - a.margin) for s, e in zip(edges[::2], edges[1::2])
          if od[e - 1, 0] - od[s, 0] >= a.min_len]
    print(f'{len(iv)} stationary intervals, {sum(b - s for s, b in iv):.0f} s', flush=True)
    r, types = open_reader(a.bag, [POINTS])
    cur = {i: dict(stamp=[], range=[], refl=[], best=None, best_n=-1) for i in range(len(iv))}
    starts = np.array([s for s, _ in iv])
    while r.has_next():
        _, data, t_rec = r.read_next()
        tr = t_rec * 1e-9
        k = np.searchsorted(starts, tr) - 1
        if k < 0 or tr > iv[k][1]:
            continue
        m = deserialize_message(data, get_message(types[POINTS]))
        fs = {f.name: f for f in m.fields}
        buf = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
        H = m.height
        W = m.width
        ring = field(buf, fs['ring'], '<u2' if fs['ring'].datatype == 4 else '<u1')
        col_major = H > 1 and np.array_equal(ring[:H], np.arange(H))

        def img(x):
            if col_major:
                return np.swapaxes(x.reshape((W, H) + x.shape[1:]), 0, 1)
            return x.reshape((H, W) + x.shape[1:])

        rng = img(field(buf, fs['range'], '<u4'))
        c = cur[k]
        c['stamp'].append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
        c['range'].append(rng)
        c['refl'].append(img(field(buf, fs['reflectivity'], '<u2')))
        nret = int((rng > 0).sum())
        if nret > c['best_n']:
            c['best_n'] = nret
            c['best'] = img(np.stack([field(buf, fs[q], '<f4') for q in 'xyz'], 1))
    name = Path(a.bag).stem
    for i, c in cur.items():
        if not c['stamp']:
            continue
        f = out / f'{name}_still{i:02d}.npz'
        np.savez_compressed(f, stamp=np.array(c['stamp']), range=np.array(c['range']),
                            refl=np.array(c['refl']), xyz=c['best'], interval=np.array(iv[i]))
        print(f'  interval {i}: {len(c["stamp"])} sweeps -> {f.name}', flush=True)


if __name__ == '__main__':
    main()
