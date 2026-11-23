#!/usr/bin/env python3
"""Organized Ouster sweeps of a ROS 1 or ROS 2 bag within given still intervals.

The intervals (seconds from the start of the bag) come from bag_still.py, which
finds them in the IMU. Works with the clouds of the ouster driver (organized,
height = rings, with a `range` field in mm); clouds stored column by column
are transposed.

Usage: bag_static_extract.py <bag> <points_topic> <out_dir> --name NAME --intervals a:b[,c:d...]
Writes one npz per interval, as husky_extract.py: range [mm] and reflectivity
images, the xyz of the sweep with most returns (for the pixel directions), and stamps.
"""
import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def field(buf, f, dtype):
    n = np.dtype(dtype).itemsize
    return buf[:, f.offset:f.offset + n].copy().view(dtype)[:, 0]


# sensor_msgs/PointField datatypes used by the ouster drivers.
UINT = {2: '<u1', 4: '<u2', 6: '<u4'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('topic')
    ap.add_argument('out')
    ap.add_argument('--name', required=True)
    ap.add_argument('--intervals', required=True, help='a:b[,c:d...] in s from the start of the bag')
    ap.add_argument('--margin', type=float, default=0.5)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    iv = [tuple(float(x) for x in s.split(':')) for s in a.intervals.split(',')]
    with AnyReader([Path(a.bag)]) as r:
        con = [c for c in r.connections if c.topic == a.topic]
        t0 = r.start_time
        for i, (lo, hi) in enumerate(iv):
            start = t0 + int((lo + a.margin) * 1e9)
            stop = t0 + int((hi - a.margin) * 1e9)
            stamp = []
            ranges = []
            refl = []
            best = None
            best_n = -1
            for c, _, raw in r.messages(connections=con, start=start, stop=stop):
                m = r.deserialize(raw, c.msgtype)
                fs = {f.name: f for f in m.fields}
                buf = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
                H = m.height
                W = m.width
                ring = field(buf, fs['ring'], UINT[fs['ring'].datatype])
                col_major = H > 1 and np.array_equal(ring[:H], np.arange(H))

                def img(x):
                    if col_major:
                        return np.swapaxes(x.reshape((W, H) + x.shape[1:]), 0, 1)
                    return x.reshape((H, W) + x.shape[1:])

                rng = img(field(buf, fs['range'], UINT[fs['range'].datatype]).astype(np.uint32))
                stamp.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
                ranges.append(rng)
                refl.append(img(field(buf, fs['reflectivity'], UINT[fs['reflectivity'].datatype])
                                .astype(np.uint16)))
                nret = int((rng > 0).sum())
                if nret > best_n:
                    best_n = nret
                    best = img(np.stack([field(buf, fs[q], '<f4') for q in 'xyz'], 1))
            if not stamp:
                continue
            f = out / f'{a.name}_still{i:02d}.npz'
            np.savez_compressed(f, stamp=np.array(stamp), range=np.array(ranges),
                                refl=np.array(refl), xyz=best, interval=np.array([lo, hi]))
            print(f'  interval {i} ({lo:.1f}-{hi:.1f} s): {len(stamp)} sweeps -> {f.name}',
                  flush=True)


if __name__ == '__main__':
    main()
