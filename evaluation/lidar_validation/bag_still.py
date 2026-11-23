#!/usr/bin/env python3
"""Still intervals of a ROS 1 or ROS 2 bag from its IMU: 1 s windows whose gyroscope and
accelerometer spread (norm of the per-axis standard deviations) are below thresholds, in runs
of at least 10 s, searched in the first and last minutes of the bag.

Usage: bag_still.py <bag> <imu_topic> [gyro_std rad/s = 0.012] [accel_std m/s2 = 0.25]
"""
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def scan(bag, topic, head=180.0, tail=120.0):
    with AnyReader([Path(bag)]) as r:
        con = [c for c in r.connections if c.topic == topic]
        t0, t1 = r.start_time, r.end_time
        out = []
        for lo, hi in ((t0, min(t1, t0 + int(head * 1e9))), (max(t0, t1 - int(tail * 1e9)), t1)):
            for c, t, raw in r.messages(connections=con, start=lo, stop=hi):
                m = r.deserialize(raw, c.msgtype)
                g = m.angular_velocity
                a = m.linear_acceleration
                out.append((t * 1e-9, g.x, g.y, g.z, a.x, a.y, a.z))
        x = np.unique(np.array(out), axis=0)
        return x, t0 * 1e-9, t1 * 1e-9


def intervals(x, gthr, athr, min_len=10.0):
    t = x[:, 0]
    w = np.floor((t - t[0]) / 1.0).astype(int)
    still = []
    for k in np.unique(w):
        m = w == k
        if m.sum() < 20:
            continue
        gs = np.linalg.norm(x[m, 1:4].std(0))
        as_ = np.linalg.norm(x[m, 4:7].std(0))
        still.append((t[m][0], t[m][-1], gs, as_))
    still = np.array(still)
    ok = (still[:, 2] < gthr) & (still[:, 3] < athr)
    res = []
    i = 0
    while i < len(still):
        if ok[i]:
            j = i
            while j + 1 < len(still) and ok[j + 1] and still[j + 1, 0] - still[j, 1] < 1.5:
                j += 1
            if still[j, 1] - still[i, 0] >= min_len:
                res.append((still[i, 0], still[j, 1]))
            i = j + 1
        else:
            i += 1
    return still, res


if __name__ == '__main__':
    bag, topic = sys.argv[1], sys.argv[2]
    gthr = float(sys.argv[3]) if len(sys.argv) > 3 else 0.012
    athr = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25
    x, t0, t1 = scan(bag, topic)
    st, res = intervals(x, gthr, athr)
    q = np.percentile(st[:, 2:], [5, 25, 50], axis=0)
    print(f'{Path(bag).name} {topic}: {t1 - t0:.0f} s; 1-s std gyro p5/25/50 {q[:, 0].round(4)} acc {q[:, 1].round(3)}')
    for a, b in res:
        print(f'   still {a - t0:7.1f} .. {b - t0:7.1f} s  ({b - a:.1f} s)')
