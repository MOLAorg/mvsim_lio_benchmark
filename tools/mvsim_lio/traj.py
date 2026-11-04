"""Smooth, physically plausible 6-DoF trajectories for mvsim-dataset-gen.

A trajectory is described by a list of items:
  P(x, y, z=None, v=None)   path point. z is only used with explicit-z
                            trajectories (multi-level structures); v, if given,
                            is the speed limit from this point on [m/s].
  Stop(seconds)             come to rest at the last point and wait.
  Spin(angle_deg, seconds)  come to rest and rotate in place (smooth profile).

The builder resamples the polyline, rounds its corners, computes a speed
profile limited by the platform's longitudinal and lateral accelerations,
samples it in time, smooths it, and then adds z/pitch/roll from the terrain
(or from the explicit z profile) and the platform's own motion (gait bobbing
and sway for handheld/legged carriers). Every trajectory starts and ends with
a few seconds at rest, so that LiDAR-inertial systems can initialize.

Output: poses at RATE_HZ, as arrays (t, x, y, z, yaw, pitch, roll). Angles in
radians, MRPT convention: R = Rz(yaw) Ry(pitch) Rx(roll), positive pitch is
nose-down.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation

RATE_HZ = 200.0
T0 = 100.0  # first timestamp [s]; never 0, which some tools read as "unset"


@dataclass
class P:
    x: float
    y: float
    z: float = None
    v: float = None


@dataclass
class Stop:
    seconds: float


@dataclass
class Spin:
    angle_deg: float
    seconds: float


# Carrier platforms. h_base: height of the base_link frame above the ground
# (the rig's LiDAR is 0.75 m above base_link, see rigs/). length/width: wheel
# footprint used to tilt the body with the terrain. gait: (step length [m],
# vertical bob amplitude [m], roll sway [deg], pitch sway [deg]) or None.
PLATFORMS = {
    'car': dict(h_base=1.15, length=2.7, width=1.6, a_long=1.5, a_lat=2.5, corner=4.0,
                gait=None, vmax=15.0),
    'ugv': dict(h_base=0.0, length=0.8, width=0.6, a_long=1.0, a_lat=1.5, corner=0.8,
                gait=None, vmax=3.0),
    'legged': dict(h_base=0.35, length=0.7, width=0.4, a_long=1.0, a_lat=1.5, corner=0.6,
                   gait=(0.30, 0.012, 1.0, 1.2), vmax=1.8),
    'handheld': dict(h_base=0.75, length=0.3, width=0.3, a_long=0.8, a_lat=1.2, corner=0.6,
                     gait=(0.70, 0.025, 1.5, 1.0), vmax=1.8),
}


def _resample_polyline(pts, ds):
    """pts: (N, k) array (x, y, [z], v). Returns samples every ds along XY(Z)."""
    seg = np.linalg.norm(np.diff(pts[:, :3], axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(math.ceil(s[-1] / ds)) + 1)
    si = np.linspace(0.0, s[-1], n)
    out = np.column_stack([np.interp(si, s, pts[:, k]) for k in range(3)])
    # Speed limit: piecewise constant, the value of the segment's start point.
    idx = np.clip(np.searchsorted(s, si, side='right') - 1, 0, len(pts) - 2)
    v = pts[idx, 3]
    return out, v


def _smooth_path(xyz, sigma_samples):
    """Gaussian smoothing along the path keeping both endpoints fixed."""
    if sigma_samples < 0.5 or len(xyz) < 5:
        return xyz
    pad = min(int(4 * sigma_samples) + 1, len(xyz) - 1)
    out = np.empty_like(xyz)
    for k in range(xyz.shape[1]):
        a = xyz[:, k]
        # Odd reflection about the endpoints keeps them (and the end tangents) fixed.
        left = 2 * a[0] - a[1:pad + 1][::-1]
        right = 2 * a[-1] - a[-pad - 1:-1][::-1]
        ext = np.concatenate([left, a, right])
        out[:, k] = gaussian_filter1d(ext, sigma_samples, mode='nearest')[pad:-pad]
    return out


def _speed_profile(s, kappa, vlim, a_long, a_lat):
    v = np.minimum(vlim, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-9)))
    v[0] = 0.0
    v[-1] = 0.0
    ds = np.diff(s)
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_long * ds[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_long * ds[i]))
    return v


def _run_to_time(pts, plat, dt):
    """One run of path points (from rest to rest) -> time series."""
    xyz, vlim = _resample_polyline(pts, ds=0.05)
    sig = plat['corner'] / 0.05
    # Corners are rounded with the platform radius, but z (explicit-z paths:
    # ramps, stairs) only over ~1 m, like a body following a slope change.
    xyz = np.column_stack([_smooth_path(xyz[:, :2], sig), _smooth_path(xyz[:, 2:], min(sig, 20.0))])
    d = np.diff(xyz, axis=0)
    seg = np.linalg.norm(d[:, :2], axis=1)
    seg = np.maximum(seg, 1e-9)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    # Heading and curvature along the path:
    hx = np.gradient(xyz[:, 0], s)
    hy = np.gradient(xyz[:, 1], s)
    yaw = np.unwrap(np.arctan2(hy, hx))
    kappa = np.gradient(yaw, s)
    kappa = gaussian_filter1d(kappa, 5)
    slope = np.gradient(xyz[:, 2], s)
    v = _speed_profile(s, kappa, vlim, plat['a_long'], plat['a_lat'])
    # Time along the path (trapezoidal in 1/v, robust at v=0 endpoints):
    vm = np.maximum(0.5 * (v[1:] + v[:-1]), 1e-3)
    tt = np.concatenate([[0.0], np.cumsum(seg / vm)])
    tq = np.arange(0.0, tt[-1], dt)
    sq = np.interp(tq, tt, s)
    return dict(
        x=np.interp(sq, s, xyz[:, 0]), y=np.interp(sq, s, xyz[:, 1]),
        z=np.interp(sq, s, xyz[:, 2]), yaw=np.interp(sq, s, yaw),
        slope=np.interp(sq, s, slope), s=sq)


class _Flat:
    """Flat ground at z = 0, for worlds without an elevation map."""

    @staticmethod
    def height(x, y):
        return np.zeros_like(np.asarray(x, dtype=float))


def build(items, platform, terrain=None, explicit_z=False, seed=0, rest=3.0,
          time_sigma=0.15, look_around_deg=0.0, pitch_scale=1.0):
    """Returns dict of arrays t, x, y, z, yaw, pitch, roll (base_link poses)."""
    plat = PLATFORMS[platform]
    dt = 1.0 / RATE_HZ
    rng = np.random.default_rng(seed)
    cols = {k: [] for k in ('x', 'y', 'z', 'yaw', 'slope', 's')}
    state = None  # last (x, y, z, yaw, slope, s)
    s_total = 0.0

    def hold(n, yaw_profile=None):
        x, y, z, yaw, slope, s0 = state
        cols['x'].append(np.full(n, x))
        cols['y'].append(np.full(n, y))
        cols['z'].append(np.full(n, z))
        cols['yaw'].append(yaw + (yaw_profile if yaw_profile is not None else np.zeros(n)))
        cols['slope'].append(np.full(n, slope))
        cols['s'].append(np.full(n, s0))

    run = []
    first = items[0]
    assert isinstance(first, P)
    vcur = plat['vmax']

    def flush_run():
        nonlocal state, run, s_total
        if len(run) < 2:
            run = run[-1:]
            return
        arr = np.array(run, dtype=float)
        r = _run_to_time(arr, plat, dt)
        if state is not None:
            # Keep yaw continuous across runs (unwrap the new run near the old yaw).
            r['yaw'] += 2 * math.pi * round((state[3] - r['yaw'][0]) / (2 * math.pi))
            delta = r['yaw'][0] - state[3]
            if abs(delta) > math.radians(2.0):
                # The new run starts with another heading: turn in place first.
                n = int((abs(delta) / math.radians(45.0) + 1.0) * RATE_HZ)
                u = np.linspace(0.0, 1.0, n)
                hold(n, delta * (u - np.sin(2 * math.pi * u) / (2 * math.pi)))
        r['s'] = r['s'] + s_total
        for k in cols:
            cols[k].append(r[k])
        s_total = r['s'][-1]
        state = (r['x'][-1], r['y'][-1], r['z'][-1], r['yaw'][-1], r['slope'][-1], s_total)
        run = run[-1:]

    # Initial yaw: towards the first distinct point.
    nxt = next(it for it in items[1:] if isinstance(it, P))
    yaw0 = math.atan2(nxt.y - first.y, nxt.x - first.x)
    state = (first.x, first.y, first.z or 0.0, yaw0, 0.0, 0.0)
    hold(int(rest * RATE_HZ))
    for it in items:
        if isinstance(it, P):
            if it.v is not None:
                vcur = min(it.v, plat['vmax'])
            run.append((it.x, it.y, it.z if it.z is not None else 0.0, vcur))
        elif isinstance(it, Stop):
            flush_run()
            hold(int(it.seconds * RATE_HZ))
        elif isinstance(it, Spin):
            flush_run()
            n = int(it.seconds * RATE_HZ)
            u = np.linspace(0.0, 1.0, n)
            prof = math.radians(it.angle_deg) * (u - np.sin(2 * math.pi * u) / (2 * math.pi))
            hold(n, prof)
            state = (state[0], state[1], state[2], state[3] + math.radians(it.angle_deg),
                     state[4], state[5])
        else:
            raise TypeError(it)
    flush_run()
    hold(int(rest * RATE_HZ))

    c = {k: np.concatenate(v) for k, v in cols.items()}
    n = len(c['x'])
    t = T0 + dt * np.arange(n)
    sig = time_sigma * RATE_HZ
    x = gaussian_filter1d(c['x'], sig, mode='nearest')
    y = gaussian_filter1d(c['y'], sig, mode='nearest')
    yaw = gaussian_filter1d(c['yaw'], sig, mode='nearest')
    s = gaussian_filter1d(c['s'], sig, mode='nearest')

    if look_around_deg > 0:
        # Slow, band-limited head/rig rotation (handheld carriers look around).
        noise = gaussian_filter1d(rng.standard_normal(n), 1.5 * RATE_HZ, mode='nearest')
        noise *= math.radians(look_around_deg) / max(np.std(noise), 1e-9)
        # Fade in/out so that the rest periods stay at rest:
        speed = np.gradient(s, dt)
        w = gaussian_filter1d((speed > 0.05).astype(float), 1.0 * RATE_HZ, mode='nearest')
        yaw = yaw + noise * w

    if explicit_z:
        z = gaussian_filter1d(c['z'], sig, mode='nearest')
        pitch = -pitch_scale * np.arctan(gaussian_filter1d(c['slope'], sig, mode='nearest'))
        roll = np.zeros(n)
    else:
        if terrain is None:
            terrain = _Flat()
        L, W = plat['length'], plat['width']
        cy, sy = np.cos(yaw), np.sin(yaw)

        def probe(lx, ly):
            return terrain.height(x + lx * cy - ly * sy, y + lx * sy + ly * cy)

        zfl, zfr = probe(L / 2, W / 2), probe(L / 2, -W / 2)
        zbl, zbr = probe(-L / 2, W / 2), probe(-L / 2, -W / 2)
        z = 0.25 * (zfl + zfr + zbl + zbr)
        pitch = np.arctan(-((zfl + zfr) - (zbl + zbr)) / (2 * L))
        roll = np.arctan(((zfl + zbl) - (zfr + zbr)) / (2 * W))
        z = gaussian_filter1d(z, sig, mode='nearest')
        pitch = gaussian_filter1d(pitch, sig, mode='nearest')
        roll = gaussian_filter1d(roll, sig, mode='nearest')

    z = z + plat['h_base']
    if plat['gait']:
        step, bob, sway_r, sway_p = plat['gait']
        speed = np.gradient(s, dt)
        amp = np.clip(speed / 1.2, 0.0, 1.0)
        amp = gaussian_filter1d(amp, 0.3 * RATE_HZ, mode='nearest')
        phase = 2 * math.pi * s / step
        z = z + bob * amp * np.cos(2 * phase)
        roll = roll + math.radians(sway_r) * amp * np.sin(phase)
        pitch = pitch + math.radians(sway_p) * amp * np.sin(2 * phase + 0.7)

    return dict(t=t, x=x, y=y, z=z, yaw=yaw, pitch=pitch, roll=roll)


def to_quat(tr):
    """(N, 4) quaternions qx qy qz qw for R = Rz(yaw) Ry(pitch) Rx(roll)."""
    rot = Rotation.from_euler('ZYX', np.column_stack([tr['yaw'], tr['pitch'], tr['roll']]))
    q = rot.as_quat()  # x, y, z, w
    # Keep a continuous sign (no hemisphere flips between consecutive samples):
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    return q


def write_tum(path, tr, every=1):
    q = to_quat(tr)
    with open(path, 'w') as f:
        f.write('# timestamp x y z qx qy qz qw  (base_link in the world frame)\n')
        for i in range(0, len(tr['t']), every):
            f.write(f"{tr['t'][i]:.4f} {tr['x'][i]:.6f} {tr['y'][i]:.6f} {tr['z'][i]:.6f} "
                    f'{q[i, 0]:.9f} {q[i, 1]:.9f} {q[i, 2]:.9f} {q[i, 3]:.9f}\n')


def stats(tr):
    t, x, y, z = tr['t'], tr['x'], tr['y'], tr['z']
    dt = t[1] - t[0]
    vx, vy, vz = (np.gradient(a, dt) for a in (x, y, z))
    sp = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
    yawrate = np.gradient(tr['yaw'], dt)
    length = float(np.sum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2 + np.diff(z) ** 2)))
    return dict(duration=float(t[-1] - t[0]), length=length, vmax=float(sp.max()),
                vmean=float(length / (t[-1] - t[0])),
                yawrate_max_deg=float(np.degrees(np.abs(yawrate).max())),
                pitch_max_deg=float(np.degrees(np.abs(tr['pitch']).max())),
                roll_max_deg=float(np.degrees(np.abs(tr['roll']).max())),
                z_range=float(z.max() - z.min()))
