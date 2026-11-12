#!/usr/bin/env python3
"""Renders the animated banner at the top of the README (docs/banner.gif and
docs/banner.mp4).

Each segment shows one world: the exact LiDAR map of one of its sequences is
revealed as the sensor travels along its ground-truth trajectory, seen from a
fixed camera, with MVSim's own rendering of the world as an inset. Points are
colored by height, shaded with eye-dome lighting, and glow briefly when first
seen. Everything runs on the CPU (numpy point splatting with a z-buffer). The
camera is fixed within a segment so that consecutive frames differ only where
new points appear, which keeps the GIF small.

Inputs: one point-cloud map per segment, <maps>/<world>__<traj>.bin, made of
little-endian float32 records (x, y, z, t_first): voxel centroids of the
clean global-shutter scans placed at the ground-truth poses, and the time
[s, from the trajectory start] each voxel was first seen. The trajectory is
the committed worlds/<world>/trajectories/<traj>.tum. The maps of the
published banner were made with tools/map_export (bench-map-export) as:

  bench-map-export --rawlog <data>/<world>/<traj>/os1_32-clean-global.rawlog \
      --traj <data>/<world>/<traj>/gt_200hz.tum --out <maps>/<world>__<traj>.bin \
      --voxel 0.12 --every 2 --t-end <last time of the segment>

Requirements: numpy, matplotlib (colormap, font), Pillow, ffmpeg.

Usage: tools/make_banner.py --maps <dir> [--out docs] [--work <frames dir>] [--fps 12]
       [--only world,...]
"""
import argparse
import subprocess
import tempfile
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent

W, H = 960, 400
BG_TOP = np.array([11, 14, 20], float)
BG_BOTTOM = np.array([24, 29, 40], float)
FOV_DEG = 55.0
LIDAR_Z = 0.75  # LiDAR height above base_link

# One segment per world: time window [s] of the sequence (times start at 100),
# camera elevation and azimuth [deg], zoom (1: the final map of the segment just
# fits the frame, larger values zoom in), and an optional cut (hide points
# higher than this above the sensor: ceilings, upper floors, tree crowns).
SEGMENTS = [
    dict(world='urban_street', traj='car_loops', label='urban street  |  car, 10 m/s',
         t=(226.0, 268.0), elev=42.0, az=-120.0, zoom=1.25, cut=None),
    dict(world='parking_garage', traj='up_down', label='parking garage  |  car, ramps between levels',
         t=(100.0, 272.0), elev=40.0, az=-70.0, zoom=1.2, cut=1.0),
    dict(world='forest', traj='trail_loop', label='forest  |  legged robot on slopes',
         t=(100.0, 240.0), elev=45.0, az=30.0, zoom=1.1, cut=7.0),
    dict(world='stair_course', traj='dog_course', label='stair course  |  quadruped, straight and spiral stairs',
         t=(100.0, 261.0), elev=38.0, az=-120.0, zoom=1.25, cut=None),
    dict(world='warehouse', traj='aisles', label='warehouse  |  UGV between racks',
         t=(100.0, 230.0), elev=50.0, az=-110.0, zoom=1.15, cut=6.5),
    dict(world='orchard', traj='rows', label='orchard  |  UGV along repetitive tree rows',
         t=(100.0, 260.0), elev=45.0, az=-60.0, zoom=1.15, cut=None),
]

MAP_S = 2.8       # map animation per segment
HOLD_S = 0.4      # final map held still
FADE_S = 0.17     # fade in / out between segments
INSET_W = 230     # width of the MVSim screenshot inset [px]


def load_traj(world, traj):
    d = np.loadtxt(ROOT / 'worlds' / world / 'trajectories' / f'{traj}.tum', comments='#')
    return d[:, 0], d[:, 1:4]


def position_at(t, ts, ps):
    return np.array([np.interp(t, ts, ps[:, i]) for i in range(3)])


def camera(target, dist, elev_deg, az_deg):
    el, az = np.radians(elev_deg), np.radians(az_deg)
    eye = target + dist * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return eye, np.stack([right, up, fwd])


def project(P, eye, R, f):
    X = (P - eye) @ R.T
    z = X[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = W / 2 + f * X[:, 0] / z
        v = H / 2 - f * X[:, 1] / z
    return u, v, z


def fit_distance(P, target, elev, az, f, margin=24, keep=0.97):
    """Camera distance at which `keep` of the points project inside the frame."""
    rng = np.random.default_rng(0)
    Q = P[rng.choice(len(P), min(len(P), 30000), replace=False)]
    dist = 5.0
    while dist < 5000:
        eye, R = camera(target, dist, elev, az)
        u, v, z = project(Q, eye, R, f)
        inside = (z > 0.5) & (u > margin) & (u < W - margin) & (v > margin + 30) & (v < H - margin)
        if inside.mean() >= keep:
            return dist
        dist *= 1.05
    return dist


def background():
    a = np.linspace(0, 1, H)[:, None, None]
    return np.broadcast_to(BG_TOP * (1 - a) + BG_BOTTOM * a, (H, W, 3)).copy()


def render_map(P, col, glow, eye, R, f):
    """Splats points (nearer ones bigger) with a z-buffer and eye-dome lighting."""
    u, v, z = project(P, eye, R, f)
    ok = (z > 0.5) & (u > -2) & (u < W + 1) & (v > -2) & (v < H + 1)
    u, v, z, col, glow = u[ok], v[ok], z[ok], col[ok], glow[ok]
    ui, vi = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64)
    # Splat size from the projected size of a ~12 cm voxel:
    size = np.clip(0.16 * f / z, 1, 3).astype(np.int64)
    us, vs, zs, idx = [ui], [vi], [z], [np.arange(len(z))]
    for du in range(3):
        for dv in range(3):
            if du == 0 and dv == 0:
                continue
            sel = np.nonzero(size > max(du, dv))[0]
            us.append(ui[sel] + du)
            vs.append(vi[sel] + dv)
            zs.append(z[sel])
            idx.append(sel)
    us, vs, zs, idx = (np.concatenate(x) for x in (us, vs, zs, idx))
    inside = (us >= 0) & (us < W) & (vs >= 0) & (vs < H)
    us, vs, zs, idx = us[inside], vs[inside], zs[inside], idx[inside]
    pid = vs * W + us
    order = np.lexsort((zs, pid))
    first = np.unique(pid[order], return_index=True)[1]
    win = order[first]
    depth = np.full(H * W, np.inf)
    depth[pid[win]] = zs[win]
    rgb = np.zeros((H * W, 3))
    rgb[pid[win]] = col[idx[win]]
    g = np.zeros(H * W)
    g[pid[win]] = glow[idx[win]]
    depth = depth.reshape(H, W)
    rgb = rgb.reshape(H, W, 3)
    g = g.reshape(H, W)
    # Eye-dome lighting on log-depth: darkens silhouettes and creases.
    valid = np.isfinite(depth)
    ld = np.where(valid, np.log2(np.where(valid, depth, 1.0)), 0.0)
    acc = np.zeros_like(ld)
    for dy, dx in ((-2, 0), (2, 0), (0, -2), (0, 2), (-1, -1), (1, 1), (-1, 1), (1, -1)):
        nb = np.roll(np.roll(ld, dy, 0), dx, 1)
        nbv = np.roll(np.roll(valid, dy, 0), dx, 1)
        acc += np.where(nbv, np.maximum(0.0, ld - nb), 0.6)
    shade = np.exp(-1.6 * acc)
    rgb = rgb * shade[..., None]
    rgb = rgb * (1 - g[..., None]) + 255.0 * g[..., None]
    img = background()
    img[valid] = rgb[valid]
    return img


def draw_overlay(img, label, trail_uv, pos_uv, inset, font, font_small):
    im = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(im, 'RGBA')
    if len(trail_uv) > 1:
        d.line([tuple(p) for p in trail_uv], fill=(255, 255, 255, 215), width=2)
    if pos_uv is not None:
        x, y = pos_uv
        d.ellipse([x - 8, y - 8, x + 8, y + 8], outline=(255, 64, 160, 255), width=2)
        d.ellipse([x - 3, y - 3, x + 3, y + 3], fill=(255, 255, 255, 255))
    d.rectangle([0, 0, W, 44], fill=(8, 10, 15, 150))
    d.rectangle([W - 470, H - 34, W, H], fill=(8, 10, 15, 150))
    d.text((18, 12), label, font=font, fill=(235, 240, 250, 255))
    tag = 'MVSim LIO benchmark  |  exact ground truth: trajectory + map'
    tw = d.textlength(tag, font=font_small)
    d.text((W - tw - 16, H - 26), tag, font=font_small, fill=(170, 180, 200, 255))
    x0, y0 = 16, H - inset.height - 16
    im.paste(inset, (x0, y0))
    d.rectangle([x0 - 1, y0 - 1, x0 + inset.width, y0 + inset.height], outline=(120, 130, 150, 255))
    d.text((x0 + 6, y0 + 4), 'MVSim world', font=font_small, fill=(20, 24, 32, 255))
    return np.asarray(im, dtype=float)


def screenshot_inset(world):
    im = Image.open(ROOT / 'worlds' / world / 'screenshot.png').convert('RGB')
    h = round(im.height * INSET_W / im.width)
    return im.resize((INSET_W, h), Image.LANCZOS)


def render_segment(seg, maps_dir, fps, frames_dir, start_index, font, font_small):
    ts, ps = load_traj(seg['world'], seg['traj'])
    ps = ps + [0, 0, LIDAR_Z]
    m = np.fromfile(maps_dir / f"{seg['world']}__{seg['traj']}.bin", dtype=np.float32)
    m = m.reshape(-1, 4).astype(np.float64)
    t0, t1 = seg['t']
    # Only what is first seen inside the time window of the segment:
    m = m[(m[:, 3] + ts[0] >= t0) & (m[:, 3] + ts[0] <= t1)]
    P, tf = m[:, :3], m[:, 3] + ts[0]
    lo, hi = np.percentile(P[:, 2], [2, 98])
    cmap = matplotlib.colormaps['turbo']
    base_col = cmap(0.12 + 0.86 * np.clip((P[:, 2] - lo) / max(hi - lo, 1e-3), 0, 1))[:, :3] * 255
    f = (W / 2) / np.tan(np.radians(FOV_DEG / 2))
    inset = screenshot_inset(seg['world'])
    # Fixed camera, centered on the part of the trajectory shown:
    final = tf <= t1
    if seg['cut'] is not None:
        final &= P[:, 2] < ps[:, 2].max() + seg['cut']
    target = np.percentile(P[final], 50, axis=0)
    dist = fit_distance(P[final], target, seg['elev'], seg['az'], f) / seg['zoom']
    eye, R = camera(target, dist, seg['elev'], seg['az'])
    d_eye = np.linalg.norm(P - eye, axis=1)
    fog = np.exp(-np.maximum(0, d_eye - dist) / (2.0 * dist))[:, None]
    col_all = base_col * fog + BG_BOTTOM * (1 - fog)
    n_map, n_hold, n_fade = round(MAP_S * fps), round(HOLD_S * fps), round(FADE_S * fps)
    n = n_map + n_hold
    k = start_index
    for j in range(n):
        t = t0 + (t1 - t0) * min(1.0, j / (n_map - 1))
        sel = tf <= t
        if seg['cut'] is not None:
            sel &= P[:, 2] < position_at(t, ts, ps)[2] + seg['cut']
        # Newly seen points glow for about a second of animation:
        age = t - tf[sel]
        glow = 0.55 * np.exp(-age / (0.012 * (t1 - t0)))
        img = render_map(P[sel], col_all[sel], glow, eye, R, f)
        tt = np.linspace(t0, t, max(2, int((t - t0) * 4)))
        trail = np.array([position_at(x, ts, ps) for x in tt])
        u, v, z = project(trail, eye, R, f)
        trail_uv = np.stack([u, v], 1)[z > 0.5]
        pu, pv, pz = project(position_at(t, ts, ps)[None], eye, R, f)
        pos_uv = (pu[0], pv[0]) if pz[0] > 0.5 else None
        img = draw_overlay(img, seg['label'], trail_uv, pos_uv, inset, font, font_small)
        if j < n_fade:
            img = img * (j + 1) / (n_fade + 1)
        if j >= n - n_fade:
            img = img * (n - j) / (n_fade + 1)
        Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(frames_dir / f'{k:05d}.png')
        k += 1
    return k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--maps', required=True)
    ap.add_argument('--out', default=str(ROOT / 'docs'))
    ap.add_argument('--work', default='', help='directory for the frames (default: a temporary one)')
    ap.add_argument('--fps', type=int, default=12)
    ap.add_argument('--gif-width', type=int, default=800)
    ap.add_argument('--only', default='', help='comma-separated worlds (for previews)')
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    tmp = None
    if a.work:
        frames = Path(a.work)
        frames.mkdir(parents=True, exist_ok=True)
        for old in frames.glob('[0-9][0-9][0-9][0-9][0-9].png'):
            old.unlink()
    else:
        tmp = tempfile.TemporaryDirectory()
        frames = Path(tmp.name)
    fonts = Path(matplotlib.get_data_path()) / 'fonts' / 'ttf'
    font = ImageFont.truetype(str(fonts / 'DejaVuSans-Bold.ttf'), 19)
    font_small = ImageFont.truetype(str(fonts / 'DejaVuSans.ttf'), 13)
    k = 0
    for seg in SEGMENTS:
        if a.only and seg['world'] not in a.only.split(','):
            continue
        k = render_segment(seg, Path(a.maps), a.fps, frames, k, font, font_small)
        print(f"{seg['world']}: {k} frames so far", flush=True)
    pattern = str(frames / '%05d.png')
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(a.fps), '-i', pattern,
                    '-c:v', 'libx264', '-preset', 'slow', '-crf', '24', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', str(out / 'banner.mp4')], check=True)
    palette = frames / 'palette.png'
    scale = f'scale={a.gif_width}:-1:flags=lanczos'
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(a.fps), '-i', pattern,
                    '-vf', f'{scale},palettegen=max_colors=128:stats_mode=diff', str(palette)],
                   check=True)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(a.fps), '-i', pattern,
                    '-i', str(palette), '-lavfi',
                    f'{scale}[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle',
                    '-loop', '0', str(out / 'banner.gif')], check=True)
    for name in ('banner.mp4', 'banner.gif'):
        print(f'{out / name}: {(out / name).stat().st_size / 1e6:.2f} MB')
    if tmp:
        tmp.cleanup()


if __name__ == '__main__':
    main()
