#!/usr/bin/env python3
"""Renders perspective screenshots of each world with MVSim's own off-screen
renderer: `mvsim launch --headless` with a one-frame <video_recording> block
given in place of the sensor rig. Writes worlds/<name>/screenshot[_N].png.

Camera views come from SCREENSHOTS in each world.py (list of dicts with
point_to, distance, azimuth, elevation, fov, and optionally cut_z for a
cutaway view without the roofs and ceilings above that height), or a
default overview.

Usage: tools/screenshot.py [world ...]
"""
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))

from build_worlds import load_world_module  # noqa: E402
from mvsim_lio.plot import save_small_png  # noqa: E402


def render(world_xml, view, out_png, width=1280, height=800, world=None):
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        if view.get('cut_z') is not None:
            # Cutaway: same world without roofs/ceilings above cut_z
            world_xml = td / 'cutaway.world.xml'
            world_xml.write_text(world.to_xml(cut_z=view['cut_z']))
        px, py, pz = view['point_to']
        rig = td / 'shot.xml'
        rig.write_text(f'''<video_recording enabled="true">
  <output_file>{td}/frame_%03d.png</output_file>
  <fps>10</fps><width>{width}</width><height>{height}</height>
  <start_time>0</start_time><duration>0.15</duration>
  <close_simulator_when_done>true</close_simulator_when_done>
  <camera>orbit</camera>
  <cam_point_to>{px} {py} {pz}</cam_point_to>
  <cam_distance>{view['distance']}</cam_distance>
  <cam_azimuth>{view.get('azimuth', -60)}</cam_azimuth>
  <cam_elevation>{view.get('elevation', 35)}</cam_elevation>
  <fov_deg>{view.get('fov', 50)}</fov_deg>
</video_recording>
''')
        env = dict(os.environ, MVSIM_LIO_RIG=str(rig))
        r = subprocess.run(['mvsim', 'launch', str(world_xml), '--headless', '--realtime-factor',
                            '0'], env=env, capture_output=True, text=True, timeout=1800)
        frames = sorted(td.glob('frame_*.png'))
        if not frames:
            print(r.stdout[-2000:], r.stderr[-2000:])
            raise RuntimeError(f'no frame rendered for {world_xml}')
        save_small_png(Image.open(frames[0]), out_png, max_kb=300)


def main():
    names = sys.argv[1:] or sorted(p.parent.name for p in (ROOT / 'worlds').glob('*/world.xml'))
    for n in names:
        mod = load_world_module(n)
        views = getattr(mod, 'SCREENSHOTS', None)
        w = mod.make_world()
        if not views:
            xmin, xmax, ymin, ymax = w.extent
            span = max(xmax - xmin, ymax - ymin)
            views = [dict(point_to=((xmin + xmax) / 2, (ymin + ymax) / 2, 0), distance=0.9 * span)]
        for i, v in enumerate(views):
            out = ROOT / 'worlds' / n / ('screenshot.png' if i == 0 else f'screenshot_{i}.png')
            render(ROOT / 'worlds' / n / 'world.xml', v, out, world=w)
            print(f'{n}: {out.name} {out.stat().st_size / 1024:.0f} KB')


if __name__ == '__main__':
    main()
