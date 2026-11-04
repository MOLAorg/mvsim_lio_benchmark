#!/usr/bin/env python3
"""Renders every (world, trajectory, variant) sequence with mvsim-dataset-gen.

Output layout (under --out, default /mnt/datasets/mvsim_lio_benchmark):
  <world>/<traj>/<rig>-<noise>-<shutter>.rawlog       sensor data (MRPT rawlog)
  <world>/<traj>/<rig>-<noise>-<shutter>.gt.tum       base_link GT at sweep start times
  <world>/<traj>/<rig>-<noise>-<shutter>.lidar1.gt.tum  LiDAR GT at sweep start times
  <world>/<traj>/<rig>-<noise>-<shutter>.gen.log      generator log (with timing)
  <world>/<traj>/gt_200hz.tum                         base_link GT at 200 Hz

The noise RNG seed depends only on (world, trajectory), so clean/noisy and
global/rolling variants of a sequence are paired. Needs mvsim-dataset-gen in
PATH (see README: it must include the timestamp rounding fix).

Usage: tools/generate.py [--jobs 5] [--cores-per-job 8] [--first-core 0]
                         [--worlds a,b] [--variants os1_32-noisy-rolling,...] [--force]
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
import zlib
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from pathlib import Path
from queue import Queue

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))

from mvsim_lio import densify  # noqa: E402

# Default variant set: all four (noise x shutter) combinations for the main
# rig, and the realistic one for the second rig.
VARIANTS = ['os1_32-clean-global', 'os1_32-clean-rolling', 'os1_32-noisy-global',
            'os1_32-noisy-rolling', 'vlp16-noisy-rolling']


def seed_for(world, traj):
    return zlib.crc32(f'{world}/{traj}'.encode()) & 0x7fffffff


def list_sequences(worlds):
    out = []
    for w in worlds:
        for t in sorted((ROOT / 'worlds' / w / 'trajectories').glob('*.tum')):
            out.append((w, t.stem))
    return out


def run_one(world, traj, variant, out_dir, work_dir, cores, force):
    rig, noise, shutter = variant.split('-')
    odir = out_dir / world / traj
    odir.mkdir(parents=True, exist_ok=True)
    rawlog = odir / f'{variant}.rawlog'
    log = odir / f'{variant}.gen.log'
    if rawlog.exists() and log.exists() and 'Wrote vehicle ground truth' in log.read_text() \
            and not force:
        return f'{world}/{traj}/{variant}: exists, skipped'
    dense = work_dir / world / f'{traj}.dense.tum'
    cmd = ['taskset', '-c', cores, 'mvsim-dataset-gen', str(ROOT / 'worlds' / world / 'world.xml'),
           '--trajectory', str(dense), '--trajectory-format', 'tum', '-o', str(rawlog),
           '--seed', str(seed_for(world, traj)), '--shutter', shutter]
    if noise == 'clean':
        cmd.append('--noiseless')
    env = dict(os.environ, MVSIM_LIO_RIG=str(ROOT / 'rigs' / f'{rig}.rig.xml'),
               TBB_NUM_THREADS=str(len(expand(cores))))
    t0 = time.time()
    with open(log, 'w') as f:
        f.write('# ' + ' '.join(cmd) + '\n')
        f.flush()
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, env=env)
        dt = time.time() - t0
        f.write(f'\n# exit code {r.returncode}, wall time {dt:.1f} s\n')
    if r.returncode != 0:
        return f'{world}/{traj}/{variant}: FAILED (see {log})'
    return f'{world}/{traj}/{variant}: OK in {dt:.0f} s, {rawlog.stat().st_size / 1e6:.0f} MB'


def _call(job):
    fn, *args = job
    return fn(*args)


def expand(cores):
    out = []
    for part in cores.split(','):
        a, _, b = part.partition('-')
        out.extend(range(int(a), int(b or a) + 1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/mnt/datasets/mvsim_lio_benchmark')
    ap.add_argument('--jobs', type=int, default=5)
    ap.add_argument('--cores-per-job', type=int, default=8)
    ap.add_argument('--first-core', type=int, default=0)
    ap.add_argument('--worlds', default='')
    ap.add_argument('--trajs', default='', help='comma-separated trajectory names')
    ap.add_argument('--variants', default=','.join(VARIANTS))
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()

    if not shutil.which('mvsim-dataset-gen'):
        sys.exit('mvsim-dataset-gen not in PATH')
    worlds = a.worlds.split(',') if a.worlds else sorted(
        p.parent.name for p in (ROOT / 'worlds').glob('*/world.xml'))
    seqs = list_sequences(worlds)
    if a.trajs:
        seqs = [s for s in seqs if s[1] in a.trajs.split(',')]
    out_dir = Path(a.out)
    work_dir = out_dir / '.work'
    # Dense trajectories (shared by every variant) and 200 Hz ground truth:
    todo = []
    for w, t in seqs:
        (work_dir / w).mkdir(parents=True, exist_ok=True)
        (out_dir / w / t).mkdir(parents=True, exist_ok=True)
        src = ROOT / 'worlds' / w / 'trajectories' / f'{t}.tum'
        dense = work_dir / w / f'{t}.dense.tum'
        if a.force or not dense.exists() or dense.stat().st_mtime < src.stat().st_mtime:
            todo.append((densify.write_dense, src, dense))
        todo.append((densify.write_gt, src, out_dir / w / t / 'gt_200hz.tum'))
    with ProcessPoolExecutor(max_workers=8) as ex:
        list(ex.map(_call, todo))

    pool = Queue()
    for j in range(a.jobs):
        c0 = a.first_core + j * a.cores_per_job
        pool.put(f'{c0}-{c0 + a.cores_per_job - 1}')

    def task(args):
        cores = pool.get()
        try:
            msg = run_one(*args, out_dir, work_dir, cores, a.force)
        finally:
            pool.put(cores)
        print(msg, flush=True)
        return msg

    jobs = [(w, t, v) for w, t in seqs for v in a.variants.split(',')]
    print(f'{len(jobs)} sequences to render with {a.jobs} jobs x {a.cores_per_job} cores',
          flush=True)
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        results = list(ex.map(task, jobs))
    failed = [r for r in results if 'FAILED' in r]
    print(f'DONE: {len(results) - len(failed)} ok, {len(failed)} failed')
    for r in failed:
        print(r)


if __name__ == '__main__':
    main()
