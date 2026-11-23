#!/usr/bin/env python3
"""Reference and estimated point-cloud maps, and their map metrics.

  make_maps.py ref   [--seqs w/t,...]
      Reference map per sequence: os1_32-clean-global sweeps at the true
      poses (gt_200hz.tum), every sweep, 10 cm voxels.
  make_maps.py est   --variants v1,v2 --methods m1,m2 [--seqs ...]
      Map implied by each estimated trajectory: the variant's own sweeps (every
      sweep, as for the reference), each point placed with the estimate
      (SE(3)-aligned to the ground truth as in eval_traj.py) at its firing time;
      then map metrics against the reference, written to
      <runs>/<variant>/<w>__<t>/<method>.map.json.

Jobs run one core each, from --cores (re-read from the cores file).
"""
import argparse
import json
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from queue import Queue

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import eval_traj  # noqa: E402
import run_lio  # noqa: E402
import map_metrics  # noqa: E402

DATA = run_lio.DATA
OUT = run_lio.EFFORT_ROOT
TOOL = OUT / 'build' / 'map_export' / 'bench-map-export'
ENV = run_lio.ENV_SETUP + ' && '
VOXEL = 0.1


def export(core, rawlog, traj, out, every=1, align=None):
    cmd = (f'{ENV} taskset -c {core} nice -n 10 {TOOL} --rawlog {rawlog} --traj {traj} '
           f'--out {out}.tmp --voxel {VOXEL} --every {every}')
    if align is not None:
        cmd += ' --align "' + ' '.join(f'{v:.12g}' for v in align[:3, :].ravel()) + '"'
    r = subprocess.run(['bash', '-c', cmd + f' && mv {out}.tmp {out}'],
                       capture_output=True, text=True)
    return r.returncode == 0, (r.stdout + r.stderr).strip()[-300:]


def ref_job(core, seq):
    world, traj = seq.split('/')
    out = OUT / 'maps' / 'ref' / f'{world}__{traj}.bin'
    if out.exists():
        return f'{seq}: exists'
    out.parent.mkdir(parents=True, exist_ok=True)
    ok, msg = export(core, DATA / seq / 'os1_32-clean-global.rawlog', DATA / seq / 'gt_200hz.tum',
                     out)
    return f'{seq}: {"OK" if ok else "FAILED"} {msg}'


def alignment(world, traj, tum):
    """SE(3) that maps the estimate onto the ground truth (as in eval_traj)."""
    t, E = eval_traj.load_tum(tum)
    t0, pos, rot = eval_traj.gt_spline(world, traj)
    ok = (t >= t0[0]) & (t <= t0[-1])
    G = eval_traj.to_T(pos(t[ok]), rot(t[ok]).as_quat())
    return eval_traj.umeyama(E[ok][:, :3, 3], G[:, :3, 3])


def est_job(core, variant, seq, method, runs, every):
    world, traj = seq.split('/')
    d = Path(runs) / variant / f'{world}__{traj}'
    d.mkdir(parents=True, exist_ok=True)
    # Pseudo-method "gt": the variant's own sweeps at the true poses, i.e. the
    # floor that the sensor errors alone put on the map metrics.
    tum = DATA / seq / 'gt_200hz.tum' if method == 'gt' else d / f'{method}.tum'
    res = d / f'{method}.map.json'
    if res.exists():
        return f'{variant}/{seq}/{method}: exists'
    if not tum.exists() or tum.stat().st_size == 0:
        return f'{variant}/{seq}/{method}: no trajectory'
    ref = OUT / 'maps' / 'ref' / f'{world}__{traj}.bin'
    if not ref.exists():
        return f'{variant}/{seq}/{method}: no reference map'
    if method != 'gt':
        # Diverged or partial runs: no map metrics (a scattered map would also
        # need unbounded memory).
        r = eval_traj.score(world, traj, tum)
        status = eval_traj.classify(r)
        if status != 'ok':
            res.write_text(json.dumps({'status': status}) + '\n')
            return f'{variant}/{seq}/{method}: {status}, no map metrics'
    try:
        A = np.eye(4) if method == 'gt' else alignment(world, traj, tum)
    except Exception as e:  # noqa: BLE001
        return f'{variant}/{seq}/{method}: alignment failed ({e})'
    out = OUT / 'maps' / 'est' / variant / f'{world}__{traj}__{method}.bin'
    out.parent.mkdir(parents=True, exist_ok=True)
    ok, msg = export(core, DATA / seq / f'{variant}.rawlog', tum, out, every, A)
    if not ok:
        return f'{variant}/{seq}/{method}: FAILED {msg}'
    m = {'status': 'ok', **map_metrics.metrics(map_metrics.load(out), map_metrics.load(ref))}
    c = re.search(r'scans used: (\d+) points: (\d+)', msg)
    if c:
        m['scans'] = int(c.group(1))
        m['points'] = int(c.group(2))
    res.write_text(json.dumps(m, indent=1) + '\n')
    out.unlink()  # bulky; the metrics are what is kept
    return f'{variant}/{seq}/{method}: chamfer {m["chamfer"]:.3f} f0.1 {m["f_0.1"]:.3f}'


def all_seqs():
    return sorted(f'{p.parent.parent.name}/{p.parent.name}' for p in DATA.glob('*/*/gt_200hz.tum'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['ref', 'est'])
    ap.add_argument('--seqs', default='')
    ap.add_argument('--variants', default='')
    ap.add_argument('--methods', default='')
    ap.add_argument('--runs', default=str(OUT / 'runs'))
    ap.add_argument('--cores', default='71')
    ap.add_argument('--every', type=int, default=1)
    ap.add_argument('--mem-cap-gb', type=float, default=22.5)
    ap.add_argument('--job-mem-gb', type=float, default=2.0)
    ap.add_argument('--min-headroom-gb', type=float, default=15.0)
    a = ap.parse_args()
    seqs = [s for s in a.seqs.split(',') if s] or all_seqs()
    cores = Queue()
    for part in a.cores.split(','):
        lo, _, hi = part.partition('-')
        for c in range(int(lo), int(hi or lo) + 1):
            cores.put(c)
    n = cores.qsize()
    if a.mode == 'ref':
        jobs = [(ref_job, s) for s in seqs]
    else:
        jobs = [(est_job, v, s, m, a.runs, a.every) for v in a.variants.split(',')
                for s in seqs for m in a.methods.split(',')]

    def run(job):
        c = cores.get()
        try:
            # Shared memory budget with the LIO runner (same accounting).
            while (run_lio.batch_rss_gb(None) + a.job_mem_gb > a.mem_cap_gb
                   or run_lio.headroom_gb() < a.min_headroom_gb):
                time.sleep(15)
            t0 = time.time()
            msg = job[0](c, *job[1:])
            print(f'{msg} [{time.time() - t0:.0f}s]', flush=True)
        finally:
            cores.put(c)

    with ThreadPoolExecutor(max_workers=n) as ex:
        list(ex.map(run, jobs))
    print('ALL DONE', flush=True)


if __name__ == '__main__':
    main()
