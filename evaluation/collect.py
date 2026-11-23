#!/usr/bin/env python3
"""Collects the batch outputs into $LIO_ROOT/results.

  results/traj_<variant>.tsv   trajectory metrics of every run (eval_traj.py)
  results/maps_<variant>.tsv   map metrics of every run with a <method>.map.json
  results/trajectories/<variant>.tar.xz   the estimated trajectories (TUM)

Usage: collect.py [--runs <dir>] [--no-tar]
"""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('LIO_ROOT', Path.home() / 'mvsim_lio_eval'))
RESULTS = ROOT / 'results'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', default=str(ROOT / 'runs'))
    ap.add_argument('--no-tar', action='store_true')
    a = ap.parse_args()
    runs = Path(a.runs)
    RESULTS.mkdir(exist_ok=True)
    for vdir in sorted(p for p in runs.iterdir() if p.is_dir() and not p.name.startswith('.')):
        v = vdir.name
        tmp = runs / f'.{v}.view'
        tmp.mkdir(exist_ok=True)
        link = tmp / v
        if not link.exists():
            link.symlink_to(vdir)
        subprocess.run([sys.executable, str(HERE / 'eval_traj.py'), '--runs', str(tmp), '-o',
                        str(RESULTS / f'traj_{v}.tsv')], check=True)
        recs = []
        keys = []
        for f in sorted(vdir.glob('*/*.map.json')):
            m = json.loads(f.read_text())
            world, traj = f.parent.name.split('__')
            recs.append((world, traj, f.name[:-len('.map.json')], m))
            keys += [k for k in m if k not in keys]
        rows = [[v, w, t, meth] + [m.get(k, '') for k in keys] for w, t, meth, m in recs]
        if rows:
            with open(RESULTS / f'maps_{v}.tsv', 'w') as fh:
                fh.write('\t'.join(['variant', 'world', 'traj', 'method'] + keys) + '\n')
                for r in rows:
                    fh.write('\t'.join(f'{x:.6g}' if isinstance(x, float) else str(x)
                                       for x in r) + '\n')
        if not a.no_tar:
            (RESULTS / 'trajectories').mkdir(exist_ok=True)
            tums = sorted(str(p.relative_to(runs)) for p in vdir.glob('*/*.tum'))
            if tums:
                subprocess.run(['tar', '-cJf', str(RESULTS / 'trajectories' / f'{v}.tar.xz'),
                                '-C', str(runs)] + tums, check=True)
        print(v, 'collected')


if __name__ == '__main__':
    main()
