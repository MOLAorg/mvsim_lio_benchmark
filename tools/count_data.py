#!/usr/bin/env python3
"""Counts what each rendered rawlog contains (LiDAR sweeps and points, IMU and
odometry samples) and writes it next to the rawlog as <variant>.counts.json,
for the totals in the README (tools/make_docs.py).

Uses `bench-map-export --count-only` (tools/map_export; build it first).

Usage: tools/count_data.py [--data /mnt/datasets/mvsim_lio_benchmark]
                           [--tool build/map_export/bench-map-export] [--jobs 4] [--force]
"""
import argparse
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def count(tool, rawlog, force):
    out = rawlog.with_suffix('.counts.json')
    if out.exists() and not force:
        return f'{rawlog}: exists'
    log = rawlog.with_suffix('.gen.log')
    if log.exists() and 'Wrote vehicle ground truth' not in log.read_text():
        return f'{rawlog}: still rendering (or incomplete), skipped'
    r = subprocess.run([str(tool), '--rawlog', str(rawlog), '--count-only'],
                       capture_output=True, text=True)
    if r.returncode != 0:
        return f'{rawlog}: FAILED {r.stderr.strip()[-200:]}'
    counts = json.loads(r.stdout.strip().splitlines()[-1])
    out.write_text(json.dumps(counts) + '\n')
    return f'{rawlog}: {counts}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default='/mnt/datasets/mvsim_lio_benchmark')
    ap.add_argument('--tool', default=str(ROOT / 'build' / 'map_export' / 'bench-map-export'))
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    rawlogs = sorted(Path(a.data).glob('*/*/*.rawlog'))
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        for msg in ex.map(lambda f: count(a.tool, f, a.force), rawlogs):
            print(msg, flush=True)


if __name__ == '__main__':
    main()
