#!/usr/bin/env python3
"""Sequence difficulty classes, and results summarized per class.

Score of a sequence: the median, over every method run with its defaults, of the APE
as a percentage of the path length on the reference variant, capped at CAP_PCT (a
failed run counts as the cap; runs that did not fit in memory are left out). The 30
sequences are split into tertiles: easy, moderate, hard. The classes are computed
once and kept (results/difficulty.tsv); --reclassify computes them again.

  results/difficulty.tsv              world, traj, score_pct, n_methods, class
  results/summary_by_difficulty.tsv   per variant, method and class: runs, failures,
                                      median APE [m] and RTE10 [%] of the successful runs

Usage: difficulty.py [--reclassify]
"""
import argparse
import csv
import os
import statistics
from pathlib import Path

RESULTS = Path(os.environ.get('LIO_ROOT', Path.home() / 'mvsim_lio_eval')) / 'results'
REFERENCE = 'os1_32-clean-global'
CAP_PCT = 10.0
CLASSES = ('easy', 'moderate', 'hard')


def load(variant=None):
    rows = []
    for f in sorted(RESULTS.glob('traj_*.tsv')):
        with open(f) as fh:
            rows += [r for r in csv.DictReader(fh, delimiter='\t')
                     if variant is None or r['variant'] == variant]
    return rows


def classify():
    scores = {}
    for r in load(REFERENCE):
        if '+' in r['method'] or r['status'] == 'memory_limit':
            continue
        if r['status'] == 'ok' and r['ape'] and float(r['length_m']) > 0:
            pct = min(CAP_PCT, 100.0 * float(r['ape']) / float(r['length_m']))
        else:
            pct = CAP_PCT
        scores.setdefault((r['world'], r['traj']), []).append(pct)
    ranked = sorted(scores, key=lambda k: (statistics.median(scores[k]), k))
    n = len(ranked)
    with open(RESULTS / 'difficulty.tsv', 'w') as fh:
        fh.write('world\ttraj\tscore_pct\tn_methods\tclass\n')
        for i, k in enumerate(ranked):
            cls = CLASSES[min(len(CLASSES) - 1, i * len(CLASSES) // n)]
            fh.write(f'{k[0]}\t{k[1]}\t{statistics.median(scores[k]):.4f}\t'
                     f'{len(scores[k])}\t{cls}\n')


def classes():
    with open(RESULTS / 'difficulty.tsv') as fh:
        return {(r['world'], r['traj']): r['class'] for r in csv.DictReader(fh, delimiter='\t')}


def summarize():
    cls = classes()
    groups = {}
    for r in load():
        if r['status'] == 'memory_limit' or (r['world'], r['traj']) not in cls:
            continue
        groups.setdefault((r['variant'], r['method'], cls[(r['world'], r['traj'])]), []).append(r)
    with open(RESULTS / 'summary_by_difficulty.tsv', 'w') as fh:
        fh.write('variant\tmethod\tclass\tn_runs\tn_fail\tape_median\trte10_median\n')
        for (v, m, c) in sorted(groups, key=lambda g: (g[0], g[1], CLASSES.index(g[2]))):
            rs = groups[(v, m, c)]
            ok = [r for r in rs if r['status'] == 'ok']
            ape = statistics.median(float(r['ape']) for r in ok) if ok else float('nan')
            rte = statistics.median(float(r['rte10']) for r in ok) if ok else float('nan')
            fh.write(f'{v}\t{m}\t{c}\t{len(rs)}\t{len(rs) - len(ok)}\t{ape:.4f}\t{rte:.4f}\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reclassify', action='store_true')
    a = ap.parse_args()
    if a.reclassify or not (RESULTS / 'difficulty.tsv').exists():
        classify()
    summarize()


if __name__ == '__main__':
    main()
