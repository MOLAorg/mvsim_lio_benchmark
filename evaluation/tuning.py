#!/usr/bin/env python3
"""Equal-budget tuning grid of configs/tuning.json.

  tuning.py methods [m1,m2,...]
      Comma list of the grid's "method+t<bits>" names, defaults excluded (for run_lio.py).
  tuning.py select [--tuning-runs DIR] [--runs DIR]
      Scores every combination on the training split and picks one per method:
      results/tuning_scores.tsv and configs/tuned.json.

Score of a combination: mean, over its training runs, of the APE as a percentage
of the path length, capped at the failure threshold (10 %); a failed run counts
as the cap. The defaults (t0000...) are the main batch's runs. Ties go to the
combination with fewer changes from the defaults.
"""
import argparse
import itertools
import json
import os
import sys
from pathlib import Path

import eval_traj

HERE = Path(__file__).resolve().parent
CFG = HERE / 'configs'
ROOT = Path(os.environ.get('LIO_ROOT', Path.home() / 'mvsim_lio_eval'))
RESULTS = ROOT / 'results'
CAP_PCT = 10.0


def grid():
    return json.loads((CFG / 'tuning.json').read_text())


def tags(spec):
    """Valid combinations of one method's factors, the defaults first."""
    n = len(spec['factors'])
    out = []
    for bits in itertools.product('01', repeat=n):
        on = {i for i, b in enumerate(bits) if b == '1'}
        if any(set(g) <= on for g in spec.get('exclusive', [])):
            continue
        out.append('t' + ''.join(bits))
    return out


def run_score(tum):
    """Capped APE [% of the path length] of one run, or None if it is not finished."""
    w, t = tum.parent.name.split('__')
    if tum.exists() and tum.stat().st_size > 0:
        r = eval_traj.score(w, t, tum)
        r['status'] = eval_traj.classify(r)
        if r['status'] != 'ok':
            return CAP_PCT, r['status']
        return min(CAP_PCT, 100.0 * r['ape'] / r['length_m']), 'ok'
    tm = tum.with_suffix('.time')
    if tum.with_suffix('.timeout').exists():
        return CAP_PCT, 'timeout'
    if tum.with_suffix('.memlimit').exists():
        from run_lio import retries_exhausted
        return (CAP_PCT, 'memory_limit') if retries_exhausted(tum) else None
    if tm.exists() and tm.stat().st_size > 0 and 'signal 15' not in tm.read_text():
        return CAP_PCT, 'no_output'
    return None


def select(a):
    g = grid()
    rows = []
    tuned = {}
    for method, spec in g['methods'].items():
        best = None
        for tag in tags(spec):
            default = set(tag[1:]) == {'0'}
            name = method if default else f'{method}+{tag}'
            root = Path(a.runs) if default else Path(a.tuning_runs)
            scores = []
            fails = 0
            missing = 0
            for v in g['train_variants']:
                if method == 'trajlo' and v.endswith('-global'):
                    continue
                for s in g['train_seqs']:
                    r = run_score(root / v / s.replace('/', '__') / f'{name}.tum')
                    if r is None:
                        missing += 1
                        continue
                    scores.append(r[0])
                    fails += r[1] != 'ok'
            on = [f['name'] for f, b in zip(spec['factors'], tag[1:]) if b == '1']
            score = sum(scores) / len(scores) if scores and not missing else float('nan')
            rows.append([method, tag, '+'.join(on) or 'default', len(scores), missing, fails,
                         f'{score:.3f}'])
            if missing:
                continue
            key = (score, len(on))
            if best is None or key < best[0]:
                best = (key, tag)
        if best:
            tuned[method] = best[1]
    lines = ['method\ttag\tfactors\tn_runs\tmissing\tn_fail\tscore_pct']
    lines += ['\t'.join(map(str, r)) for r in rows]
    (RESULTS / 'tuning_scores.tsv').write_text('\n'.join(lines) + '\n')
    (CFG / 'tuned.json').write_text(json.dumps(tuned, indent=1) + '\n')
    print('\n'.join(lines))
    print(json.dumps(tuned))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('methods')
    m.add_argument('only', nargs='?', default='')
    s = sub.add_parser('select')
    s.add_argument('--runs', default=str(ROOT / 'runs'))
    s.add_argument('--tuning-runs', default=str(ROOT / 'tuning'))
    a = ap.parse_args()
    if a.cmd == 'methods':
        only = [x for x in a.only.split(',') if x]
        names = [f'{k}+{t}' for k, spec in grid()['methods'].items()
                 if not only or k in only for t in tags(spec)[1:]]
        print(','.join(names))
    else:
        select(a)


if __name__ == '__main__':
    sys.exit(main())
