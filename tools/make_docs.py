#!/usr/bin/env python3
"""Regenerates the documentation derived from the worlds and the data:

  worlds/<name>/README.md          per-world description and trajectory table
  README.md                        the tables between <!-- BEGIN x --> / <!-- END x -->
                                   markers (worlds, sequences, gallery, manifest,
                                   validation)
  MANIFEST.tsv, SHA256SUMS         every generated file: size and sha256
                                   (only with --manifest; hashing takes a while)

Usage: tools/make_docs.py [--manifest] [--data /mnt/datasets/mvsim_lio_benchmark]
"""
import argparse
import hashlib
import json
import re
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))

from build_worlds import load_world_module  # noqa: E402

VARIANT_ORDER = ['os1_32-clean-global', 'os1_32-clean-rolling', 'os1_32-noisy-global',
                 'os1_32-noisy-rolling', 'vlp16-noisy-rolling']


def world_names():
    return sorted(p.parent.name for p in (ROOT / 'worlds').glob('*/world.py'))


def first_paragraph(doc):
    return ' '.join(doc.strip().split('\n\n')[0].split())


def describe(name):
    mod = load_world_module(name)
    w = mod.make_world()
    stats = json.loads((ROOT / 'worlds' / name / 'trajectories' / 'stats.json').read_text())
    x0, x1, y0, y1 = w.extent
    doc = first_paragraph(mod.__doc__)
    head, _, rest = doc.partition(': ')
    desc = rest or doc
    desc = desc[:1].upper() + desc[1:]
    return dict(name=name, doc=desc, size=f'{x1 - x0:.0f} x {y1 - y0:.0f} m',
                prims=w.n_prims, stats=stats, mod=mod)


def fmt_dur(s):
    return f'{int(s // 60)}:{int(round(s % 60)):02d}'


def world_readme(d):
    name = d['name']
    shots = sorted((ROOT / 'worlds' / name).glob('screenshot*.png'))
    lines = [f'# {name}', '', d['doc'], '',
             f'Extent of the plotted area: {d["size"]}; {d["prims"]} ray-traced primitives.', '']
    for s in shots:
        lines.append(f'![{name} {s.stem}]({s.name})')
    lines += ['', '![trajectories](trajectories.png)', '',
              '| trajectory | carrier | duration | length | max speed | max pitch / roll | z range | description |',
              '|---|---|---|---|---|---|---|---|']
    for t, s in d['stats'].items():
        lines.append(f'| `{t}` | {s["platform"]} | {fmt_dur(s["duration"])} | {s["length"]:.0f} m | '
                     f'{s["vmax"]:.1f} m/s | {s["pitch_max_deg"]:.0f} / {s["roll_max_deg"]:.0f} deg | '
                     f'{s["z_range"]:.1f} m | {s["description"]} |')
    lines += ['', 'Regenerate only this world:', '', '```bash',
              f'tools/build_worlds.py {name}', f'tools/generate.py --worlds {name}', '```', '']
    (ROOT / 'worlds' / name / 'README.md').write_text('\n'.join(lines))


def replace_block(text, key, body):
    pat = re.compile(rf'(<!-- BEGIN {key} -->\n).*?(<!-- END {key} -->)', re.S)
    if not pat.search(text):
        raise KeyError(key)
    return pat.sub(lambda m: m.group(1) + body.rstrip() + '\n' + m.group(2), text)


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 22), b''):
            h.update(chunk)
    return h.hexdigest()


def human(n):
    for u in ('B', 'KB', 'MB', 'GB', 'TB'):
        if n < 1024 or u == 'TB':
            return f'{n:.1f} {u}' if u != 'B' else f'{n} B'
        n /= 1024.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', action='store_true')
    ap.add_argument('--data', default='/mnt/datasets/mvsim_lio_benchmark')
    a = ap.parse_args()
    data = Path(a.data)
    descs = [describe(n) for n in world_names()]
    for d in descs:
        world_readme(d)

    readme = (ROOT / 'README.md').read_text()
    # Worlds table
    rows = ['| world | geometry and what it stresses | size | trajectories (carrier, duration) |',
            '|---|---|---|---|']
    for d in descs:
        tr = '<br>'.join(f'`{t}` ({s["platform"]}, {fmt_dur(s["duration"])}, {s["length"]:.0f} m)'
                         for t, s in d['stats'].items())
        rows.append(f'| [{d["name"]}](worlds/{d["name"]}/README.md) | {d["doc"]} | {d["size"]} | {tr} |')
    readme = replace_block(readme, 'worlds', '\n'.join(rows))
    # Gallery: first screenshot + trajectory plot per world
    g = []
    for d in descs:
        g.append(f'### {d["name"]}\n')
        g.append(f'<img src="worlds/{d["name"]}/screenshot.png" width="49%"> '
                 f'<img src="worlds/{d["name"]}/trajectories.png" width="40%">\n')
    readme = replace_block(readme, 'gallery', '\n'.join(g))
    # Sequences table with motion statistics
    rows = ['| sequence | carrier | duration | length | max speed | max yaw rate | max pitch / roll | z range |',
            '|---|---|---|---|---|---|---|---|']
    total = 0.0
    for d in descs:
        for t, s in d['stats'].items():
            total += s['duration']
            rows.append(f'| `{d["name"]}/{t}` | {s["platform"]} | {fmt_dur(s["duration"])} | '
                        f'{s["length"]:.0f} m | {s["vmax"]:.1f} m/s | {s["yawrate_max_deg"]:.0f} deg/s | '
                        f'{s["pitch_max_deg"]:.0f} / {s["roll_max_deg"]:.0f} deg | {s["z_range"]:.1f} m |')
    rows.append(f'\nTotal: {sum(len(d["stats"]) for d in descs)} trajectories, '
                f'{total / 60:.0f} minutes, per sensor variant.')
    readme = replace_block(readme, 'sequences', '\n'.join(rows))

    if a.manifest:
        files = sorted(p for p in data.glob('*/*/*') if p.is_file() and p.suffix in
                       ('.rawlog', '.tum', '.log') and not p.relative_to(data).parts[0]
                       .startswith('.'))
        with ThreadPoolExecutor(max_workers=16) as ex:
            hashes = list(ex.map(sha256, files))
        with open(ROOT / 'MANIFEST.tsv', 'w') as f:
            f.write('path\tbytes\tsha256\n')
            for p, h in zip(files, hashes):
                f.write(f'{p.relative_to(data)}\t{p.stat().st_size}\t{h}\n')
        with open(ROOT / 'SHA256SUMS', 'w') as f:
            for p, h in zip(files, hashes):
                f.write(f'{h}  {p.relative_to(data)}\n')
    if (ROOT / 'MANIFEST.tsv').exists():
        per = {}
        for line in (ROOT / 'MANIFEST.tsv').read_text().splitlines()[1:]:
            path, size, _ = line.split('\t')
            w = path.split('/')[0]
            n, b, r = per.get(w, (0, 0, 0))
            per[w] = (n + 1, b + int(size), r + (1 if path.endswith('.rawlog') else 0))
        rows = ['| world | rawlogs | files | size |', '|---|---|---|---|']
        for w, (n, b, r) in sorted(per.items()):
            rows.append(f'| {w} | {r} | {n} | {human(b)} |')
        tb = sum(v[1] for v in per.values())
        rows.append(f'| **total** | {sum(v[2] for v in per.values())} | '
                    f'{sum(v[0] for v in per.values())} | **{human(tb)}** |')
        readme = replace_block(readme, 'manifest', '\n'.join(rows))
    # Generation time, from the generator logs
    gt = {}
    for log in sorted(data.glob('*/*/*.gen.log')):
        m = re.search(r'wall time ([0-9.]+) s', log.read_text()[-300:])
        if not m:
            continue
        w, t = log.parent.parent.name, log.parent.name
        v = log.name[:-len('.gen.log')]
        dur = next(d['stats'][t]['duration'] for d in descs if d['name'] == w)
        gt.setdefault(w, {}).setdefault(v, []).append((float(m.group(1)), dur))
    if gt:
        # Worlds whose generator logs are gone keep their previous rows.
        old_rows = {}
        m = re.search(r'<!-- BEGIN gentime -->\n(.*?)<!-- END gentime -->', readme, re.S)
        for line in (m.group(1).splitlines() if m else []):
            cells = [c.strip() for c in line.strip('|').split('|')]
            if line.startswith('| ') and cells[0] not in ('world',) and cells[0] not in gt:
                old_rows[cells[0]] = line
        variants = list(VARIANT_ORDER)
        rows = ['Generation speed (simulated seconds per wall-clock second, 8 cores per job, '
                'Xeon E5-2696 v4 at 2.2 GHz, on a shared machine), per world and variant:', '',
                '| world | ' + ' | '.join(variants) + ' |', '|---|' + '---|' * len(variants)]
        for w in sorted(set(gt) | set(old_rows)):
            if w not in gt:
                rows.append(old_rows[w])
                continue
            cells = []
            for v in variants:
                runs = gt[w].get(v, [])
                cells.append(f'{sum(r[1] for r in runs) / sum(r[0] for r in runs):.1f}x'
                             if runs else '')
            rows.append(f'| {w} | ' + ' | '.join(cells) + ' |')
        readme = replace_block(readme, 'gentime', '\n'.join(rows))
    vfile = data / 'validation' / 'summary.json'
    if vfile.exists():
        res = json.loads(vfile.read_text())
        # The table shows the two variants validated for every sequence; other
        # variants validated for single files stay in summary.json.
        variants = ['os1_32-clean-global', 'os1_32-noisy-rolling']
        rows = ['| sequence | ' + ' | '.join(f'{v}: APE SE(3) / drift' for v in variants) + ' |',
                '|---|' + '---|' * len(variants)]
        seqs = sorted({'/'.join(k.split('/')[:2]) for k in res})
        for s in seqs:
            cells = []
            for v in variants:
                r = res.get(f'{s}/{v}')
                if not r:
                    cells.append('')
                elif 'error' in r:
                    cells.append(r['error'])
                else:
                    cells.append(f'{r["ape_se3"]:.2f} m / {r["drift_pct"]:.2f} %')
            rows.append(f'| `{s}` | ' + ' | '.join(cells) + ' |')
        readme = replace_block(readme, 'validation', '\n'.join(rows))
    (ROOT / 'README.md').write_text(readme)


if __name__ == '__main__':
    main()
