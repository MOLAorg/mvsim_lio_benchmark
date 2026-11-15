#!/usr/bin/env python3
"""Symlink tree of the hosted files, for an upload to the Hugging Face Hub.

  tools/hf_stage.py --out DIR [--variants a,b] [--data /mnt/datasets/mvsim_lio_benchmark]
                    [--manifest]

Links every file of MANIFEST.tsv that belongs to the given variants (all of them if omitted),
the files every variant shares (gt_200hz.tum), and the dataset card (hf/README.md) with its
banner. --manifest also links MANIFEST.tsv and SHA256SUMS. Then:

  hf upload MOLAorg/MVSim-LIO DIR . --repo-type dataset
"""
import argparse
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def link(src, dst):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.is_symlink() or dst.exists():
        dst.unlink()
    os.symlink(src, dst)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--variants', default='')
    ap.add_argument('--data', default='/mnt/datasets/mvsim_lio_benchmark')
    ap.add_argument('--manifest', action='store_true')
    a = ap.parse_args()
    out = Path(a.out)
    data = Path(a.data)
    wanted = {v for v in a.variants.split(',') if v}
    n = 0
    size = 0
    for line in (ROOT / 'MANIFEST.tsv').read_text().splitlines()[1:]:
        path, nbytes, _ = line.split('\t')
        variant = Path(path).name.split('.')[0]
        shared = '-' not in variant
        if wanted and not shared and variant not in wanted:
            continue
        f = data / path
        if f.stat().st_size != int(nbytes):
            raise SystemExit(f'{f}: size differs from MANIFEST.tsv')
        link(f, out / path)
        n += 1
        size += int(nbytes)
    link(ROOT / 'hf' / 'README.md', out / 'README.md')
    link(ROOT / 'docs' / 'banner.gif', out / 'assets' / 'banner.gif')
    if a.manifest:
        for name in ('MANIFEST.tsv', 'SHA256SUMS'):
            link(ROOT / name, out / name)
    print(f'{n} data files ({size / 1e9:.1f} GB) linked under {out}')


if __name__ == '__main__':
    main()
