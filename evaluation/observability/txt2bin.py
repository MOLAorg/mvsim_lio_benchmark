#!/usr/bin/env python3
"""Filter for rawlog-to-text: lidar.txt on stdin -> binary records on stdout.

Record: x y z (float32, sensor frame), ring label (uint16); 16 bytes.
"""
import sys

import numpy as np
import pandas as pd

REC = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('ring', '<u2'), ('label', '<u2')])

out = sys.stdout.buffer
for chunk in pd.read_csv(sys.stdin, sep=' ', header=None, comment='#', chunksize=2_000_000,
                         names=['t', 'x', 'y', 'z', 'ring', 'label', 'inst'],
                         usecols=['x', 'y', 'z', 'ring', 'label']):
    rec = np.empty(len(chunk), REC)
    for k in ('x', 'y', 'z'):
        rec[k] = chunk[k].to_numpy(np.float32)
    rec['ring'] = chunk['ring'].to_numpy(np.uint16)
    rec['label'] = chunk['label'].to_numpy(np.uint16)
    out.write(rec.tobytes())
