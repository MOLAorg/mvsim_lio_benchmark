# MVSim-LIO: plain-text copy

The same data as the `.rawlog` files, in plain-text tables compressed with xz
(read by Python's standard `lzma` module, `xz -d`, 7-Zip and most archivers).
`mvsim_lio_text.py` reads them with Python 3 and NumPy only.

One folder per sequence variant, `text/<world>/<trajectory>/<variant>/`, and
the 200 Hz ground truth of each sequence, `text/<world>/<trajectory>/gt_200hz.tum.xz`.

| file | content |
|---|---|
| `lidar.txt.xz` | one LiDAR point per line: `t_ns x y z ring label instance` |
| `sweeps.txt.xz` | one sweep per line: `sweep stamp_s num_points` |
| `imu.txt.xz` | `stamp_s wx wy wz ax ay az` |
| `odometry.txt.xz` | `stamp_s x y yaw vx vy omega` |
| `extrinsics.yaml` | poses of the LiDAR and the IMU in `base_link` |
| `gt.tum`, `lidar1.gt.tum` | ground truth of `base_link` and of the LiDAR at the sweep stamps (TUM: `stamp_s x y z qx qy qz qw`) |
| `errors.json` | parameters of the LiDAR error model drawn for this sequence (`real` variants) |
| `counts.json` | numbers of sweeps, points, IMU and odometry readings |

Conventions:

- `lidar.txt` lists the sweeps one after another, in the order of
  `sweeps.txt`; sweep `k` has `num_points` lines. The first line of every
  table is a `#` header.
- Point coordinates are in meters, rounded to 1 mm, in the LiDAR frame at the
  point's own firing time `stamp_s + t_ns * 1e-9` (rolling shutter: `t_ns`
  grows with the column; global shutter: `t_ns = 0`). `ring` is the beam
  index, `label` the semantic class and `instance` the object
  instance (0: none), as in the rawlogs.
- `stamp_s`: seconds, 7 decimals, on the single clock of the simulation.
- IMU: angular velocity [rad/s] and specific force [m/s^2] in the IMU frame
  (at rest, `az` is about +9.8).
- Odometry: planar pose and velocity of `base_link` derived from the ground
  truth; meaningful only for wheeled carriers.

Differences from the rawlogs: point coordinates are rounded to 1 mm (0.3 mm
RMS) and `t_ns` to 1 ns; everything else is printed with at least 9
significant digits.

```python
import numpy as np
import mvsim_lio_text as m
d = 'text/office/spins/os1_32-real-rolling'
for k, stamp, pts in m.iter_sweeps(d):   # pts: structured array, one sweep
    xyz = np.c_[pts['x'], pts['y'], pts['z']]
imu = m.read_imu(d)
gt = m.read_tum(d + '/gt.tum')
```
