# bench-map-export

Point-cloud map of a sequence from its rawlog and a trajectory (TUM,
`base_link` in the world frame). Every LiDAR point is placed with the
trajectory interpolated at its own firing time (sweep stamp + per-point `t`),
so rolling-shutter sweeps are de-skewed by that same trajectory. Output:
little-endian float32 records `(x, y, z, t_first)`, one per occupied voxel
(the centroid of its points and the time it was first seen, from the
trajectory start).

- With `gt_200hz.tum` and a `clean` variant: the exact reference map of what
  the sensor saw.
- With an estimated trajectory (optionally aligned with `--align`, 3x4
  row-major): the map that the estimate implies, to compare against the
  reference.

Build (needs MRPT, e.g. from `source tools/env.sh`):

```bash
cmake -S tools/map_export -B build/map_export -DCMAKE_BUILD_TYPE=Release
cmake --build build/map_export
build/map_export/bench-map-export --rawlog <data>/forest/crossing/os1_32-clean-global.rawlog \
    --traj <data>/forest/crossing/gt_200hz.tum --out forest_crossing.bin --voxel 0.1
```

Options: `--voxel` [m] (0.1), `--every N` (use every N-th sweep, 1),
`--label` (lidar1), `--offset` [s] added to the sweep stamps, `--align`,
`--no-deskew`, `--t-end` (stop after this sweep stamp).
