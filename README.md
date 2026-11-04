# mvsim_lio_benchmark

A simulated LiDAR-inertial benchmark: 13 [MVSim](https://github.com/MRPT/mvsim)
worlds, 26 predefined 6-DoF trajectories, and the scripts that render them to
MRPT `.rawlog` datasets (rotating 3D LiDAR with per-point timestamps, 200 Hz
IMU, wheel odometry) with exact ground truth.

Uses:

1. Evaluation of LiDAR and LiDAR-inertial odometry and SLAM frameworks, on
   scenes that range from well constrained (streets, campus) to degenerate
   (tunnel, open field, greenhouse aisles), with controlled sensor noise and
   motion distortion.
2. Training data for learning-based methods (e.g. registration covariance
   models), with ground truth free of any error and geometry that covers
   walls, ceilings, stairs, ramps, terrain, vegetation, cars, racks and
   repetitive structure.

Every sequence is fully reproducible: worlds, trajectories and sensor
parameters are pinned in this repository, and the noise uses fixed seeds.

## Worlds

<!-- BEGIN worlds -->
<!-- END worlds -->

Screenshots are rendered by MVSim itself (`tools/screenshot.py`); indoor
worlds are shown as cutaways without their roofs or ceilings, which the
datasets do have. The plots show the top view of every primitive and the
trajectories (circle: start), and the height of `base_link` over time.

<!-- BEGIN gallery -->
<!-- END gallery -->

## Sequences

Every trajectory starts and ends with 3 s at rest (for IMU initialization).
"Carrier" is the platform the motion emulates (see below).

<!-- BEGIN sequences -->
<!-- END sequences -->

## Sensors and variants

Each trajectory is rendered with these five variants (files are named
`<rig>-<noise>-<shutter>`):

| variant | LiDAR | LiDAR noise | IMU noise | shutter |
|---|---|---|---|---|
| `os1_32-clean-global` | OS1-32 style | none | none | global |
| `os1_32-clean-rolling` | OS1-32 style | none | none | rolling |
| `os1_32-noisy-global` | OS1-32 style | 2 cm | MEMS (below) | global |
| `os1_32-noisy-rolling` | OS1-32 style | 2 cm | MEMS (below) | rolling |
| `vlp16-noisy-rolling` | VLP-16 style | 2 cm | MEMS (below) | rolling |

Rigs (`rigs/*.rig.xml`, sensor parameters pinned in the files):

| rig | rings | vertical FOV | columns | rate | range | range noise std |
|---|---|---|---|---|---|---|
| `os1_32` | 32, uniform | 45 deg (+-22.5) | 1024 | 10 Hz | 0.5 to 60 m | 0.02 m |
| `vlp16` | 16, uniform | 30 deg (+-15) | 1808 | 10 Hz | 0.4 to 100 m | 0.02 m |

- **Global shutter**: the whole sweep is cast from the pose at the sweep start;
  every point has `t = 0`.
- **Rolling shutter** (realistic spinning LiDAR): column `c` fires at
  `t = c * 0.1 / columns` s after the sweep start, from the pose at that
  instant. Points are given in the LiDAR frame *at their own firing time*
  (motion-skewed, as real drivers output them); the per-point `t` field lets a
  consumer de-skew them. Column 0 points backwards (azimuth -180 deg, -X) and
  the azimuth grows counter-clockwise (towards +Y).
- **IMU** (`imu1`): 200 Hz, angular velocity and specific force (proper
  acceleration, +9.81 m/s^2 on +Z at rest) in the IMU frame, from finite
  differences of a C2 spline of the trajectory. No orientation output. Noise in
  the noisy variants (per sample at 200 Hz, and bias random walk; biases start
  at zero):

  | | white noise (per sample) | density | bias random walk |
  |---|---|---|---|
  | gyroscope | 3.7e-3 rad/s | 2.6e-4 rad/s/sqrt(Hz) | 2.0e-5 rad/s/sqrt(s) |
  | accelerometer | 3.2e-2 m/s^2 | 2.3e-3 m/s^2/sqrt(Hz) | 1.0e-3 m/s^2/sqrt(s) |

- **Wheel odometry** (`odom`): planar odometry at 10 Hz derived from the
  ground truth (1% relative noise in the noisy variants). It is written for
  every carrier, including the handheld and legged ones, where it is not
  physically meaningful.
- **Intensity**: not generated (MVSim has no material model).

### Frames and extrinsics

All rotations between sensor frames are identity.

| frame | position in `base_link` |
|---|---|
| `lidar1` | (0, 0, 0.75) m |
| `imu1` | (0.05, -0.03, 0.45) m |

The ground truth is the pose of `base_link`, whose height above the ground
depends on the carrier platform the trajectory emulates:

| carrier | `base_link` above ground | LiDAR above ground | motion |
|---|---|---|---|
| `car` | 1.15 m | 1.90 m | up to 15 m/s, lateral acceleration <= 2.5 m/s^2, body tilted by the terrain under a 2.7 x 1.6 m wheelbase |
| `ugv` | 0.00 m | 0.75 m | up to 3 m/s, in-place turns and spins, tilted by the terrain under a 0.8 x 0.6 m footprint |
| `legged` | 0.35 m | 1.10 m | up to 1.8 m/s, gait bobbing (1.2 cm) and sway (1 deg), pitched along stairs |
| `handheld` | 0.75 m | 1.50 m | up to 1.8 m/s, gait bobbing (2.5 cm) and sway (1.5 deg), slow look-around yaw |

### Conventions

- World frame: Z up, gravity 9.81 m/s^2 along -Z. Units: meters, seconds,
  radians.
- Timestamps: seconds, starting at `t = 100 s` for every sequence (never 0,
  which some tools read as "unset"). LiDAR observations are stamped at the
  sweep start; per-point `t` is relative to that stamp.
- TUM files: `timestamp x y z qx qy qz qw` (the pose of the frame in the
  world).
- Trajectories are defined in `worlds/<world>/trajectories/<traj>.tum` (20 Hz
  knots, `base_link`); the generator interpolates them with a C2 spline
  (`tools/mvsim_lio/densify.py`).

## Files

Per sequence, under `<world>/<trajectory>/`:

| file | contents |
|---|---|
| `<variant>.rawlog` | MRPT rawlog (gzip): `CObservationPointCloud` (`lidar1`, a `CGenericPointsMap` with fields `x y z t ring`; `sensorPose` = `lidar1` in `base_link`), `CObservationIMU` (`imu1`), `CObservationOdometry` (`odom`), in time order |
| `<variant>.gt.tum` | `base_link` ground truth at the sweep start times (10 Hz) |
| `<variant>.lidar1.gt.tum` | `lidar1` ground truth at the sweep start times |
| `<variant>.gen.log` | generator command line and log |
| `gt_200hz.tum` | `base_link` ground truth at 200 Hz (same for every variant) |

Note that some odometry systems stamp a scan at its mid-sweep time (MOLA does
for rolling-shutter data, +0.05 s): compare against the ground truth at the
estimate's own timestamps (e.g. interpolating `gt_200hz.tum`), not by
nearest-stamp association with the 10 Hz files.

## How to regenerate

Requirements: Python 3 with numpy, scipy, matplotlib and Pillow, and a build
of MVSim's `mvsim-dataset-gen` that includes three fixes not yet in a release
(see [AGENTS.md](AGENTS.md)): timestamp rounding (without it the simulated
accelerometer is wrong by several m/s^2), IMU kinematics at the IMU mounting
pose (lever arm), and a heightfield speed-up (without it terrain worlds render
about 100x slower). `tools/check_imu.py <world> <traj>` checks the IMU of a
generated clean sequence against the trajectory.

```bash
export MVSIM_LIO_MVSIM_INSTALL=/path/to/mvsim/install   # colcon install space
source tools/env.sh
tools/build_worlds.py              # worlds/*/world.xml, trajectories, plots
tools/generate.py --jobs 5 --cores-per-job 8   # all sequences and variants
tools/screenshot.py                # worlds/*/screenshot*.png
tools/validate_mola.py --variant os1_32-noisy-rolling   # optional sanity check
tools/make_docs.py --manifest      # README tables, MANIFEST.tsv, SHA256SUMS
```

`tools/generate.py --worlds forest --trajs trail_loop --variants os1_32-noisy-rolling`
renders a single sequence. Outputs are byte-for-byte reproducible.

<!-- BEGIN gentime -->
<!-- END gentime -->

## Validation with MOLA LiDAR odometry

As a sanity check of data, timestamps, extrinsics and ground truth, and as a
first difficulty ranking, every sequence was run through MOLA LiDAR odometry
(`mola-lo-cli-rawlog`, default `lidar3d-default` pipeline, no loop closure).
Metrics are for the translation of `base_link`, with the ground truth
interpolated at the estimate timestamps: APE RMSE after SE(3) alignment, and
end-point drift (aligning only the first pose) as a percentage of the
distance traveled.

<!-- BEGIN validation -->
<!-- END validation -->

## Dataset manifest

Generated data size per world (all variants). The full list of files with
their sizes and SHA-256 checksums is in [MANIFEST.tsv](MANIFEST.tsv) and
[SHA256SUMS](SHA256SUMS) (`sha256sum -c SHA256SUMS` from the data directory).

<!-- BEGIN manifest -->
<!-- END manifest -->

## Download

TODO: the datasets will be published later; the location is not decided yet.

## Limitations

- Geometry is restricted to what `mvsim-dataset-gen` ray-traces exactly:
  vertical prisms (boxes, walls), cylinders, spheres, wedges (ramps, roofs),
  horizontal planes and heightfields. 3D mesh models (`<visual>` models) are
  not ray-traced, so trees are made of trunks, branches and clusters of small
  spheres, and cars of boxes and wheels.
- No glass: windows and doors are open gaps. No intensity, multi-echo,
  reflections, beam divergence or dropouts other than the maximum range.
- The scenes are static (no dynamic objects).
- The IMU has white noise and bias random walk, but no initial bias, scale
  factor or axis misalignment errors. All sensors share one clock (no time
  offsets).

## License

TODO (owner to decide): proposed BSD-3-Clause for the code and CC BY 4.0 for
the generated data.
