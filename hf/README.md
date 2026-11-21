---
license: cc-by-4.0
pretty_name: MVSim-LIO
tags:
- robotics
- lidar
- imu
- lidar-inertial-odometry
- lidar-odometry
- slam
- point-cloud
- simulation
- semantic-labels
- mrpt
task_categories:
- robotics
size_categories:
- n<1K
viewer: false
---

# MVSim-LIO: a simulated LiDAR-inertial benchmark with exact ground truth

![Exact ground-truth maps and trajectories of six of the worlds](https://huggingface.co/datasets/MOLAorg/MVSim-LIO/resolve/main/assets/banner.gif)

<sub>Exact maps (noise-free sweeps at the true poses) revealed along the ground-truth trajectory; inset: the MVSim world.</sub>

15 [MVSim](https://github.com/MRPT/mvsim) worlds and 30 predefined 6-DoF trajectories
(20.1 km, 148 minutes) of five carriers (car, UGV, legged, hand-held, quadruped dog), rendered
as MRPT `.rawlog` datasets: a rotating 3D LiDAR with per-point timestamps, a 200 Hz IMU and
wheel odometry, with exact ground truth. Each trajectory is rendered with nine sensor variants
(270 sequences, 272 GB).

- **Exact ground truth**: the trajectory is the simulator's input (a C2 spline you can evaluate
  at any time), and the map is the world's analytic geometry. No GNSS/INS, total station or TLS
  registration error, no extrinsic calibration or time synchronization error.
- **Paired variants**: every trajectory is rendered with global and rolling shutter, and with
  three noise levels (noise-free, Gaussian, and a realistic LiDAR error model), sharing the same
  random seeds, so that each factor can be studied on its own.
- **Realistic sensor errors**: range noise growing with range, a beam model (footprint spread on
  oblique surfaces, mixed returns at depth edges), dropouts of weak returns at long range and
  grazing incidence, and per-beam calibration errors, with parameters drawn per sequence from
  ranges that bracket two real Ouster sensors; MEMS-grade IMU noise with bias random walk.
- **Per-point semantic and instance labels**.

Uses: evaluation of LiDAR and LiDAR-inertial odometry and SLAM, on scenes that range from well
constrained (streets, campus) to degenerate (tunnel, open field, greenhouse aisles), with
controlled sensor noise and motion distortion; and training data for learning-based methods,
with ground truth free of any error.

## Download

With the [Hugging Face CLI](https://huggingface.co/docs/huggingface_hub/guides/cli) (`hf`):

```bash
# Everything but the plain-text copy (272 GB):
hf download MOLAorg/MVSim-LIO --repo-type dataset --local-dir mvsim_lio --exclude "text/*"

# The plain-text copy of one variant, with its reader:
hf download MOLAorg/MVSim-LIO --repo-type dataset --local-dir mvsim_lio \
  --include "text/*/*/os1_32-real-rolling/*" --include "text/*/*/gt_200hz.tum.xz" \
  --include "text/README.md" --include "text/mvsim_lio_text.py"

# One variant, with the 200 Hz ground truth of every sequence:
hf download MOLAorg/MVSim-LIO --repo-type dataset --local-dir mvsim_lio \
  --include "*/*/os1_32-real-rolling.*" --include "*/*/gt_200hz.tum"

# One sequence, all its variants:
hf download MOLAorg/MVSim-LIO --repo-type dataset --local-dir mvsim_lio --include "office/spins/*"
```

Size of each variant (all 30 sequences):

| variant | size | variant | size |
|---|---|---|---|
| `os1_32-clean-global` | 21.5 GB | `os1_32-real-rolling` | 25.9 GB |
| `os1_32-clean-rolling` | 22.1 GB | `vlp16-noisy-rolling` | 24.0 GB |
| `os1_32-noisy-global` | 25.5 GB | `os1_64-noisy-rolling` | 50.6 GB |
| `os1_32-noisy-rolling` | 25.9 GB | `os1_64-real-rolling` | 50.5 GB |
| `os1_32-real-global` | 25.4 GB | `gt_200hz.tum` (shared) | 0.2 GB |

Each file's size and SHA-256 checksum are in `MANIFEST.tsv` and `SHA256SUMS`
(`sha256sum -c SHA256SUMS` in the download directory; with a partial download, check only the
lines of the files you have).

## Sequences

Every trajectory starts and ends with 3 s at rest (for IMU initialization).

| sequence | carrier | duration | length | max speed | max yaw rate | max pitch / roll | z range |
|---|---|---|---|---|---|---|---|
| `apartment_building/dog_floors` | dog | 7:32 | 273 m | 1.4 m/s | 118 deg/s | 29 / 2 deg | 6.0 m |
| `apartment_building/dog_stairs` | dog | 3:21 | 121 m | 1.6 m/s | 122 deg/s | 29 / 2 deg | 6.0 m |
| `campus/long_tour` | ugv | 15:13 | 1796 m | 2.0 m/s | 91 deg/s | 2 / 2 deg | 5.6 m |
| `campus/ring_drive` | car | 6:19 | 2792 m | 8.0 m/s | 47 deg/s | 2 / 2 deg | 4.2 m |
| `forest/trail_loop` | legged | 5:40 | 425 m | 1.5 m/s | 22 deg/s | 12 / 11 deg | 8.0 m |
| `forest/crossing` | ugv | 2:17 | 195 m | 1.5 m/s | 5 deg/s | 11 / 10 deg | 6.4 m |
| `greenhouse/aisles` | ugv | 6:07 | 414 m | 1.2 m/s | 82 deg/s | 0 / 0 deg | 0.0 m |
| `greenhouse/walk` | handheld | 4:14 | 265 m | 1.2 m/s | 93 deg/s | 1 / 1 deg | 0.0 m |
| `office/walk` | handheld | 3:53 | 226 m | 1.3 m/s | 96 deg/s | 1 / 2 deg | 0.1 m |
| `office/spins` | ugv | 2:39 | 120 m | 1.0 m/s | 179 deg/s | 0 / 0 deg | 0.0 m |
| `open_field/lap` | ugv | 6:33 | 769 m | 2.0 m/s | 66 deg/s | 5 / 4 deg | 7.2 m |
| `open_field/zigzag` | ugv | 4:42 | 702 m | 3.0 m/s | 169 deg/s | 5 / 5 deg | 7.4 m |
| `orchard/rows` | ugv | 7:53 | 696 m | 1.5 m/s | 82 deg/s | 3 / 3 deg | 4.5 m |
| `orchard/headland` | ugv | 3:41 | 525 m | 2.5 m/s | 82 deg/s | 2 / 3 deg | 5.5 m |
| `parking_garage/walk_down` | handheld | 4:01 | 315 m | 1.5 m/s | 96 deg/s | 10 / 2 deg | 6.5 m |
| `parking_garage/up_down` | car | 2:51 | 565 m | 3.5 m/s | 74 deg/s | 9 / 0 deg | 6.4 m |
| `parking_lot/search` | car | 3:42 | 934 m | 4.5 m/s | 47 deg/s | 0 / 0 deg | 0.0 m |
| `parking_lot/ugv_loop` | ugv | 5:10 | 569 m | 2.0 m/s | 143 deg/s | 0 / 0 deg | 0.0 m |
| `rough_terrain/hill_loop` | ugv | 6:58 | 623 m | 1.6 m/s | 72 deg/s | 22 / 19 deg | 14.1 m |
| `rough_terrain/traverse_fast` | ugv | 1:57 | 329 m | 3.2 m/s | 35 deg/s | 22 / 13 deg | 11.4 m |
| `stair_course/dog_course` | dog | 2:40 | 110 m | 1.5 m/s | 134 deg/s | 30 / 2 deg | 6.0 m |
| `stairwell/dog_floors` | dog | 3:49 | 163 m | 1.4 m/s | 100 deg/s | 30 / 2 deg | 7.0 m |
| `stairwell/legged_floors` | legged | 4:48 | 207 m | 1.0 m/s | 87 deg/s | 32 / 1 deg | 7.0 m |
| `stairwell/handheld_stairs` | handheld | 2:40 | 118 m | 1.2 m/s | 85 deg/s | 7 / 1 deg | 3.5 m |
| `tunnel/car_through` | car | 2:28 | 1782 m | 15.0 m/s | 32 deg/s | 0 / 0 deg | 0.0 m |
| `tunnel/ugv_inspection` | ugv | 5:28 | 798 m | 2.5 m/s | 2 deg/s | 0 / 0 deg | 0.0 m |
| `urban_street/car_loops` | car | 5:48 | 2935 m | 10.0 m/s | 47 deg/s | 0 / 0 deg | 0.0 m |
| `urban_street/pedestrian` | handheld | 8:23 | 671 m | 1.5 m/s | 90 deg/s | 5 / 2 deg | 0.2 m |
| `warehouse/aisles` | ugv | 3:38 | 410 m | 2.0 m/s | 82 deg/s | 0 / 0 deg | 0.0 m |
| `warehouse/walk` | handheld | 3:20 | 260 m | 1.4 m/s | 88 deg/s | 1 / 2 deg | 0.1 m |

The worlds: an apartment building and a stairwell building (stairs, identical floors), a
university campus, a forest on hills, a multi-span greenhouse, an office floor (a long
corridor), an open field with few features, an orchard, a three-level parking garage, an
open-air parking lot, rough terrain with boulders, a courtyard with straight and spiral stairs,
a 735 m road tunnel, a city grid, and a logistics warehouse with pallet racks.

## Sensors and variants

Files are named `<rig>-<noise>-<shutter>`:

| variant | LiDAR | LiDAR noise | IMU noise | shutter |
|---|---|---|---|---|
| `os1_32-clean-global` | OS1-32 style | none | none | global |
| `os1_32-clean-rolling` | OS1-32 style | none | none | rolling |
| `os1_32-noisy-global` | OS1-32 style | 2 cm | MEMS | global |
| `os1_32-noisy-rolling` | OS1-32 style | 2 cm | MEMS | rolling |
| `os1_32-real-global` | OS1-32 style | realistic error model | MEMS | global |
| `os1_32-real-rolling` | OS1-32 style | realistic error model | MEMS | rolling |
| `vlp16-noisy-rolling` | VLP-16 style | 2 cm | MEMS | rolling |
| `os1_64-noisy-rolling` | OS1-64 style | 2 cm | MEMS | rolling |
| `os1_64-real-rolling` | OS1-64 style | realistic error model | MEMS | rolling |

The `real` variants are the closest to a real sensor and the intended default for
benchmarking; the others isolate one factor at a time. The noise seed depends only on the
sequence, so the variants of a sequence are paired. The `os1_64` variants differ from their
`os1_32` counterparts only in the ring count.

| rig | rings | vertical FOV | columns | rate | range |
|---|---|---|---|---|---|
| `os1_32` | 32, uniform | 45 deg (+-22.5) | 1024 | 10 Hz | 0.5 to 60 m |
| `vlp16` | 16, uniform | 30 deg (+-15) | 1808 | 10 Hz | 0.4 to 100 m |
| `os1_64` | 64, uniform | 45 deg (+-22.5) | 1024 | 10 Hz | 0.5 to 60 m |

- **Realistic error model** (`real` variants, per ray, error model v2): a beam sampled by seven
  sub-rays, whose hits form echoes (one per surface, or merged across surfaces closer than the
  echo separation) and whose strongest echo is the return, so that mixed returns at depth edges
  follow the geometry; Gaussian range noise with std `sqrt(s0^2 + (k r)^2)`; the echo's range
  spread (footprint on oblique surfaces, depth of thin objects), capped; a detection range that
  drops weak returns at long range and grazing incidence; per-ring elevation, azimuth and range
  biases. The parameters are drawn per sequence from ranges that bracket two real Ouster
  sensors, and stored next to each rawlog as `<variant>.errors.json`. The first release of the
  `real` variants (another error model, with a grazing-angle cutoff and random mixed pixels) is
  kept under the tag `v1.0`: `hf download MOLAorg/MVSim-LIO --repo-type dataset --revision v1.0`.
  This release is tagged `v2.0`.
- **Global shutter**: the whole sweep is cast from the pose at the sweep start; every point has
  `t = 0`.
- **Rolling shutter**: column `c` fires at `t = c * 0.1 / columns` s after the sweep start, from
  the pose at that instant. Points are given in the LiDAR frame at their own firing time
  (motion-skewed, as real drivers output them); the per-point `t` field lets a consumer de-skew
  them.
- **IMU** (`imu1`): 200 Hz angular velocity and specific force (+9.81 m/s^2 on +Z at rest) in
  the IMU frame, no orientation output. Noise in the noisy and real variants:

  | | white noise (per sample) | density | bias random walk |
  |---|---|---|---|
  | gyroscope | 3.7e-3 rad/s | 2.6e-4 rad/s/sqrt(Hz) | 2.0e-5 rad/s/sqrt(s) |
  | accelerometer | 3.2e-2 m/s^2 | 2.3e-3 m/s^2/sqrt(Hz) | 1.0e-3 m/s^2/sqrt(s) |

- **Wheel odometry** (`odom`): planar odometry at 10 Hz derived from the ground truth (1%
  relative noise in the noisy variants). It is written for every carrier, including the
  hand-held, legged and dog ones, where it is not physically meaningful: leave it out for those,
  and whenever a LiDAR-only or LiDAR-inertial method is evaluated.
- **Intensity**: not generated.

Frames: all rotations between sensor frames are identity; `lidar1` is at (0, 0, 0.75) m and
`imu1` at (0.05, -0.03, 0.45) m in `base_link`. The ground truth is the pose of `base_link`.

Conventions: world frame Z up, gravity 9.81 m/s^2 along -Z; meters, seconds, radians.
Timestamps start at `t = 100 s` in every sequence. LiDAR observations are stamped at the sweep
start; the per-point `t` is relative to that stamp. TUM files: `timestamp x y z qx qy qz qw`.

## Semantic labels

Each point carries two `uint16` fields: `label`, the class of the surface its ray hit, and
`instance` (0 for extended surfaces such as roads, terrain, floors and walls). Mixed returns
are labeled `outlier`. Outdoor classes reuse the SemanticKITTI raw label ids; indoor ones start
at 100.

| id | class | id | class | id | class |
|---|---|---|---|---|---|
| 0 | unlabeled | 50 | building | 100 | floor |
| 1 | outlier | 51 | fence | 101 | ceiling |
| 10 | car | 52 | other-structure | 102 | wall |
| 20 | other-vehicle | 70 | vegetation | 103 | column |
| 40 | road | 71 | trunk | 104 | stairs |
| 44 | parking | 72 | terrain | 105 | table |
| 48 | sidewalk | 80 | pole | 106 | chair |
| 49 | other-ground | 81 | traffic-sign | 107 | shelf |
| | | 99 | other-object | 108 | other-furniture |
| | | | | 109 | goods |

## Files

Per sequence, under `<world>/<trajectory>/`:

| file | contents |
|---|---|
| `<variant>.rawlog` | MRPT rawlog (gzip): `CObservationPointCloud` (`lidar1`, a `CGenericPointsMap` with fields `x y z t ring label instance`; `sensorPose` = `lidar1` in `base_link`), `CObservationIMU` (`imu1`; `sensorPose` = `imu1` in `base_link`), `CObservationOdometry` (`odom`), in time order |
| `<variant>.gt.tum` | `base_link` ground truth at the sweep start times (10 Hz) |
| `<variant>.lidar1.gt.tum` | `lidar1` ground truth at the sweep start times |
| `<variant>.counts.json` | number of sweeps, points and IMU / odometry samples |
| `<variant>.gen.log` | generator command line and log |
| `<variant>.errors.json` | `real` variants only: the drawn parameters of the LiDAR error model |
| `gt_200hz.tum` | `base_link` ground truth at 200 Hz (the same for every variant) |

Some odometry systems stamp a scan at its mid-sweep time: compare against the ground truth at
the estimate's own timestamps (e.g. interpolating `gt_200hz.tum`), not by nearest-stamp
association with the 10 Hz files.

Rawlogs can be read with [MRPT](https://github.com/MRPT/mrpt) (C++ or Python) and run directly
with [MOLA](https://github.com/MOLAorg/mola) (`mola-lo-cli-rawlog`).

### Plain-text copy

Every rawlog also exists as plain-text tables compressed with xz, under
`text/<world>/<trajectory>/<variant>/`: `lidar.txt.xz` (one point per line,
`t_ns x y z ring label instance`), `sweeps.txt.xz`, `imu.txt.xz`, `odometry.txt.xz`,
`extrinsics.yaml` and copies of the ground truth files. They need neither MRPT nor ROS:
[`text/mvsim_lio_text.py`](text/mvsim_lio_text.py) reads them with Python and NumPy only, and
[`text/README.md`](text/README.md) describes the format. Point coordinates are rounded to 1 mm.

## Using the data from ROS 2

`rawlog2rosbag` (in [mrpt_ros_bridge](https://github.com/MRPT/mrpt_ros_bridge), branch `ros2`)
converts a sequence into a ROS 2 bag:

```bash
S=office/spins
rawlog2rosbag -i $S/os1_32-real-rolling.rawlog -o office_spins_os1_32-real-rolling \
  --gt-tum $S/gt_200hz.tum
```

| topic | type | contents |
|---|---|---|
| `/lidar1/points` | `sensor_msgs/PointCloud2` | one sweep per message (10 Hz), in the `lidar1` frame: `x y z t` (float32), `ring`, `label`, `instance` (uint16); `t` in seconds from the header stamp |
| `/imu1/imu` | `sensor_msgs/Imu` | 200 Hz angular velocity and specific force, in the `imu1` frame (no orientation) |
| `/odom/odom` | `nav_msgs/Odometry` | wheel odometry, `odom` to `base_link` (10 Hz) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `odom` to `base_link`; `base_link` to `lidar1` and `imu1` |
| `/ground_truth` | `nav_msgs/Odometry` | with `--gt-tum`: `base_link` in the `map` frame |

Stamps start at 100 s: play the bag with `ros2 bag play --clock` and run the nodes with
`use_sim_time:=true`.

## Limitations

- Geometry is made of what MVSim ray-traces exactly: boxes, walls, cylinders, spheres, wedges,
  planes and heightfields. Trees are trunks, branches and clusters of spheres; cars are boxes
  and wheels.
- No glass, intensity, multi-echo or reflections. The error model is geometric (no material
  model).
- The scenes are static.
- The IMU has white noise and bias random walk, but no initial bias, scale factor or axis
  misalignment. All sensors share one clock.

## License

[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/): you may share and adapt the data for
any purpose, including commercial use and model training, with attribution.

## Citation

A paper describing the benchmark is in preparation; its reference will be added here. Until
then, please cite this dataset page.
