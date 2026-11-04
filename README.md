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
| world | geometry and what it stresses | size | trajectories (carrier, duration) |
|---|---|---|---|
| [campus](worlds/campus/README.md) | A 500 x 400 m university campus on gently rolling ground (+-3 m): nine buildings (one with a courtyard, a 25 m tower), a central plaza with a fountain and benches, a tree-lined ring road with lamp posts, about 300 trees (most in a park), a car park, and a fenced sports field with goals and bleachers. Mixed structure and vegetation at large scale; long sequences with loops. | 500 x 400 m | `long_tour` (ugv, 15:13, 1796 m)<br>`ring_drive` (car, 6:19, 2792 m) |
| [forest](worlds/forest/README.md) | 200 x 200 m of hilly woodland (+-5 m, slopes up to 25%) with about 1350 trees (porous sphere-cluster crowns, branches, some conifers), bushes and fallen logs. Two cleared trails. Stresses unstructured, vegetation-like clutter with few planar surfaces, plus pitch/roll on slopes. | 200 x 200 m | `trail_loop` (legged, 5:40, 425 m)<br>`crossing` (ugv, 2:17, 195 m) |
| [greenhouse](worlds/greenhouse/README.md) | A 96 x 40 m multi-span greenhouse (5 spans of 8 m, gutters at 4.5 m, gable roofs up to 5.8 m seen from inside), a post grid every 4 m, trusses, and long crop rows: porous tomato-like hedges in three spans and benches with pots in two. Repetitive along the 87 m aisles, where every section looks the same; indoor with a sloped ceiling. | 110 x 52 m | `aisles` (ugv, 6:07, 414 m)<br>`walk` (handheld, 4:14, 265 m) |
| [office](worlds/office/README.md) | One office floor, 60 x 30 m with a 2.8 m ceiling: a 50 m long, 2.2 m wide corridor, 16 rooms behind 1.2 m doors (desks with legs, chairs, shelves, a meeting room), an open lobby with columns, and windows to an outside with a few trees and cars. Stresses the long-corridor degeneracy, very short ranges in rooms, door transitions and fast in-place rotations. | 80 x 50 m | `walk` (handheld, 3:53, 226 m)<br>`spins` (ugv, 2:39, 120 m) |
| [open_field](worlds/open_field/README.md) | 400 x 400 m of gently rolling grassland (+-4 m) with very few features: a dozen poles, a fence line, three lone trees, hay bales and a small shed. Stresses horizontal degeneracy: the ground constrains z, roll and pitch, but x, y and yaw rest on a handful of distant objects. | 400 x 400 m | `lap` (ugv, 6:33, 769 m)<br>`zigzag` (ugv, 4:42, 702 m) |
| [orchard](worlds/orchard/README.md) | 23 rows x 31 fruit trees (5 m between rows, 4 m within a row) on a gently sloped field (3%; ~4% of the trees missing), with trellis posts, a perimeter farm track, a hedge, a fence, a shed and a water tank. Highly repetitive along and across the rows (aliasing); the LiDAR sees trunks under the crowns. | 170 x 150 m | `rows` (ugv, 7:53, 696 m)<br>`headland` (ugv, 3:41, 525 m) |
| [parking_garage](worlds/parking_garage/README.md) | A three-level parking structure (64 x 40 m, levels at 0, 3.2 and 6.4 m, 2.9 m clearance) linked by two 20 m ramps (16% grade), with a column grid, parapets, an open roof level, ~60% occupied bays, and a few buildings and trees around it. Stresses multi-level geometry with ceilings, ramps (pitch), repetitive columns, and vertical drift between floors. | 120 x 90 m | `walk_down` (handheld, 4:01, 315 m)<br>`up_down` (car, 2:51, 565 m) |
| [parking_lot](worlds/parking_lot/README.md) | A 160 x 120 m open-air parking lot in front of a 120 m long supermarket: five double rows of perpendicular bays (~70% occupied by cars of four sizes), curbed end islands with trees, tall light poles, cart corrals, hedges and a fence. Repetitive rows of similar cars; large open aisles. | 160 x 127 m | `search` (car, 3:42, 934 m)<br>`ugv_loop` (ugv, 5:10, 569 m) |
| [rough_terrain](worlds/rough_terrain/README.md) | 260 x 260 m of steep, bumpy hills and valleys (+-10 m, slopes up to 26 deg, with 0.6 m bumps) with two craters, ~400 boulders of 0.3-1.6 m and a few dead trees. Stresses large, fast-changing pitch and roll, and a scene made almost only of terrain and rocks (no planes or poles). | 260 x 260 m | `hill_loop` (ugv, 6:58, 623 m)<br>`traverse_fast` (ugv, 1:57, 329 m) |
| [stairwell](worlds/stairwell/README.md) | A three-storey building (30 x 16 m, 3.5 m floor to floor) with a U-shaped stairwell (two flights of 10 steps, 17.5 cm risers, 28 cm treads, 32 deg, per storey, plus mid landings), a corridor and furnished rooms on each floor, slabs with ceilings, and windows. Stresses stairs (fast z changes with large pitch for a legged carrier), narrow spaces, ceilings and multi-floor ambiguity (identical floor layouts). | 50 x 32 m | `legged_floors` (legged, 4:48, 207 m)<br>`handheld_stairs` (handheld, 2:40, 118 m) |
| [tunnel](worlds/tunnel/README.md) | A 735 m road tunnel (straight, a 300 m radius 45 deg bend, straight), 10 m wide and 6.5 m high with a stepped arch, raised walkways, lamps every 10 m, signs every 100 m, emergency niches every 150 m and jet fan pairs every 200 m, with portals and a turnaround area outside. The classic degenerate case: almost nothing constrains motion along the axis. | 834 x 440 m | `car_through` (car, 2:28, 1782 m)<br>`ugv_inspection` (ugv, 5:28, 798 m) |
| [urban_street](worlds/urban_street/README.md) | A 3 x 3 grid of city blocks (~370 x 370 m) with 16 m wide streets, raised sidewalks with curbs, buildings 8-45 m tall with facade relief, a courtyard block, a park block, parked cars, street trees, lamp posts, benches, bins, bus stops, sign posts and traffic lights. Well-constrained, structured scene; the car route revisits streets (loops). | 370 x 370 m | `car_loops` (car, 5:48, 2935 m)<br>`pedestrian` (handheld, 8:23, 671 m) |
| [warehouse](worlds/warehouse/README.md) | A 90 x 60 m logistics hall, 10 m high with a ceiling and roof trusses, six double rows of 8 m pallet racks (5 levels, randomly filled) separated by 3.6 m aisles, loading docks with door openings, staging areas with pallets and an office cabin. Indoor and repetitive: the aisles look alike, with long, nearly identical views along them. | 110 x 80 m | `aisles` (ugv, 3:38, 410 m)<br>`walk` (handheld, 3:20, 260 m) |
<!-- END worlds -->

Screenshots are rendered by MVSim itself (`tools/screenshot.py`); indoor
worlds are shown as cutaways without their roofs or ceilings, which the
datasets do have. The plots show the top view of every primitive and the
trajectories (circle: start), and the height of `base_link` over time.

<!-- BEGIN gallery -->
### campus

<img src="worlds/campus/screenshot.png" width="49%"> <img src="worlds/campus/trajectories.png" width="40%">

### forest

<img src="worlds/forest/screenshot.png" width="49%"> <img src="worlds/forest/trajectories.png" width="40%">

### greenhouse

<img src="worlds/greenhouse/screenshot.png" width="49%"> <img src="worlds/greenhouse/trajectories.png" width="40%">

### office

<img src="worlds/office/screenshot.png" width="49%"> <img src="worlds/office/trajectories.png" width="40%">

### open_field

<img src="worlds/open_field/screenshot.png" width="49%"> <img src="worlds/open_field/trajectories.png" width="40%">

### orchard

<img src="worlds/orchard/screenshot.png" width="49%"> <img src="worlds/orchard/trajectories.png" width="40%">

### parking_garage

<img src="worlds/parking_garage/screenshot.png" width="49%"> <img src="worlds/parking_garage/trajectories.png" width="40%">

### parking_lot

<img src="worlds/parking_lot/screenshot.png" width="49%"> <img src="worlds/parking_lot/trajectories.png" width="40%">

### rough_terrain

<img src="worlds/rough_terrain/screenshot.png" width="49%"> <img src="worlds/rough_terrain/trajectories.png" width="40%">

### stairwell

<img src="worlds/stairwell/screenshot.png" width="49%"> <img src="worlds/stairwell/trajectories.png" width="40%">

### tunnel

<img src="worlds/tunnel/screenshot.png" width="49%"> <img src="worlds/tunnel/trajectories.png" width="40%">

### urban_street

<img src="worlds/urban_street/screenshot.png" width="49%"> <img src="worlds/urban_street/trajectories.png" width="40%">

### warehouse

<img src="worlds/warehouse/screenshot.png" width="49%"> <img src="worlds/warehouse/trajectories.png" width="40%">
<!-- END gallery -->

## Sequences

Every trajectory starts and ends with 3 s at rest (for IMU initialization).
"Carrier" is the platform the motion emulates (see below).

<!-- BEGIN sequences -->
| sequence | carrier | duration | length | max speed | max yaw rate | max pitch / roll | z range |
|---|---|---|---|---|---|---|---|
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
| `stairwell/legged_floors` | legged | 4:48 | 207 m | 1.0 m/s | 87 deg/s | 32 / 1 deg | 7.0 m |
| `stairwell/handheld_stairs` | handheld | 2:40 | 118 m | 1.2 m/s | 85 deg/s | 7 / 1 deg | 3.5 m |
| `tunnel/car_through` | car | 2:28 | 1782 m | 15.0 m/s | 32 deg/s | 0 / 0 deg | 0.0 m |
| `tunnel/ugv_inspection` | ugv | 5:28 | 798 m | 2.5 m/s | 2 deg/s | 0 / 0 deg | 0.0 m |
| `urban_street/car_loops` | car | 5:48 | 2935 m | 10.0 m/s | 47 deg/s | 0 / 0 deg | 0.0 m |
| `urban_street/pedestrian` | handheld | 8:23 | 671 m | 1.5 m/s | 90 deg/s | 5 / 2 deg | 0.2 m |
| `warehouse/aisles` | ugv | 3:38 | 410 m | 2.0 m/s | 82 deg/s | 0 / 0 deg | 0.0 m |
| `warehouse/walk` | handheld | 3:20 | 260 m | 1.4 m/s | 88 deg/s | 1 / 2 deg | 0.1 m |

Total: 26 trajectories, 130 minutes, per sensor variant.
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
Generation speed (simulated seconds per wall-clock second, 8 cores per job, Xeon E5-2696 v4 at 2.2 GHz), per world and variant:

| world | os1_32-clean-global | os1_32-clean-rolling | os1_32-noisy-global | os1_32-noisy-rolling | vlp16-noisy-rolling |
|---|---|---|---|---|---|
| campus | 4.7x | 4.7x | 4.7x | 4.5x | 4.4x |
| forest | 2.8x | 2.8x | 2.8x | 2.8x | 2.8x |
| greenhouse | 2.9x | 2.9x | 2.5x | 2.4x | 2.7x |
| office | 3.6x | 3.5x | 2.5x | 2.6x | 2.8x |
| open_field | 6.2x | 6.1x | 5.8x | 5.8x | 5.8x |
| orchard | 3.7x | 3.6x | 3.6x | 3.6x | 3.7x |
| parking_garage | 3.5x | 3.5x | 3.0x | 3.0x | 3.2x |
| parking_lot | 6.7x | 6.6x | 3.9x | 3.9x | 4.4x |
| rough_terrain | 4.9x | 4.9x | 4.8x | 4.9x | 4.7x |
| stairwell | 3.0x | 2.8x | 2.8x | 2.7x | 3.0x |
| tunnel | 5.1x | 5.1x | 3.1x | 3.1x | 3.3x |
| urban_street | 3.8x | 3.7x | 3.1x | 3.1x | 3.2x |
| warehouse | 2.6x | 2.6x | 2.3x | 2.3x | 2.5x |
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
| sequence | os1_32-clean-global: APE SE(3) / drift | os1_32-noisy-rolling: APE SE(3) / drift |
|---|---|---|
| `campus/long_tour` | 10.97 m / 1.66 % | 9.77 m / 1.72 % |
| `campus/ring_drive` | 22.91 m / 2.65 % | 38.73 m / 3.68 % |
| `forest/crossing` | 0.01 m / 0.44 % | 0.01 m / 0.41 % |
| `forest/trail_loop` | 0.24 m / 0.13 % | 3.63 m / 3.89 % |
| `greenhouse/aisles` | 0.00 m / 0.00 % | 0.02 m / 0.00 % |
| `greenhouse/walk` | 0.00 m / 0.00 % | 0.06 m / 0.00 % |
| `office/spins` | 0.05 m / 0.00 % | 0.11 m / 0.00 % |
| `office/walk` | 0.06 m / 0.00 % | 0.09 m / 0.00 % |
| `open_field/lap` | 13.79 m / 10.62 % | 14.21 m / 9.10 % |
| `open_field/zigzag` | 9.58 m / 4.07 % | 15.20 m / 6.69 % |
| `orchard/headland` | 0.31 m / 0.31 % | 0.75 m / 0.83 % |
| `orchard/rows` | 0.01 m / 0.02 % | 0.03 m / 0.07 % |
| `parking_garage/up_down` | 0.12 m / 0.00 % | 0.12 m / 0.01 % |
| `parking_garage/walk_down` | 0.13 m / 0.10 % | 0.23 m / 0.13 % |
| `parking_lot/search` | 0.01 m / 0.00 % | 0.14 m / 0.04 % |
| `parking_lot/ugv_loop` | 0.01 m / 0.00 % | 0.07 m / 0.05 % |
| `rough_terrain/hill_loop` | 1.21 m / 0.19 % | 1.65 m / 0.32 % |
| `rough_terrain/traverse_fast` | 0.14 m / 0.13 % | 0.22 m / 0.80 % |
| `stairwell/handheld_stairs` | 0.01 m / 0.00 % | 0.07 m / 0.00 % |
| `stairwell/legged_floors` | 0.13 m / 0.00 % | 0.12 m / 0.01 % |
| `tunnel/car_through` | 1.27 m / 0.03 % | 2.80 m / 0.29 % |
| `tunnel/ugv_inspection` | 8.83 m / 4.23 % | 11.98 m / 5.33 % |
| `urban_street/car_loops` | 0.02 m / 0.00 % | 1.16 m / 0.11 % |
| `urban_street/pedestrian` | 0.01 m / 0.00 % | 1.77 m / 1.18 % |
| `warehouse/aisles` | 0.00 m / 0.00 % | 0.01 m / 0.01 % |
| `warehouse/walk` | 0.01 m / 0.01 % | 0.12 m / 0.18 % |
<!-- END validation -->

Reading the table:

- With clean, global-shutter data, the structured worlds come out at the
  millimeter to centimeter level (e.g. `greenhouse/aisles` 1 mm,
  `warehouse/aisles` 2 mm, `urban_street/car_loops` 2 cm over 2.9 km). This
  confirms that the data, timestamps, extrinsics and ground truth agree.
- The sequences where the odometry fails also fail with clean data, so they
  fail because of the scene, not because of errors in the data:
  `open_field` (ground-dominated scans with few objects in range: the
  translation is underestimated from the first meters), `tunnel/ugv_inspection`
  (motion along the tunnel axis is unconstrained), and `campus` (the open lawn
  of the sports field, and the 400 m east-west straights of the ring road
  with only thin lamp posts and trees in range).
- `forest/trail_loop` drifts with the noisy sensor (3.9%) but not with the
  clean one (0.13%): range noise on porous canopies, combined with the legged
  gait.
- The greenhouse and orchard rows, although repetitive, turned out easy for
  this scan-to-map odometry.

First difficulty ranking (noisy rolling shutter, end-point drift):

| class | sequences |
|---|---|
| easy (< 0.1%) | `office/*`, `greenhouse/*`, `stairwell/*`, `warehouse/aisles`, `parking_garage/up_down`, `parking_lot/*`, `orchard/rows` |
| moderate (0.1% to 1.2%) | `urban_street/*`, `parking_garage/walk_down`, `warehouse/walk`, `tunnel/car_through`, `rough_terrain/*`, `forest/crossing`, `orchard/headland` |
| hard (> 1.5%) | `campus/*`, `forest/trail_loop`, `tunnel/ugv_inspection`, `open_field/*` |

## Dataset manifest

Generated data size per world (all variants). The full list of files with
their sizes and SHA-256 checksums is in [MANIFEST.tsv](MANIFEST.tsv) and
[SHA256SUMS](SHA256SUMS) (`sha256sum -c SHA256SUMS` from the data directory).

<!-- BEGIN manifest -->
| world | rawlogs | files | size |
|---|---|---|---|
| campus | 10 | 42 | 12.7 GB |
| forest | 10 | 42 | 7.2 GB |
| greenhouse | 10 | 42 | 9.4 GB |
| office | 10 | 42 | 5.5 GB |
| open_field | 10 | 42 | 5.5 GB |
| orchard | 10 | 42 | 8.9 GB |
| parking_garage | 10 | 42 | 5.3 GB |
| parking_lot | 10 | 42 | 4.4 GB |
| rough_terrain | 10 | 42 | 5.0 GB |
| stairwell | 10 | 42 | 6.9 GB |
| tunnel | 10 | 42 | 5.5 GB |
| urban_street | 10 | 42 | 10.6 GB |
| warehouse | 10 | 42 | 6.1 GB |
| **total** | 130 | 546 | **93.0 GB** |
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

- Code, world files, trajectories and sensor definitions in this repository:
  BSD 3-Clause, see [LICENSE](LICENSE).
- Generated datasets (`.rawlog`, `.gt.tum`): CC BY 4.0, see
  [DATA_LICENSE.md](DATA_LICENSE.md). Commercial use and model training are
  allowed with attribution.
