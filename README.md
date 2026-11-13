# mvsim_lio_benchmark

<p align="center"><img src="docs/banner.gif" width="100%" alt="Exact ground-truth maps and trajectories of six of the worlds"></p>

<p align="center"><sub>Exact maps (noise-free sweeps at the true poses) revealed along the ground-truth trajectory; inset: the MVSim world. <a href="docs/banner.mp4">MP4 version</a>.</sub></p>

A simulated LiDAR-inertial benchmark: 15 [MVSim](https://github.com/MRPT/mvsim)
worlds, 30 predefined 6-DoF trajectories (20.1 km, 148 minutes) of five
carriers, and the scripts that render them to MRPT `.rawlog` datasets
(rotating 3D LiDAR with per-point timestamps, 200 Hz IMU, wheel odometry) with
exact ground truth.

- **Exact ground truth**: the trajectory is the simulator's input (a C2
  spline you can evaluate at any time), and the map is the world's analytic
  geometry. No GNSS/INS, total station or TLS registration error, no
  extrinsic calibration or time synchronization error.
- **Paired variants**: every trajectory is rendered with global and rolling
  shutter, and with three noise levels (noise-free, Gaussian, and a
  realistic LiDAR error model), sharing the same random seeds, so that each
  factor can be studied on its own.
- **Realistic sensor errors**: range noise growing with range, grazing-angle
  dropouts, beam divergence, mixed pixels at depth edges and per-beam
  calibration errors, with parameters drawn per sequence; MEMS-grade IMU
  noise with bias random walk.

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

## The dataset in numbers

<!-- BEGIN totals -->
| | |
|---|---|
| worlds | 15 |
| trajectories | 30, of 5 carriers (car, dog, handheld, legged, ugv) |
| trajectory length | 20.1 km (2.46 h) per variant |
| sensor variants | 9 |
| rendered sequences | 270 (181 km, 22.2 h of sensor data) |
| LiDAR sweeps | 798.02 thousand |
| LiDAR points | 24.66 billion |
| IMU samples | 15.96 million |
| data size | 264.5 GB |

Per variant:

| variant | sequences | LiDAR sweeps | LiDAR points | points per sweep | IMU samples |
|---|---|---|---|---|---|
| `os1_32-clean-global` | 30 | 88,669 | 2.32 billion | 26,118 | 1,772,920 |
| `os1_32-clean-rolling` | 30 | 88,669 | 2.32 billion | 26,118 | 1,772,920 |
| `os1_32-noisy-global` | 30 | 88,669 | 2.32 billion | 26,116 | 1,772,920 |
| `os1_32-noisy-rolling` | 30 | 88,669 | 2.32 billion | 26,116 | 1,772,920 |
| `os1_32-real-global` | 30 | 88,669 | 2.16 billion | 24,407 | 1,772,920 |
| `os1_32-real-rolling` | 30 | 88,669 | 2.16 billion | 24,407 | 1,772,920 |
| `vlp16-noisy-rolling` | 30 | 88,669 | 2.11 billion | 23,825 | 1,772,920 |
| `os1_64-noisy-rolling` | 30 | 88,669 | 4.63 billion | 52,249 | 1,772,920 |
| `os1_64-real-rolling` | 30 | 88,669 | 4.32 billion | 48,746 | 1,772,920 |
<!-- END totals -->

## Worlds

<!-- BEGIN worlds -->
| world | geometry and what it stresses | size | trajectories (carrier, duration) |
|---|---|---|---|
| [apartment_building](worlds/apartment_building/README.md) | A newly built three-storey apartment building (20 x 14 m, 3.0 m floor to floor, 2.75 m ceilings) with a dog-leg stair (two flights of 9 steps, 16.7 cm risers, 28 cm treads, a half landing and a central wall) in a central core, and two empty apartments per floor (living room, corridor, two bedrooms, kitchen, bathroom): bare walls, floors, ceilings, door and window openings, no furniture. Stresses stairs for a legged robot, identical floors (vertical ambiguity), small bare rooms with short ranges and door transitions. | 44 x 34 m | `dog_floors` (dog, 7:32, 273 m)<br>`dog_stairs` (dog, 3:21, 121 m) |
| [campus](worlds/campus/README.md) | A 500 x 400 m university campus on gently rolling ground (+-3 m): nine buildings (one with a courtyard, a 25 m tower), a central plaza with a fountain and benches, a tree-lined ring road with lamp posts, about 300 trees (most in a park), a car park, and a fenced sports field with goals and bleachers. Mixed structure and vegetation at large scale; long sequences with loops. | 500 x 400 m | `long_tour` (ugv, 15:13, 1796 m)<br>`ring_drive` (car, 6:19, 2792 m) |
| [forest](worlds/forest/README.md) | 200 x 200 m of hilly woodland (+-5 m, slopes up to 25%) with about 1350 trees (porous sphere-cluster crowns, branches, some conifers), bushes and fallen logs. Two cleared trails. Stresses unstructured, vegetation-like clutter with few planar surfaces, plus pitch/roll on slopes. | 200 x 200 m | `trail_loop` (legged, 5:40, 425 m)<br>`crossing` (ugv, 2:17, 195 m) |
| [greenhouse](worlds/greenhouse/README.md) | A 96 x 40 m multi-span greenhouse (5 spans of 8 m, gutters at 4.5 m, gable roofs up to 5.8 m seen from inside), a post grid every 4 m, trusses, and long crop rows: porous tomato-like hedges in three spans and benches with pots in two. Repetitive along the 87 m aisles, where every section looks the same; indoor with a sloped ceiling. | 110 x 52 m | `aisles` (ugv, 6:07, 414 m)<br>`walk` (handheld, 4:14, 265 m) |
| [office](worlds/office/README.md) | One office floor, 60 x 30 m with a 2.8 m ceiling: a 50 m long, 2.2 m wide corridor, 16 rooms behind 1.2 m doors (desks with legs, chairs, shelves, a meeting room), an open lobby with columns, and windows to an outside with a few trees and cars. Stresses the long-corridor degeneracy, very short ranges in rooms, door transitions and fast in-place rotations. | 80 x 50 m | `walk` (handheld, 3:53, 226 m)<br>`spins` (ugv, 2:39, 120 m) |
| [open_field](worlds/open_field/README.md) | 400 x 400 m of gently rolling grassland (+-4 m) with very few features: a dozen poles, a fence line, three lone trees, hay bales and a small shed. Stresses horizontal degeneracy: the ground constrains z, roll and pitch, but x, y and yaw rest on a handful of distant objects. | 400 x 400 m | `lap` (ugv, 6:33, 769 m)<br>`zigzag` (ugv, 4:42, 702 m) |
| [orchard](worlds/orchard/README.md) | 23 rows x 31 fruit trees (5 m between rows, 4 m within a row) on a gently sloped field (3%; ~4% of the trees missing), with trellis posts, a perimeter farm track, a hedge, a fence, a shed and a water tank. Highly repetitive along and across the rows (aliasing); the LiDAR sees trunks under the crowns. | 170 x 150 m | `rows` (ugv, 7:53, 696 m)<br>`headland` (ugv, 3:41, 525 m) |
| [parking_garage](worlds/parking_garage/README.md) | A three-level parking structure (64 x 40 m, levels at 0, 3.2 and 6.4 m, 2.9 m clearance) linked by two 20 m ramps (16% grade), with a column grid, parapets, an open roof level, ~60% occupied bays, and a few buildings and trees around it. Stresses multi-level geometry with ceilings, ramps (pitch), repetitive columns, and vertical drift between floors. | 120 x 90 m | `walk_down` (handheld, 4:01, 315 m)<br>`up_down` (car, 2:51, 565 m) |
| [parking_lot](worlds/parking_lot/README.md) | A 160 x 120 m open-air parking lot in front of a 120 m long supermarket: five double rows of perpendicular bays (~70% occupied by cars of four sizes), curbed end islands with trees, tall light poles, cart corrals, hedges and a fence. Repetitive rows of similar cars; large open aisles. | 160 x 127 m | `search` (car, 3:42, 934 m)<br>`ugv_loop` (ugv, 5:10, 569 m) |
| [rough_terrain](worlds/rough_terrain/README.md) | 260 x 260 m of steep, bumpy hills and valleys (+-10 m, slopes up to 26 deg, with 0.6 m bumps) with two craters, ~400 boulders of 0.3-1.6 m and a few dead trees. Stresses large, fast-changing pitch and roll, and a scene made almost only of terrain and rocks (no planes or poles). | 260 x 260 m | `hill_loop` (ugv, 6:58, 623 m)<br>`traverse_fast` (ugv, 1:57, 329 m) |
| [stair_course](worlds/stair_course/README.md) | An urban courtyard (60 x 50 m, facades all around) with two stair types for a legged robot: a straight, 2 m wide outdoor flight of 14 steps (17 cm risers, 30 cm treads) up to a 2.38 m terrace with parapets, and a spiral stair (20 steps of 18 cm, one full turn around a central column, floating treads and railing posts) from the terrace up to a 6 m platform on columns. Stresses straight and helical stair climbing and descending, with large, continuous yaw on the spiral. | 64 x 54 m | `dog_course` (dog, 2:40, 110 m) |
| [stairwell](worlds/stairwell/README.md) | A three-storey building (30 x 16 m, 3.5 m floor to floor) with a U-shaped stairwell (two flights of 10 steps, 17.5 cm risers, 28 cm treads, 32 deg, per storey, plus mid landings), a corridor and furnished rooms on each floor, slabs with ceilings, and windows. Stresses stairs (fast z changes with large pitch for a legged carrier), narrow spaces, ceilings and multi-floor ambiguity (identical floor layouts). | 50 x 32 m | `dog_floors` (dog, 3:49, 163 m)<br>`legged_floors` (legged, 4:48, 207 m)<br>`handheld_stairs` (handheld, 2:40, 118 m) |
| [tunnel](worlds/tunnel/README.md) | A 735 m road tunnel (straight, a 300 m radius 45 deg bend, straight), 10 m wide and 6.5 m high with a stepped arch, raised walkways, lamps every 10 m, signs every 100 m, emergency niches every 150 m and jet fan pairs every 200 m, with portals and a turnaround area outside. The classic degenerate case: almost nothing constrains motion along the axis. | 834 x 440 m | `car_through` (car, 2:28, 1782 m)<br>`ugv_inspection` (ugv, 5:28, 798 m) |
| [urban_street](worlds/urban_street/README.md) | A 3 x 3 grid of city blocks (~370 x 370 m) with 16 m wide streets, raised sidewalks with curbs, buildings 8-45 m tall with facade relief, a courtyard block, a park block, parked cars, street trees, lamp posts, benches, bins, bus stops, sign posts and traffic lights. Well-constrained, structured scene; the car route revisits streets (loops). | 370 x 370 m | `car_loops` (car, 5:48, 2935 m)<br>`pedestrian` (handheld, 8:23, 671 m) |
| [warehouse](worlds/warehouse/README.md) | A 90 x 60 m logistics hall, 10 m high with a ceiling and roof trusses, six double rows of 8 m pallet racks (5 levels, randomly filled) separated by 3.6 m aisles, loading docks with door openings, staging areas with pallets and an office cabin. Indoor and repetitive: the aisles look alike, with long, nearly identical views along them. | 110 x 80 m | `aisles` (ugv, 3:38, 410 m)<br>`walk` (handheld, 3:20, 260 m) |
<!-- END worlds -->

Screenshots are rendered by MVSim itself (`tools/screenshot.py`); indoor
worlds are shown as cutaways without their roofs or ceilings, which the
datasets do have. The plots show the top view of every primitive and the
trajectories (circle: start), and the height of `base_link` over time.

<!-- BEGIN gallery -->
### apartment_building

<img src="worlds/apartment_building/screenshot.png" width="49%"> <img src="worlds/apartment_building/trajectories.png" width="40%">

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

### stair_course

<img src="worlds/stair_course/screenshot.png" width="49%"> <img src="worlds/stair_course/trajectories.png" width="40%">

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

Total: 30 trajectories, 148 minutes, per sensor variant.
<!-- END sequences -->

## Sensors and variants

Each trajectory is rendered with these nine variants (files are named
`<rig>-<noise>-<shutter>`):

| variant | LiDAR | LiDAR noise | IMU noise | shutter |
|---|---|---|---|---|
| `os1_32-clean-global` | OS1-32 style | none | none | global |
| `os1_32-clean-rolling` | OS1-32 style | none | none | rolling |
| `os1_32-noisy-global` | OS1-32 style | 2 cm | MEMS (below) | global |
| `os1_32-noisy-rolling` | OS1-32 style | 2 cm | MEMS (below) | rolling |
| `os1_32-real-global` | OS1-32 style | realistic error model (below) | MEMS (below) | global |
| `os1_32-real-rolling` | OS1-32 style | realistic error model (below) | MEMS (below) | rolling |
| `vlp16-noisy-rolling` | VLP-16 style | 2 cm | MEMS (below) | rolling |
| `os1_64-noisy-rolling` | OS1-64 style | 2 cm | MEMS (below) | rolling |
| `os1_64-real-rolling` | OS1-64 style | realistic error model (below) | MEMS (below) | rolling |

The `real` variants are the closest to a real sensor and the intended
default for benchmarking; the others isolate one factor at a time. The
noise seed depends only on the sequence, so the variants of a sequence are
paired (e.g. `-global` and `-rolling` share their error parameters).
The `os1_64` variants differ from their `os1_32` counterparts only in the
ring count (same FOV, columns and range, and the same error model
parameters), to compare LiDAR densities.

Rigs (`rigs/*.rig.xml`, sensor parameters pinned in the files). The first three
are rendered in the published data; the others are defined for
`tools/generate.py` and used by derived works, but not rendered here:

| rig | rings | vertical FOV | columns | rate | range | LiDAR height on `base_link` |
|---|---|---|---|---|---|---|
| `os1_32` | 32, uniform | 45 deg (+-22.5) | 1024 | 10 Hz | 0.5 to 60 m | 0.75 m |
| `vlp16` | 16, uniform | 30 deg (+-15) | 1808 | 10 Hz | 0.4 to 100 m | 0.75 m |
| `os1_64` | 64, uniform | 45 deg (+-22.5) | 1024 | 10 Hz | 0.5 to 60 m | 0.75 m |
| `os1_16` | 16, uniform | 33.2 deg | 1024 | 10 Hz | 0.5 to 120 m | 0.75 m |
| `os1_64_car` | 64, uniform | 42.4 deg | 1024 | 10 Hz | 0.5 to 120 m | 1.80 m |
| `os0_128` | 128, uniform | 90 deg | 1024 | 10 Hz | 0.3 to 50 m | 0.75 m |
| `hdl64` | 64 | -24.8 to +2.0 deg | 2048 | 10 Hz | 0.9 to 120 m | 1.73 m |
| `xt32` | 32 | -16 to +15 deg | 2000 | 10 Hz | 0.5 to 120 m | 0.75 m |
| `qt64` | 64 | 104.2 deg | 600 | 10 Hz | 0.1 to 60 m | 0.75 m |

- **Range noise**: Gaussian, 0.02 m std in the `noisy` variants, none in the
  `clean` ones.
- **Realistic error model** (`real` variants, applied by `mvsim-dataset-gen`
  per ray): (1) no return beyond a maximum incidence angle; (2) Gaussian range
  noise with std `sqrt(s0^2 + (k r)^2)`, growing with the range `r`; (3) the
  beam footprint (`r` x divergence) on an oblique surface spreads the return
  uniformly over `r * divergence * tan(incidence)`; (4) at depth edges (a
  neighbor ray hits off the local tangent plane by more than
  max(5 cm, 2% of `r`)), a mixed return at a range drawn between both
  surfaces, with some probability; (5) per-ring calibration errors drawn once
  per sequence: elevation and azimuth offsets (the ray samples the scene
  along the perturbed direction, but the point is reported along the nominal
  one) and a range bias. The parameters are drawn uniformly per sequence and
  rig from these ranges (`REAL_ERROR_RANGES` in `tools/generate.py`), and
  stored next to each rawlog as `<variant>.errors.json`:

  | parameter | range |
  |---|---|
  | range noise std, constant `s0` | 0.005 to 0.02 m |
  | range noise std, per meter `k` | 0 to 3e-4 |
  | full beam divergence | 0.05 to 0.3 deg |
  | maximum incidence angle | 78 to 88 deg |
  | mixed-return probability at depth edges | 0.1 to 0.5 |
  | per-ring elevation offset std | 0 to 0.04 deg |
  | per-ring azimuth offset std | 0 to 0.04 deg |
  | per-ring range bias std | 0 to 0.01 m |

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
  every carrier, including the handheld, legged and dog ones, where it is not
  physically meaningful: do not use it for those sequences. It is the
  horizontal projection of the path, so on slopes and stairs it also disagrees
  with any body-frame motion estimate. MOLA's state-estimation smoother fuses
  every odometry label by default: for these carriers, set
  `do_process_odometry_labels_re: "^$"` in its YAML.
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
| `dog` | 0.00 m | 0.75 m | quadruped robot: up to 1.5 m/s (body center; the ground-level `base_link` sways slightly faster) on the flat and 0.3 to 0.6 m/s on stairs; 2 Hz trot with a 0.8 cm body bounce at 4 Hz, roll (2 deg), pitch (1.5 deg) and yaw (1 deg) oscillations; on stairs an extra bounce (1.5 cm) and pitch (2 deg) per tread, and the body pitched at 85% of the stair slope; gait rotations pivot about the body center, 0.45 m above `base_link` |

Stair types: straight flights (`stair_course`), a spiral stair
(`stair_course`), U-shaped stairs with a well (`stairwell`) and dog-leg stairs
with a central wall (`apartment_building`). Steps are boxes; the spiral's
treads are rectangular boxes around a central column.

### Conventions

- World frame: Z up, gravity 9.81 m/s^2 along -Z. Units: meters, seconds,
  radians.
- Timestamps: seconds, starting at `t = 100 s` for every sequence (never 0,
  which some tools read as "unset"). LiDAR observations are stamped at the
  sweep start; per-point `t` is relative to that stamp.
- TUM files: `timestamp x y z qx qy qz qw` (the pose of the frame in the
  world).
- Trajectories are defined in `worlds/<world>/trajectories/<traj>.tum` (20 Hz
  knots, 50 Hz for the `dog` sequences so that the gait survives the spline;
  `base_link`); the generator interpolates them with a C2 spline
  (`tools/mvsim_lio/densify.py`).

### Semantic labels

Every primitive of the worlds has a semantic class, and every discrete object
(a car, a tree with its trunk and foliage, a desk...) an instance id
(`tools/mvsim_lio/labels.py`; the world builder's `semantic()` and
`instance()` contexts). Rendered with `tools/generate.py --labels` (MVSim with
MRPT/mvsim#182), each point gets two `uint16` fields: `label`, the class of
the surface its ray hit, and `instance` (0 for extended surfaces such as
roads, terrain, floors and walls). Mixed returns, a range between two
surfaces, are labeled `outlier`. Labels never change the points: a labeled
render is identical to the unlabeled one with the same MVSim build.
The `os1_32-clean-rolling`, `os1_32-real-*` and `os1_64-*` variants carry the
labels. The others (`os1_32-clean-global`, `os1_32-noisy-*`,
`vlp16-noisy-rolling`) do not.
Outdoor classes reuse the SemanticKITTI raw label ids; indoor ones start at
100.

| id | class |
|---|---|
| 0 | unlabeled |
| 1 | outlier |
| 10 | car |
| 20 | other-vehicle |
| 40 | road |
| 44 | parking |
| 48 | sidewalk |
| 49 | other-ground |
| 50 | building |
| 51 | fence |
| 52 | other-structure |
| 70 | vegetation |
| 71 | trunk |
| 72 | terrain |
| 80 | pole |
| 81 | traffic-sign |
| 99 | other-object |
| 100 | floor |
| 101 | ceiling |
| 102 | wall |
| 103 | column |
| 104 | stairs |
| 105 | table |
| 106 | chair |
| 107 | shelf |
| 108 | other-furniture |
| 109 | goods |

## Files

Per sequence, under `<world>/<trajectory>/`:

| file | contents |
|---|---|
| `<variant>.rawlog` | MRPT rawlog (gzip): `CObservationPointCloud` (`lidar1`, a `CGenericPointsMap` with fields `x y z t ring`, plus `label instance` if rendered with labels; `sensorPose` = `lidar1` in `base_link`), `CObservationIMU` (`imu1`; `sensorPose` = `imu1` in `base_link`), `CObservationOdometry` (`odom`), in time order |
| `<variant>.gt.tum` | `base_link` ground truth at the sweep start times (10 Hz) |
| `<variant>.lidar1.gt.tum` | `lidar1` ground truth at the sweep start times |
| `<variant>.gen.log` | generator command line and log |
| `<variant>.errors.json` | `real` variants only: the drawn parameters of the realistic LiDAR error model |
| `gt_200hz.tum` | `base_link` ground truth at 200 Hz (same for every variant) |

Reference maps: `tools/map_export` builds the exact point-cloud map of what
the sensor saw (a `clean` variant placed with `gt_200hz.tum`), or the map
implied by an estimated trajectory, for map-quality metrics (see its
[README](tools/map_export/README.md)).

Note that some odometry systems stamp a scan at its mid-sweep time (MOLA does
for rolling-shutter data, +0.05 s): compare against the ground truth at the
estimate's own timestamps (e.g. interpolating `gt_200hz.tum`), not by
nearest-stamp association with the 10 Hz files.

## Using the data from ROS 2

`rawlog2rosbag` (in [mrpt_ros_bridge](https://github.com/MRPT/mrpt_ros_bridge))
converts a sequence into a ROS 2 bag. It is not in the ROS binary releases
yet, so build it from source in a colcon workspace:

```bash
cd ~/ros2_ws/src && git clone -b ros2 https://github.com/MRPT/mrpt_ros_bridge.git
cd ~/ros2_ws && colcon build --packages-up-to rawlog2rosbag && source install/setup.bash
```

Then, for one sequence (the ground truth is optional):

```bash
S=office/spins
rawlog2rosbag -i $S/os1_32-noisy-rolling.rawlog -o office_spins_os1_32-noisy-rolling \
  --gt-tum $S/gt_200hz.tum
```

| topic | type | contents |
|---|---|---|
| `/lidar1/points` | `sensor_msgs/PointCloud2` | one sweep per message (10 Hz), in the `lidar1` frame: `x y z t` (float32), `ring` (uint16), plus `label instance` (uint16) in sequences rendered with labels. `t` is in seconds from the header stamp, the sweep start |
| `/imu1/imu` | `sensor_msgs/Imu` | 200 Hz angular velocity and specific force, in the `imu1` frame (no orientation) |
| `/odom/odom` | `nav_msgs/Odometry` | wheel odometry, `odom` to `base_link` (10 Hz) |
| `/tf` | `tf2_msgs/TFMessage` | `odom` to `base_link`, from the wheel odometry |
| `/tf_static` | `tf2_msgs/TFMessage` | `base_link` to `lidar1` and `imu1` (see [Frames and extrinsics](#frames-and-extrinsics)) |
| `/ground_truth` | `nav_msgs/Odometry` | with `--gt-tum`: `base_link` in the `map` frame |

- Stamps start at 100 s (see [Conventions](#conventions)): play the bag with
  `ros2 bag play --clock` and run the nodes with `use_sim_time:=true`.
- To leave the wheel odometry and its `/tf` out of the bag (e.g. for a LiDAR-inertial method
  that publishes `odom` itself), pass `-c no_odom.yaml` with:
  ```yaml
  sensors:
    odom:
      skip: true
  ```
- Size and speed, for `office/spins` (a 520 MiB rawlog, 159 s): 30 s and 818 MiB
  with the default MCAP `zstd_fast` preset; `--storage-preset zstd_small` writes
  518 MiB but takes 13 min; `--storage-id sqlite3` 1.1 GiB.
- Add `--no-latched-qos` for bags to be read by ROS 2 Humble.

## How to regenerate

Requirements: Python 3 with numpy, scipy, matplotlib and Pillow, and a build
of MVSim's `develop` branch that
includes MRPT/mvsim #168, #170 and #171 (see [AGENTS.md](AGENTS.md)): exact
IMU timestamps (without them the simulated accelerometer is wrong by several
m/s^2), IMU kinematics at the IMU mounting pose (lever arm), and a heightfield
speed-up (without it terrain worlds render about 100x slower).
`tools/check_imu.py <world> <traj>` checks the IMU of a generated clean
sequence against the trajectory.

```bash
# MVSim built with colcon, e.g. in ~/ros2_ws: colcon build --packages-select mvsim
export MVSIM_LIO_MVSIM_INSTALL=/path/to/mvsim/install   # if not in ~/ros2_ws/install
source tools/env.sh
tools/build_worlds.py              # worlds/*/world.xml, trajectories, plots
tools/generate.py --jobs 5 --cores-per-job 8   # all sequences and the nine variants
tools/screenshot.py                # worlds/*/screenshot*.png
tools/make_banner.py --maps <dir>  # docs/banner.{gif,mp4} (maps from tools/map_export)
tools/validate_mola.py --variant os1_32-noisy-rolling   # optional sanity check
tools/make_docs.py --manifest      # README tables, MANIFEST.tsv, SHA256SUMS
```

`tools/generate.py --worlds forest --trajs trail_loop --variants os1_32-noisy-rolling`
renders a single sequence. Outputs are byte-for-byte reproducible with the same
MVSim build.

### Data provenance

These MVSim builds produced the published files:

- The 2026-11-04 release (the first 13 worlds) was rendered with a local
  build of the three fixes before their upstream review.
- The robot dog sequences (`apartment_building/*`, `stair_course/*`,
  `stairwell/dog_floors`) were rendered on 2026-11-06 with upstream MVSim
  `develop` at commit 7b203a52, which merged those fixes.
- Nine files of the first release were also re-rendered with 7b203a52:
  `stairwell/handheld_stairs` (all five variants) and
  `stairwell/legged_floors` (the four `os1_32` variants).
  `stairwell/legged_floors/vlp16-noisy-rolling` is still the 2026-11-04 file.
- The `os1_32-real-global` and `os1_32-real-rolling` variants of all 30
  sequences were rendered on 2026-11-11 with upstream MVSim `develop` at
  commit 5c7e3a86, which includes the realistic error model.
- The `os1_64-*` variants were rendered on 2026-11-12 with MVSim 2.1.0
  (`develop` at 1ffefda4) plus MRPT/mvsim#182 (point labels) and #183 (IMU
  sensor pose).
- The labels were added on 2026-11-12 and 2026-11-13 by rendering again with
  that same build. Each `os1_32-real-*` file was replaced only if its points
  and ground truth were identical to the released file (all 60 were). All 30
  `os1_32-clean-rolling` files were replaced: 6 are identical, and the 24 of
  the first release differ at the float-rounding level described below.
  Through rays that graze an edge, that can move a single point by up to the
  range jump, and change the point count of a few sweeps.

Until MRPT/mvsim#183, the generator left the `sensorPose` of the IMU
observations at the identity. The samples themselves were already simulated
at the mounting pose. On 2026-11-12 the `imu1` observations of every
`os1_32-*` and `vlp16-*` file got their mounting pose, in place: only that
field changed, and a patched file is byte-identical to a render with the
fixed generator ([`tools/rawlog_tools`](tools/rawlog_tools/README.md)).

The review of the timestamp fix made the trajectory interpolation use times
relative to the first knot. Upstream renders of the first-release sequences
therefore differ from the 2026-11-04 files only at the float-rounding level:
in the LiDAR points, and in a few ground-truth lines in the last printed
digit (1e-6) of a quaternion component. Regenerating the
first-release files with upstream MVSim gives files that are equivalent but
not byte-identical. The generator logs (`*.gen.log`) of the first-release
files that were not re-rendered were lost, and are not in the manifest.


<!-- BEGIN gentime -->
Generation speed (simulated seconds per wall-clock second; cores per job in each column, Xeon E5-2696 v4 at 2.2 GHz, on a shared machine), per world and variant:

| world | os1_32-clean-global (8 cores) | os1_32-clean-rolling (4 cores) | os1_32-noisy-global (8 cores) | os1_32-noisy-rolling (8 cores) | os1_32-real-global (4 cores) | os1_32-real-rolling (4 cores) | vlp16-noisy-rolling (8 cores) |
|---|---|---|---|---|---|---|---|
| apartment_building | 3.6x | 2.1x | 3.1x | 3.0x | 1.7x | 1.5x | 3.2x |
| campus | 4.7x | 3.4x | 4.7x | 4.5x | 3.2x | 2.6x | 4.4x |
| forest | 2.8x | 2.2x | 2.8x | 2.8x | 2.0x | 1.8x | 2.8x |
| greenhouse | 2.9x | 1.9x | 2.5x | 2.4x | 1.5x | 1.4x | 2.7x |
| office | 3.6x | 2.7x | 2.5x | 2.6x | 2.0x | 1.8x | 2.8x |
| open_field | 6.2x | 4.0x | 5.8x | 5.8x | 4.6x | 4.2x | 5.8x |
| orchard | 3.7x | 2.5x | 3.6x | 3.6x | 2.2x | 2.3x | 3.7x |
| parking_garage | 3.5x | 2.2x | 3.0x | 3.0x | 1.8x | 2.1x | 3.2x |
| parking_lot | 6.7x | 4.6x | 3.9x | 3.9x | 2.8x | 3.6x | 4.4x |
| rough_terrain | 4.9x | 3.5x | 4.8x | 4.9x | 3.1x | 3.4x | 4.7x |
| stair_course | 3.4x | 2.3x | 3.2x | 3.2x | 2.0x | 2.1x | 3.4x |
| stairwell | 3.1x | 2.1x | 2.9x | 2.9x | 1.8x | 1.8x | 3.1x |
| tunnel | 5.1x | 3.5x | 3.1x | 3.1x | 2.0x | 2.2x | 3.3x |
| urban_street | 3.8x | 2.6x | 3.1x | 3.1x | 2.0x | 2.1x | 3.2x |
| warehouse | 2.6x | 1.4x | 2.3x | 2.3x | 1.1x | 1.1x | 2.5x |
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
| `apartment_building/dog_floors` | 0.11 m / 0.00 % | 0.14 m / 0.00 % |
| `apartment_building/dog_stairs` | 0.11 m / 0.00 % | 0.13 m / 0.00 % |
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
| `stair_course/dog_course` | 0.04 m / 0.00 % | 0.07 m / 0.01 % |
| `stairwell/dog_floors` | 0.14 m / 0.00 % | 0.16 m / 0.01 % |
| `stairwell/handheld_stairs` | 0.01 m / 0.00 % | 0.06 m / 0.01 % |
| `stairwell/legged_floors` | 0.13 m / 0.00 % | 0.13 m / 0.00 % |
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
- The robot dog sequences (stairs of all four types, bare apartment rooms)
  stay at 4 to 16 cm APE and below 0.02% drift, also with noise: trot and
  stair motion do not break this odometry, which de-skews with a
  constant-velocity model. Their IMU carries the gait (about 2.6 m/s^2 std
  vertically on flat ground), which is what LiDAR-inertial methods have to
  cope with.

First difficulty ranking (noisy rolling shutter, end-point drift):

| class | sequences |
|---|---|
| easy (< 0.1%) | `apartment_building/*`, `stair_course/*`, `office/*`, `greenhouse/*`, `stairwell/*`, `warehouse/aisles`, `parking_garage/up_down`, `parking_lot/*`, `orchard/rows` |
| moderate (0.1% to 1.2%) | `urban_street/*`, `parking_garage/walk_down`, `warehouse/walk`, `tunnel/car_through`, `rough_terrain/*`, `forest/crossing`, `orchard/headland` |
| hard (> 1.5%) | `campus/*`, `forest/trail_loop`, `tunnel/ugv_inspection`, `open_field/*` |

## Dataset manifest

Generated data size per world (all variants). The full list of files with
their sizes and SHA-256 checksums is in [MANIFEST.tsv](MANIFEST.tsv) and
[SHA256SUMS](SHA256SUMS) (`sha256sum -c SHA256SUMS` from the data directory).

<!-- BEGIN manifest -->
| world | rawlogs | files | size |
|---|---|---|---|
| apartment_building | 18 | 98 | 24.4 GB |
| campus | 18 | 90 | 29.1 GB |
| forest | 18 | 90 | 17.2 GB |
| greenhouse | 18 | 90 | 23.7 GB |
| office | 18 | 90 | 14.0 GB |
| open_field | 18 | 90 | 12.0 GB |
| orchard | 18 | 90 | 21.2 GB |
| parking_garage | 18 | 90 | 12.8 GB |
| parking_lot | 18 | 90 | 11.1 GB |
| rough_terrain | 18 | 90 | 11.0 GB |
| stair_course | 9 | 49 | 5.9 GB |
| stairwell | 27 | 146 | 25.5 GB |
| tunnel | 18 | 90 | 15.2 GB |
| urban_street | 18 | 90 | 25.7 GB |
| warehouse | 18 | 90 | 15.6 GB |
| **total** | 270 | 1373 | **264.5 GB** |
<!-- END manifest -->

## Download

TODO: the datasets will be published later; the location is not decided yet.

## Limitations

- Geometry is restricted to what `mvsim-dataset-gen` ray-traces exactly:
  vertical prisms (boxes, walls), cylinders, spheres, wedges (ramps, roofs),
  horizontal planes and heightfields. 3D mesh models (`<visual>` models) are
  not ray-traced, so trees are made of trunks, branches and clusters of small
  spheres, and cars of boxes and wheels.
- No glass: windows and doors are open gaps. No intensity, multi-echo or
  reflections. Beam divergence, grazing-angle dropouts, mixed returns and
  per-beam calibration errors exist only in the `real` variants, and are
  geometric models (there is no material model).
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
