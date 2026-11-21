# AGENTS.md: context for automated agents

## What this repo is

World definitions, trajectories and scripts for a simulated LiDAR-inertial
benchmark rendered with MVSim's offline generator `mvsim-dataset-gen`. The
generated data is NOT in git; on the machine where it was made it lives in
`/mnt/datasets/mvsim_lio_benchmark/`.

## Pipeline

1. `worlds/<name>/world.py` (the source of truth) defines the geometry with
   `tools/mvsim_lio/world.py` (+ `props.py`) and the trajectories with
   `tools/mvsim_lio/traj.py` (waypoints `P`, `Stop`, `Spin`, per carrier).
   Every primitive gets a semantic class (`labels.py`) from a
   `with w.semantic(...)` context (props tag themselves, one instance per
   call); `build_worlds.py` prints UNLABELED if any primitive has none.
2. `tools/build_worlds.py` writes `world.xml`, `trajectories/<t>.tum` (20 Hz
   knots, the canonical trajectory), `stats.json` and `trajectories.png`, and
   runs a clearance check (prints COLLISION). Output is deterministic: check
   with md5sum after edits.
3. `tools/generate.py` densifies each trajectory to 2 kHz with a C2 spline
   (`densify.py`), then runs `mvsim-dataset-gen` per variant, choosing the
   sensor rig with `MVSIM_LIO_RIG=rigs/<rig>.rig.xml` (the world XML includes
   `$env{MVSIM_LIO_RIG}`). Seeds depend only on (world, trajectory).
   Noise levels: `clean`, `noisy` (2 cm Gaussian), `real` (the realistic
   LiDAR error model with the beam model and a detection range, parameters
   drawn per (world, trajectory, rig) from `REAL_ERROR_RANGES`, which bracket
   two real Ouster sensors, and written to `<variant>.errors.json`). The
   published set is `VARIANTS` (nine variants); other rigs are defined but not
   rendered.
   `--labels` adds the per-point `label`/`instance` fields (MRPT/mvsim#182);
   it does not change the points.
4. `tools/screenshot.py`, `tools/validate_mola.py`, `tools/make_docs.py`
   (README tables between `<!-- BEGIN/END x -->` markers, per-world READMEs,
   `MANIFEST.tsv`, `SHA256SUMS`). Do not edit generated tables by hand.
5. `tools/map_export` (C++, `bench-map-export`): exact reference maps (clean
   sweeps at `gt_200hz.tum`) or maps implied by an estimated trajectory.
   `tools/make_banner.py` renders `docs/banner.{gif,mp4}` from such maps.
6. `tools/rawlog_tools` (C++): `compare-rawlogs` (identical points of two
   renders, label histogram) and `set-imu-pose` (in-place IMU pose fix).
7. Hosting: the data is on the Hugging Face Hub, dataset `MOLAorg/MVSim-LIO`.
   Its card is `hf/README.md` (keep it in sync with this README; the GitHub
   repository is not linked from it while private). `tools/hf_stage.py`
   builds a symlink tree of the `MANIFEST.tsv` files of some variants, the
   card and its banner, to upload with `hf upload ... --repo-type dataset`.

Changing a world or trajectory invalidates its data: regenerate with
`tools/generate.py --worlds <w> --trajs <t> --force` (a sequence whose rawlog
and `.gt.tum` exist is skipped unless its log shows an incomplete run), then
rerun the manifest. Always restrict `--force` with `--worlds`/`--trajs`.

## MVSim requirements

Build MVSim's `develop` (it has the three fixes below) in `~/ros2_ws`:
clone MRPT/mvsim into `~/ros2_ws/src/mvsim` and run
`colcon build --packages-select mvsim --cmake-args -DCMAKE_BUILD_TYPE=Release`
from `~/ros2_ws`. Never build the whole workspace there (other packages carry
local experimental patches). `tools/env.sh` sources `~/ros2_ws/install`, or
`$MVSIM_LIO_MVSIM_INSTALL` if set.

Data provenance (details in the README): the `real` variants (release v2,
2026-11-21, approved by the owner) are renders of MVSim develop at 8a45b9e1
(merge of MRPT/mvsim#185, the beam model and detection range; same tree as its
head 513fbd7f), with the v2 `REAL_ERROR_RANGES`; the Hub keeps the first
release under the tag `v1.0`. Every other published file is a render of MVSim
2.1.0 (develop at 1ffefda4) plus #182 and #183, with labels; develop at
8a45b9e1 reproduces them byte for byte, as does `tools/generate.py` with the
first build. Earlier
builds: the first release (2026-11-04, 13 worlds), a pre-review worktree of
the three fixes below; the dog and nine stairwell files (2026-11-06), develop
at 7b203a52; `os1_32-real-*` (2026-11-11), develop at 5c7e3a86. The label
renders (2026-11-12/13/15) replaced `os1_32-real-*` only where identical
(all 60) and the other `os1_32` and `vlp16` variants in full. Files of the
first release changed at the float-rounding level (the review of #170 made
the trajectory interpolation use times relative to the first knot) and, in
the noisy variants, in the noise draws after a changed point count. The
in-place `imu1` `sensorPose` patch of 2026-11-12 is superseded. Do not
re-render published files without the owner's approval.

- MRPT/mvsim#170, exact timestamps for the IMU finite differences.
  `mrpt::Clock::fromDouble()` truncated times. `CPose3DInterpolator` resolves
  only about 2 us with absolute times, and the stencil times were inexact
  doubles. Through the 0.5 ms step this gave accelerometer errors of several
  m/s^2: a 2 m/s straight line read -9.5 +- 7.8 m/s^2. The PR stores knots
  relative to the first one and uses exact clock ticks.
- MRPT/mvsim#171 (stacked on #170), IMU kinematics at the IMU mounting pose.
  The IMU was simulated at the vehicle origin and in the vehicle axes,
  ignoring its `<pose_3d>`, so there were no lever-arm terms.
  `tools/check_imu.py` verifies a clean sequence against the trajectory
  spline at the IMU pose: errors are about 1e-5 m/s^2 on smooth motion, and
  a few mm/s^2 with gait bobbing.
- MRPT/mvsim#168, heightfield z bounds cached. The elevation-map ray cast
  scanned the whole matrix per ray, about 100x slower than real time on a
  400 x 400 grid. Output is unchanged (checked bit for bit).

## Road worlds and moving traffic

`open_highway` and `open_road` have `HOSTED = False` in their `world.py`:
`generate.py` renders them only when named with `--worlds`, and
`make_docs.py` lists them in the README's `extra_worlds` table and keeps them
out of the tables and totals of the hosted data. Their vehicles
(`tools/mvsim_lio/traffic.py`) are named blocks parked 40 m below the
terrain; a world with a `moving_blocks(traj, t, x)` function gets one
trajectory per block from `generate.py` (`--moving-block`, MRPT/mvsim#184,
in `develop` since 2026-11-14), derived from the car's own along-road
profile, so traffic stops and starts with it. `MVSIM_LIO_DATASET_GEN`
selects a generator other than the one in PATH. `open_highway` registers its
own carrier (`car_highway`, 30 m/s limit) in its `world.py`. The car rigs
`hdl64` and `os1_64_car` have the LiDAR at its height above the road on the
`car` carrier (`base_link` is 1.15 m above the ground): no hosted variant
uses them.
A `Terrain` with a `paint` function gets a ground texture (`terrain.png`,
written by `build_worlds.py`; `paint_road()` draws asphalt and lane lines):
MVSim shows it in screenshots, the ray tracer ignores it (rawlogs are
byte-identical with and without it).

## Carriers and stairs

`traj.PLATFORMS` defines the carriers (`car`, `ugv`, `legged`, `handheld`,
`dog`). The `dog` (quadruped robot) uses a time-based trot (`traj.DOG_TROT`:
2 Hz cycle, 4 Hz body bounce, roll/pitch/yaw oscillations, plus a bounce and
pitch per tread on stairs) instead of the distance-based gait of the others.
Its gait rotations pivot about a point 0.45 m above `base_link` (the body
center, where the IMU is): rotating about `base_link` on the ground gave the
IMU ~7 m/s^2 of spurious lever-arm acceleration. Its trajectories are
committed with `knot_hz=50`: with 20 Hz knots the cubic
spline would distort the 4 Hz bounce. Set `pitch_scale` (0.85 for the dog,
0.2 for an upright handheld rig) to choose how much the body pitches with
the stairs. Stairs are `props.stairs_flight()` boxes (with a `thickness`, so
the flights above leave headroom); the spiral in `stair_course` is built
directly from yawed boxes around a column. Paths on stairs use explicit z
(`explicit_z=True`) with waypoints at the first and last tread of each
flight; put the stair speed limit (`P(..., v=...)`) on a point that is kept
when lists are concatenated with `[1:]`.

The planar `odom` stream (CObservationOdometry) is derived from the ground
truth for every carrier; it is meaningless for legged, dog and handheld
sequences, and its label collides with MOLA's own `odom` frame in the
smoother (fixed in mola_state_estimation#102). Both MOLA estimators fuse it by
default (`do_process_odometry_labels_re` is `.*`), including the simple one that
validate_mola.py runs, so its table is not LiDAR-only (tunnels depend on it).
Exclude it (`"^$"`) on non-wheeled carriers and for LiDAR(-inertial)
comparisons: fused, it bends the estimate (forest trail_loop global shutter
388 vs 102 mm with the smoother).

## Gotchas found

- `mvsim-dataset-gen` ray-traces only analytic geometry. `<visual>` meshes are
  ignored (a block without `<geometry>` becomes its bounding prism), and
  `intangible` blocks are skipped entirely (the docs say 3D LiDARs see them;
  this generator does not).
- Boxes are ray-traced as vertical prisms: pitch/roll of a box are ignored by
  the ray tracer but not by the visual model. Never tilt boxes; use ramps
  (wedges, full 6-DoF pose) for slopes. A ramp with roll = pi is an inverted
  wedge: its sloped face looks down (greenhouse roofs seen from inside).
- A box's pose is its corner, and yaw rotates about that corner:
  `World.box()` takes the center and converts.
- MVSim replaces a block's z with the terrain height under it unless the
  block has `<skip_elevation_adjust/>` (put in every block class here, with
  `<static>true</static>`).
- Elevation maps: matrix rows run along X. Use `<elevation_data_matrix>`
  (exact); elevation images are 8-bit (terracing).
- `mvsim-dataset-gen` interpolates trajectories linearly and differentiates
  them with a step of IMU period / 10: knots must be spaced by a divisor of
  that step (hence 2 kHz dense files), or accelerations are wrong.
- Write trajectories with full precision: positions rounded to 1e-7 m in the
  dense file gave +-0.4 m/s^2 of accelerometer error (second differences over
  0.5 ms). densify.py writes 12 decimals.
- Path reversals inside one run of `P` points make a cusp with a huge yaw
  rate: put a `Stop()` there (the builder then turns in place). Check
  `yawrate_max_deg` in `stats.json` after editing a trajectory.
- The clearance check skips footprints marked `drivable` (ramps, steps,
  slabs) and knows wall openings, but does not check vertical clearance.
- MOLA stamps rolling-shutter scans at mid-sweep (+0.05 s): score against the
  ground truth interpolated at the estimate timestamps (validate_mola.py does
  it from the trajectory spline), never by nearest 10 Hz stamp.
- Screenshots: `mvsim launch --headless` with a one-frame `<video_recording>`
  (PNG pattern) passed as the rig include. Indoor views use `cut_z` cutaways
  (`World.to_xml(cut_z=...)`), which only affect the screenshot.
- Rendering a world in `mvsim launch` steps Box2D: thousands of blocks are
  fine because they are static.

## Validation

`tools/validate_mola.py --variant <v>` runs `mola-lo-cli-rawlog` (labels
`lidar1`/`imu1`; the default pipeline de-skews with a constant-velocity model
and uses the accelerometer for gravity) and writes
`/mnt/datasets/mvsim_lio_benchmark/validation/summary.json`, which
`make_docs.py` turns into the README table. Clean global-shutter runs of the
structured worlds give mm to cm errors; large errors in open_field, tunnel
(UGV) and campus persist with clean data (scene degeneracy, not data bugs).

## Machine notes

Shared server, also used for pinned benchmark sweeps: use `nice -n 10` and at
most 16 cores (`--jobs 2 --cores-per-job 8 --first-core 72`), check `uptime`
first, and keep scratch files in a private subdirectory. Each mvsim-dataset-gen
job runs at about 2.3 to 6.7x real time with 8 threads (indoor worlds, where
every ray hits, are the slowest; the dog worlds ran at about 3x under load).
The first release (130 sequences) took about 40 minutes on 40 cores; the 20
dog sequences about 30 minutes on 16 cores. `make_docs.py --manifest` hashes
all 109 GB in a few minutes.
