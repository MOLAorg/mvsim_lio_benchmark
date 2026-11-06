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
2. `tools/build_worlds.py` writes `world.xml`, `trajectories/<t>.tum` (20 Hz
   knots, the canonical trajectory), `stats.json` and `trajectories.png`, and
   runs a clearance check (prints COLLISION). Output is deterministic: check
   with md5sum after edits.
3. `tools/generate.py` densifies each trajectory to 2 kHz with a C2 spline
   (`densify.py`), then runs `mvsim-dataset-gen` per variant, choosing the
   sensor rig with `MVSIM_LIO_RIG=rigs/<rig>.rig.xml` (the world XML includes
   `$env{MVSIM_LIO_RIG}`). Seeds depend only on (world, trajectory).
4. `tools/screenshot.py`, `tools/validate_mola.py`, `tools/make_docs.py`
   (README tables between `<!-- BEGIN/END x -->` markers, per-world READMEs,
   `MANIFEST.tsv`, `SHA256SUMS`). Do not edit generated tables by hand.

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

Data provenance (details in the README): the first release (2026-11-04,
13 worlds) was rendered with a pre-review worktree of the three fixes below.
The dog sequences and nine stairwell files (`handheld_stairs`, all variants;
`legged_floors`, the four `os1_32` variants) were rendered on 2026-11-06 with
develop at commit 7b203a52. The review of #170 changed the trajectory
interpolation to times relative to the first knot, so upstream renders of
first-release sequences differ at the float-rounding level (LiDAR points,
and the last printed digit of a few GT quaternions). Do not regenerate first-release sequences without the
owner's approval. Their generator logs were lost on 2026-11-06 (deleted to
queue a regeneration that was then cancelled), and are not in the manifest.

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
smoother (fixed in mola_state_estimation#102). validate_mola.py uses the
simple estimator and is unaffected.

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
