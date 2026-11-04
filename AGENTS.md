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
`tools/generate.py --worlds <w> --force` (existing outputs are skipped
otherwise), then rerun the manifest.

## MVSim requirements (patched build)

The published data was generated with a local MVSim worktree,
`~/mvsim_lio_ws/src/mvsim`, branch `local/dataset-gen-time-rounding`. That
branch is on top of `feat/dataset-gen-rolling-lidar-default`, commit 947846ed.
It was built with `~/mvsim_lio_ws/build.sh` into `~/mvsim_lio_ws/install`,
which is what `tools/env.sh` sources by default. Its fixes are now upstream
PRs, and the worktree is on `local/mvsim-lio-benchmark`, which merges all
three:

- MRPT/mvsim#170, exact timestamps for the IMU finite differences.
  `mrpt::Clock::fromDouble()` truncated times. `CPose3DInterpolator` resolves
  only about 2 us with absolute times, and the stencil times were inexact
  doubles. Through the 0.5 ms step this gave accelerometer errors of several
  m/s^2: a 2 m/s straight line read -9.5 +- 7.8 m/s^2. The PR stores knots
  relative to the first one and uses exact clock ticks. The published data
  was not affected by the interpolation part: its 2 kHz knots put every
  stencil point exactly on a knot, so no interpolation happens there.
- MRPT/mvsim#171 (stacked on #170), IMU kinematics at the IMU mounting pose.
  The IMU was simulated at the vehicle origin and in the vehicle axes,
  ignoring its `<pose_3d>`, so there were no lever-arm terms.
  `tools/check_imu.py` verifies a clean sequence against the trajectory
  spline at the IMU pose: errors are about 1e-5 m/s^2 on smooth motion, and
  a few mm/s^2 with gait bobbing.
- MRPT/mvsim#168, heightfield z bounds cached. The elevation-map ray cast
  scanned the whole matrix per ray, about 100x slower than real time on a
  400 x 400 grid. Output is unchanged (checked bit for bit).

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

Shared server: use at most ~40 cores (`--jobs 5 --cores-per-job 8`), check
`uptime` first. Each mvsim-dataset-gen job runs at 2.3 to 6.7x real time with
8 threads (indoor worlds, where every ray hits, are the slowest); all 130
sequences took about 40 minutes and 93 GB. MOLA validation of two variants
(52 runs) took about 30 minutes. `make_docs.py --manifest` hashes everything
in about 3 minutes.
