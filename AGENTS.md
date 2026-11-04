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

The data was generated with a local MVSim worktree:
`~/mvsim_lio_ws/src/mvsim`, branch `local/dataset-gen-time-rounding` (on top
of `feat/dataset-gen-rolling-lidar-default`, commit 947846ed), built with
`~/mvsim_lio_ws/build.sh` into `~/mvsim_lio_ws/install` (what `tools/env.sh`
sources by default). Two commits, not pushed upstream:

- `mvsim-dataset-gen: round timestamps to the nearest clock tick`:
  `mrpt::Clock::fromDouble()` truncates, so times land 100 ns early at random;
  the IMU finite-difference step is 0.5 ms, which turned that into errors of
  the order of the speed in m/s^2 (a 2 m/s straight line read -9.5 +- 7.8
  m/s^2). With the fix a constant-velocity line reads exactly 0.
- `raytracer: cache heightfield z bounds ...`: the elevation-map ray cast
  scanned the whole matrix per ray (about 100x slower than real time on a
  400 x 400 grid). Output unchanged (checked bit for bit).

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

## Machine notes

Shared server: use at most ~40 cores (`--jobs 5 --cores-per-job 8`), check
`uptime` first. Each mvsim-dataset-gen job runs at about 4 to 10x real time
with 8 threads; a full regeneration of all 130 sequences takes about an hour.
