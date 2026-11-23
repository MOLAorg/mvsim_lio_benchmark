# The realistic LiDAR error model against real Ouster data

Two real sensors (and three more, below), compared with the error model of the benchmark
(`mvsim-dataset-gen/LidarSimulator.cpp`, compiled into `lidar_tool` unchanged):

- **Newer College 2020, OS1-64 (first generation), with the TLS reference.**
  The Quad mesh of the prior map, trimmed to the faces within 2 cm of TLS points
  (`prep_mesh.py`). The first 15.6 s of `01_short` (156 sweeps, the device
  still) and 17 more near-still sweeps (`ncd_extract.py`; the 2020 clouds are
  stored column by column despite height = rings). Poses: ground truth (a 135
  deg yaw off the lidar frame) refined per sweep by point-to-plane ICP against
  the 1 cm TLS cloud (`ncd_align.py`; median correction 10 cm, a lever arm).
  Reference ranges: every pixel's ray cast into the mesh (`lidar_tool cast`).
  Simulated sweeps at the same poses with the error model at the low end,
  middle and high end of its parameter ranges, and noise-free
  (`ncd_sim.py`, `lidar_tool sim`). The noise-free render matches the cast
  to 0.19 mm (99.9th percentile).
- **OS0-32 on a wheeled robot (firmware 2.3.0, 1024 x 10, single return).** The
  30 intervals of 10 s or more with the robot still (wheel odometry) in the
  three recordings of 2026-11-16, 11-19 and 11-20: 4937 sweeps
  (`husky_extract.py`). No reference map: every statistic is temporal.

- **Three more Ouster units of public datasets**, in the intervals in which
  their IMU shows them still for 10 s or more (`bag_still.py`: 1 s windows with
  a gyroscope spread below 0.012 rad/s and an accelerometer spread below
  0.25 m/s2, in the first and last minutes of each bag), extracted with
  `bag_static_extract.py`: the OS1-64 of GEODE (`Offroad1_beta` 0-15, 17-36 and
  395-418 s, `Shield_tunnel7_beta` 493-517 s; 768 sweeps), and the OS1-64
  (`/os_cloud_nodee`, 64 x 2048) and OS0-128 (`/os_cloud_node`, 128 x 2048) of
  TIERS (`indoor01` 101-114.5 s, `forest02` 83-97 s; 255 and 256 sweeps). Also
  120 sweeps of the Newer College OS1-64 still at the start of `02_long`
  (0-13 s), pooled with the static run of `01_short`. The GEODE boat sequence
  was left out (still IMU, drifting hull).

Statistics (`temporal.py`, identical for real and simulated sweeps):
per-pixel robust temporal spread of the range (noise), incidence from local PCA
normals (TLS normals in `ncd_static.py`), share of sweeps with a return (a
column counts only when it holds at least 80% of its usual returns, which drops
an operator or a passer-by next to the sensor), and at depth edges along a
ring, the share of each pixel's returns between the two surfaces.

Order:

```bash
W=<work dir>
NCD=<Newer College 2020 dataset dir>
prep_mesh.py "$NCD/03_new_college_prior_map/new-college-29-01-2020-1cm-resolution-1stSection - mesh.ply" \
  $NCD/03_new_college_prior_map/new-college-29-01-2020-1cm-resolution-1stSection.ply $W/quad_mesh_clean.ply
ncd_extract.py $NCD/01_short_experiment -19 34 -71 16 $W/ncd_still.npz
ncd_align.py $W/ncd_still.npz $NCD/01_short_experiment/ground_truth/gt.tum \
  $NCD/03_new_college_prior_map/new-college-29-01-2020-1cm-resolution-1stSection.ply $W
lidar_tool cast $W/quad_mesh_clean.ply $W/rays.bin $W/hits.bin 8     # build: CMakeLists.txt, -DMVSIM_SRC=...
ncd_sim.py $W $NCD/04_calibration/beam_intrinsics.json lidar_tool
ncd_analyze.py $W lidar_tool       # also casts the simulated grid (hits_sim.bin)
ncd_static.py $W                   # -> static_summary.json
husky_extract.py <bag.mcap> $W/husky   # for each recording (ROS 2 env)
bag_still.py <bag> <imu_topic>     # still intervals, for each extra bag
bag_static_extract.py <bag> <points_topic> $W/<geode|tiers_os1_64|tiers_os0_128|ncd_long> \
  --name <seq> --intervals a:b,...
run_temporal.py $W                 # -> temporal_summary.json
```

`static_summary.json` and `temporal_summary.json` hold the statistics of the paper's
validation figure.
