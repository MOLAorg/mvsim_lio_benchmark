# Tuning factors

Every method is tuned with the same budget: up to four parameters, each at its default (first
value) or at one alternative, run on the training split (office, apartment building, parking
lot, orchard and tunnel) at the `noisy` level with both shutters. `configs/tuning.json` holds
the exact overrides, `configs/tuned.json` the chosen configuration, and
`results/tuning_scores.tsv` the scores.

| Method | Factors (default -> alternative) |
|---|---|
| MOLA-LO | ICP points 3000 -> 6000 and voxel 0.10 -> 0.07 m; ICP iterations 16 -> 32; map voxel 0.15 -> 0.10 m; keyframes searched 3 -> 6 |
| MOLA-LIO | LiDAR increment weighted by its ICP covariance; looser fixed increment sigmas; IMU noise; ICP points as MOLA-LO |
| KISS-ICP | voxel r_max/100 -> r_max/200; points per voxel 20 -> 40; initial threshold 2 -> 1 m; min. motion 0.1 -> 0.05 m |
| Traj-LO | segment 40 -> 20 ms; segments 3 -> 5; input voxel 0.4 -> 0.2 m; map voxel 0.4 -> 0.8 m |
| DLIO | voxel 0.25 -> 0.10 m; keyframe distance 1.0 -> 0.5 m; submap keyframes 10 -> 20; GICP max. distance 0.5 -> 1.0 m |
| FAST-LIO2 | scan voxel 0.5 -> 0.25 m; map voxel 0.5 -> 0.25 m; IMU noise; iterations 3 -> 5 |
| GLIM | iVox 1.0 -> 0.5 m; points 10000 -> 20000; IMU noise and bias random walk; GICP -> VGICP |
| EllipseLIO | map 0.1 -> 0.2 m or 0.05 m; IMU noise; IMU bias random walk |

IMU noise alternatives are the simulated sensor's, in each method's convention. EllipseLIO's two
map resolutions are never combined (12 configurations); its core clamps the finer one to 0.1 m.
