# Results of the paper

Per-run metrics of every configuration on every variant and sequence, as produced by
`collect.py` and `eval_traj.py` (`traj_*.tsv`), map metrics (`maps_*.tsv`, `make_maps.py`),
the tilt and yaw-and-translation APE of the `real` OS1-32 variants (`tilt_os1_32-real.tsv`),
the difficulty score and class of each sequence (`difficulty.tsv`) with the results per class
(`summary_by_difficulty.tsv`), and the scores of the tuning grid on the training split
(`tuning_scores.tsv`).

`traj_*.tsv` columns: `status` (`ok`, `diverged`, `incomplete`, `no_output`, `memory_limit`),
`coverage` and `len_ratio` (the failure rule), `ape` (SE(3)-aligned position RMSE, m),
`ape_max`, `ape_z`, `ape_rot` (deg), `drift_pct` (end point, first pose aligned, % of the path),
`rte10` and `rte50` (%), `rre10` (deg/m), `n_poses`. Method names: `<method>` for the
defaults, `<method>+t<bits>` for a tuning configuration (`../configs/tuning.json`), and
`mola_lo+map1km`, `mola_lio+map1km`, `fastlio2+map200m` for the map-extent variants
(`../configs/overrides.json`).

`timing_desktop.tsv`: runtime of every run of the synthetic variants on a desktop CPU (Ryzen 7
2700X, one physical core per run, single-threaded): `wall_s`, `cpu_s`, `maxrss_mb`, and
`rt_factor` (sequence duration over wall time). `status` is that of `traj_*.tsv`; the timed runs
give the same outcome and APE.
