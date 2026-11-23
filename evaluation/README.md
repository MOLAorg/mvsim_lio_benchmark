# Evaluation code

The scripts that ran the eight LiDAR(-inertial) odometry configurations on the benchmark and
scored them, as used for the paper.

| Script | Does |
|---|---|
| `run_lio.py` | Runs methods on variants and sequences: one job per (variant, sequence, method), single-threaded, pinned to one core, under a memory cap. Writes `<method>.tum`, `.log` and `.time` (wall, CPU time, max RSS). |
| `eval_traj.py` | Scores trajectories against the exact ground truth (the generator's spline at each estimate's stamps): APE, RTE, drift, failures. |
| `collect.py` | Runs `eval_traj.py` over a batch and gathers the metric tables. |
| `make_maps.py`, `map_metrics.py` | Reference map (clean global-shutter sweeps at the true poses) and the map of each estimate; accuracy, completeness, F-score. Needs `tools/map_export`. |
| `tuning.py` | The equal-budget tuning grid (`configs/tuning.json`) and the selection on the training sequences (`configs/tuned.json`). |
| `difficulty.py` | Difficulty classes of the sequences and results per class. |
| `lidar_validation/` | The comparison of the LiDAR error model with real Ouster sensors (its own README). |

`configs/` holds the method configurations (`*-bench.yaml`), the tuning grid and picks, and
the per-run overrides (`overrides.json`, e.g. map extents). `offsets.json` sets each method's
stamp convention for the evaluation.

## Methods

Every method runs through its MOLA wrapper, which reads the rawlogs directly. `COMMITS.tsv`
lists the repositories and commits of the build used for the paper (Release, `-O3 -DNDEBUG`,
ROS 2 lyrical). The wrappers' deterministic options make repeated runs bit-identical.

## Environment

| Variable | Default | Meaning |
|---|---|---|
| `LIO_DATA` | `~/mvsim_lio` | The dataset, as downloaded from the Hub (`<world>/<traj>/<variant>.rawlog`). |
| `LIO_ROOT` | `~/mvsim_lio_eval` | Runs (`runs/`) and results (`results/`). |
| `LIO_WS` | `~/ros2_ws/install` | The ROS 2 install with the methods. |
| `LIO_UNDERLAY` | (none) | Optional install sourced between ROS and `LIO_WS`. |

Example, the defaults of all methods on the two OS1-32 `real` variants:

```bash
python3 evaluation/run_lio.py \
  --variants os1_32-real-global,os1_32-real-rolling \
  --methods mola_lo,mola_lio,kiss,glim,dlio,fastlio2,ellipselio,trajlo \
  --cores-file cores.txt --max-jobs 8 --mem-cap-gb 16
python3 evaluation/collect.py --no-tar
```

`cores.txt` lists the cores the jobs may use, comma-separated, with ranges (e.g. `0-7,16`); it is
re-read before each launch. Traj-LO runs on rolling-shutter variants only.
