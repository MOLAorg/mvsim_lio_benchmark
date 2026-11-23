# Observability of the sweeps along the true trajectories

Method-independent check of what the LiDAR data constrains (paper: "Observability of the
hard scenes", `tab:observability`). Inputs: the clean global-shutter OS1-32 rawlogs, their
`lidar1.gt.tum`, `gt_200hz.tum`, and the exact reference maps of `make_maps.py ref` (`$LIO_ROOT/maps/ref`).

```bash
E=$LIO_ROOT/observability
# 1. Points of each rawlog to binary (tools/rawlog_tools rawlog-to-text, piped through txt2bin.py)
rawlog-to-text $LIO_DATA/tunnel/ugv_inspection/os1_32-clean-global.rawlog $E/data/tunnel__ugv_inspection "python3 txt2bin.py" .bin
# 2. Per sweep: Cramer-Rao bound (2 cm range noise), ambiguity scan, 1D LIO drift bound
python3 obs.py tunnel/ugv_inspection $E/data/tunnel__ugv_inspection $E/out/tunnel__ugv_inspection
# 3. All points, shifts of 0.5 to 20 m along the direction of travel, every fifth sweep
python3 shift_test.py tunnel/ugv_inspection $E/data/tunnel__ugv_inspection $E/out/tunnel__ugv_inspection
# 4. Summary, results/observability/ and paper/tables/observability.tex
python3 summarize.py $E/out <paper repo>
```

`tunnel_profile.py` splits the tunnel sweeps by position (inside, farther than 70 m from the
portals; straight or bend); `interior_labels.py` attributes the axial information to semantic
classes in the straight sections.
