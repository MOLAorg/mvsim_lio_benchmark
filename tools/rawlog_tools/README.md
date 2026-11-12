# rawlog tools

Two small C++ tools for checking and maintaining the published rawlogs.

- `compare-rawlogs A.rawlog B.rawlog`: compares the LiDAR sweeps of two
  renders of the same sequence (point positions, per-point `t` and `ring`)
  and prints the label histogram of `B`. Exit code 0 only if both have the
  same sweeps with identical points. Used to check that a labeled re-render
  has the same points as the released file.
- `set-imu-pose IN.rawlog OUT.rawlog x y z yaw pitch roll` (meters,
  degrees): copies a rawlog, setting the `sensorPose` of every IMU
  observation. Every other object is copied unchanged. It gave the `imu1`
  observations of the files rendered before MRPT/mvsim#183 their mounting
  pose (`0.05 -0.03 0.45 0 0 0`); the result is byte-identical to a render
  with the fixed generator.

Build (needs MRPT, e.g. from `source tools/env.sh`):

```bash
cmake -S tools/rawlog_tools -B build/rawlog_tools -DCMAKE_BUILD_TYPE=Release
cmake --build build/rawlog_tools
build/rawlog_tools/compare-rawlogs a.rawlog b.rawlog
```
