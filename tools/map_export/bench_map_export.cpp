/* Builds a point-cloud map from an MVSim LIO benchmark rawlog and a trajectory.
 *
 * Every LiDAR point is placed with the trajectory interpolated at its own
 * firing time (sweep stamp + per-point "t"), so a rolling-shutter scan is
 * de-skewed by that same trajectory. With the ground truth this gives the
 * exact map of what the sensor saw; with an estimated trajectory, the map that
 * the estimate implies.
 *
 * Output: binary little-endian float32 records (x, y, z, t_first), one per
 * occupied voxel: the centroid of its points and the time it was first seen.
 *
 * Usage:
 *   bench-map-export --rawlog f.rawlog --count-only
 *       Prints the number of LiDAR sweeps, points, IMU and odometry samples.
 *   bench-map-export --rawlog f.rawlog --traj base_link.tum --out map.bin
 *       [--voxel 0.1] [--every 1] [--label lidar1] [--offset 0]
 *       [--align "r11 r12 r13 tx r21 ... tz"] [--no-deskew] [--t-start T] [--t-end T]
 */

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
struct Knot
{
  double t = 0;
  std::array<double, 3> p{};
  std::array<double, 4> q{};  // x y z w
};

std::vector<Knot> loadTum(const std::string& path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot open " + path);
  }
  std::vector<Knot> out;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    Knot k;
    if (ss >> k.t >> k.p[0] >> k.p[1] >> k.p[2] >> k.q[0] >> k.q[1] >> k.q[2] >> k.q[3]) {
      out.push_back(k);
    }
  }
  std::sort(out.begin(), out.end(), [](const Knot& a, const Knot& b) { return a.t < b.t; });
  return out;
}

// Pose at time t: linear position, slerp rotation. False outside the knots.
bool interpolate(const std::vector<Knot>& k, double t, mrpt::poses::CPose3D& out)
{
  if (k.size() < 2 || t < k.front().t || t > k.back().t) {
    return false;
  }
  auto it = std::upper_bound(
      k.begin(), k.end(), t, [](double v, const Knot& kn) { return v < kn.t; });
  if (it == k.end()) {
    it = k.end() - 1;
  }
  const Knot& b = *it;
  const Knot& a = *(it - 1);
  const double dt = b.t - a.t;
  const double s = dt > 0 ? (t - a.t) / dt : 0.0;
  std::array<double, 4> qb = b.q;
  double dot = a.q[0] * qb[0] + a.q[1] * qb[1] + a.q[2] * qb[2] + a.q[3] * qb[3];
  if (dot < 0) {
    for (auto& v : qb) {
      v = -v;
    }
    dot = -dot;
  }
  std::array<double, 4> q{};
  if (dot > 0.9995) {
    for (int i = 0; i < 4; i++) {
      q[i] = a.q[i] + s * (qb[i] - a.q[i]);
    }
  } else {
    const double th = std::acos(dot);
    const double wa = std::sin((1 - s) * th) / std::sin(th);
    const double wb = std::sin(s * th) / std::sin(th);
    for (int i = 0; i < 4; i++) {
      q[i] = wa * a.q[i] + wb * qb[i];
    }
  }
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  const mrpt::math::CQuaternionDouble mq(q[3] / n, q[0] / n, q[1] / n, q[2] / n);
  out = mrpt::poses::CPose3D(mrpt::poses::CPose3DQuat(
      a.p[0] + s * (b.p[0] - a.p[0]), a.p[1] + s * (b.p[1] - a.p[1]),
      a.p[2] + s * (b.p[2] - a.p[2]), mq));
  return true;
}

struct Voxel
{
  double sx = 0;
  double sy = 0;
  double sz = 0;
  uint32_t n = 0;
  float tFirst = 0;
};

int64_t voxelKey(double x, double y, double z, double res)
{
  const int64_t ix = static_cast<int64_t>(std::floor(x / res)) + (1 << 20);
  const int64_t iy = static_cast<int64_t>(std::floor(y / res)) + (1 << 20);
  const int64_t iz = static_cast<int64_t>(std::floor(z / res)) + (1 << 20);
  return (ix << 42) | (iy << 21) | iz;
}

std::string argValue(int argc, char** argv, const std::string& name, const std::string& def)
{
  for (int i = 1; i + 1 < argc; i++) {
    if (name == argv[i]) {
      return argv[i + 1];
    }
  }
  return def;
}

bool hasFlag(int argc, char** argv, const std::string& name)
{
  for (int i = 1; i < argc; i++) {
    if (name == argv[i]) {
      return true;
    }
  }
  return false;
}
}  // namespace

int main(int argc, char** argv)
{
  try {
    const std::string rawlogFile = argValue(argc, argv, "--rawlog", "");
    if (!rawlogFile.empty() && hasFlag(argc, argv, "--count-only")) {
      const std::string lbl = argValue(argc, argv, "--label", "lidar1");
      mrpt::io::CFileGZInputStream f(rawlogFile);
      auto a = mrpt::serialization::archiveFrom(f);
      size_t sweeps = 0;
      size_t points = 0;
      size_t imu = 0;
      size_t odom = 0;
      size_t idx = 0;
      for (;;) {
        mrpt::obs::CActionCollection::Ptr acts;
        mrpt::obs::CSensoryFrame::Ptr sf;
        mrpt::obs::CObservation::Ptr obs;
        if (!mrpt::obs::CRawlog::getActionObservationPairOrObservation(a, acts, sf, obs, idx)) {
          break;
        }
        if (auto pc = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obs);
            pc && pc->sensorLabel == lbl && pc->pointcloud) {
          sweeps++;
          points += pc->pointcloud->size();
        } else if (std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obs)) {
          imu++;
        } else if (std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(obs)) {
          odom++;
        }
      }
      std::cout << "{\"sweeps\": " << sweeps << ", \"points\": " << points
                << ", \"imu\": " << imu << ", \"odom\": " << odom << "}\n";
      return 0;
    }
    const std::string trajFile = argValue(argc, argv, "--traj", "");
    const std::string outFile = argValue(argc, argv, "--out", "");
    if (rawlogFile.empty() || trajFile.empty() || outFile.empty()) {
      std::cerr << "Usage: bench-map-export --rawlog R --traj T.tum --out M.bin [--voxel 0.1]"
                   " [--every 1] [--label lidar1] [--offset 0] [--align \"12 numbers\"]"
                   " [--no-deskew] [--t-start T] [--t-end T]\n";
      return 1;
    }
    const double voxel = std::stod(argValue(argc, argv, "--voxel", "0.1"));
    const int every = std::stoi(argValue(argc, argv, "--every", "1"));
    const std::string label = argValue(argc, argv, "--label", "lidar1");
    const double offset = std::stod(argValue(argc, argv, "--offset", "0"));
    const double tEnd = std::stod(argValue(argc, argv, "--t-end", "1e300"));
    const double tStart = std::stod(argValue(argc, argv, "--t-start", "-1e300"));
    const bool deskew = !hasFlag(argc, argv, "--no-deskew");

    // Optional rigid alignment, left-composed with every trajectory pose:
    mrpt::poses::CPose3D align;
    if (const std::string al = argValue(argc, argv, "--align", ""); !al.empty()) {
      std::istringstream ss(al);
      mrpt::math::CMatrixDouble44 M = mrpt::math::CMatrixDouble44::Identity();
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 4; c++) {
          ss >> M(r, c);
        }
      }
      align = mrpt::poses::CPose3D(M);
    }

    const auto traj = loadTum(trajFile);
    if (traj.size() < 2) {
      throw std::runtime_error("trajectory too short: " + trajFile);
    }

    mrpt::io::CFileGZInputStream fin(rawlogFile);
    auto arch = mrpt::serialization::archiveFrom(fin);
    std::unordered_map<int64_t, Voxel> map;
    size_t scanIdx = 0;
    size_t used = 0;
    size_t nPoints = 0;
    size_t entry = 0;
    for (;;) {
      mrpt::obs::CActionCollection::Ptr acts;
      mrpt::obs::CSensoryFrame::Ptr sf;
      mrpt::obs::CObservation::Ptr obs;
      if (!mrpt::obs::CRawlog::getActionObservationPairOrObservation(arch, acts, sf, obs, entry)) {
        break;
      }
      auto pc = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obs);
      if (!pc || pc->sensorLabel != label || !pc->pointcloud) {
        continue;
      }
      if (scanIdx++ % every != 0) {
        continue;
      }
      const double t0 = mrpt::Clock::toDouble(pc->timestamp) + offset;
      if (t0 > tEnd) {
        break;
      }
      if (t0 < tStart) {
        continue;
      }
      const auto& xs = pc->pointcloud->getPointsBufferRef_x();
      const auto& ys = pc->pointcloud->getPointsBufferRef_y();
      const auto& zs = pc->pointcloud->getPointsBufferRef_z();
      const auto* ts = pc->pointcloud->getPointsBufferRef_float_field("t");
      const size_t n = xs.size();
      nPoints += n;
      mrpt::poses::CPose3D lastPose;
      double lastT = -1e300;
      bool lastOk = false;
      for (size_t i = 0; i < n; i++) {
        const double ti = t0 + ((deskew && ts) ? static_cast<double>((*ts)[i]) : 0.0);
        // Points of one column share their time: interpolate once per change.
        if (ti != lastT) {
          mrpt::poses::CPose3D base;
          lastOk = interpolate(traj, ti, base);
          if (lastOk) {
            lastPose = align + base + pc->sensorPose;
          }
          lastT = ti;
        }
        if (!lastOk) {
          continue;
        }
        double gx = 0;
        double gy = 0;
        double gz = 0;
        lastPose.composePoint(xs[i], ys[i], zs[i], gx, gy, gz);
        auto& v = map[voxelKey(gx, gy, gz, voxel)];
        if (v.n == 0) {
          v.tFirst = static_cast<float>(ti - traj.front().t);
        }
        v.sx += gx;
        v.sy += gy;
        v.sz += gz;
        v.n++;
      }
      used++;
    }

    std::ofstream fout(outFile, std::ios::binary);
    for (const auto& [key, v] : map) {
      const std::array<float, 4> rec = {
          static_cast<float>(v.sx / v.n), static_cast<float>(v.sy / v.n),
          static_cast<float>(v.sz / v.n), v.tFirst};
      fout.write(reinterpret_cast<const char*>(rec.data()), sizeof(rec));
    }
    std::cout << "scans used: " << used << " points: " << nPoints << " voxels: " << map.size() << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
