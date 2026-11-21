/* Exports a published rawlog to plain-text tables, one per stream.
 *
 * Usage: rawlog-to-text IN.rawlog OUT_DIR [COMPRESS_CMD [EXT]]
 *
 * Writes into OUT_DIR:
 *   lidar.txt       one point per line: t_ns x y z ring label instance, sweep
 *                   after sweep (sensor frame at the point's own firing time;
 *                   x y z in m with 1 mm resolution; t_ns: firing time
 *                   relative to the sweep stamp, ns)
 *   sweeps.txt      sweep stamp_s num_points (the points of each sweep follow
 *                   those of the previous one in lidar.txt)
 *   imu.txt         stamp_s wx wy wz ax ay az (rad/s, m/s^2, IMU frame)
 *   odometry.txt    stamp_s x y yaw vx vy omega (planar, base_link)
 *   extrinsics.yaml sensor poses in base_link
 * With COMPRESS_CMD (e.g. "xz -9 -T1"), lidar.txt is piped through it and
 * gets the extension EXT (default ".xz"); the other tables are written plain.
 */
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

namespace
{
std::string compressCmd;
std::string compressExt = ".xz";

class TextOut
{
   public:
	TextOut(const std::string& dir, const std::string& name)
	{
		const std::string path = dir + "/" + name;
		// Only the point table goes through the compressor: each compressor
		// process may reserve much memory, and the other tables are small.
		if (compressCmd.empty() || name != "lidar.txt")
		{
			f_ = std::fopen(path.c_str(), "wb");
		}
		else
		{
			piped_ = true;
			const std::string cmd = compressCmd + " > '" + path + compressExt + "'";
			f_ = popen(cmd.c_str(), "w");
		}
		if (!f_)
		{
			throw std::runtime_error("Cannot write " + path);
		}
		buf_.reserve(kBufSize + 256);
	}
	~TextOut() { close(); }
	TextOut(const TextOut&) = delete;
	TextOut& operator=(const TextOut&) = delete;

	void close()
	{
		if (!f_)
		{
			return;
		}
		flush();
		const int rc = piped_ ? pclose(f_) : std::fclose(f_);
		f_ = nullptr;
		if (rc != 0)
		{
			throw std::runtime_error("Error closing an output stream");
		}
	}

	void str(const char* s) { buf_.append(s); }
	void chr(char c) { buf_.push_back(c); }

	void integer(int64_t v)
	{
		char tmp[24];
		const int n = std::snprintf(tmp, sizeof(tmp), "%lld", static_cast<long long>(v));
		buf_.append(tmp, static_cast<size_t>(n));
	}

	// Fixed-point decimal with `decimals` digits, from a value already scaled
	// by 10^decimals and rounded.
	void fixed(int64_t scaled, int decimals)
	{
		if (scaled < 0)
		{
			buf_.push_back('-');
			scaled = -scaled;
		}
		int64_t p = 1;
		for (int i = 0; i < decimals; i++)
		{
			p *= 10;
		}
		integer(scaled / p);
		buf_.push_back('.');
		char tmp[24];
		std::snprintf(tmp, sizeof(tmp), "%0*lld", decimals, static_cast<long long>(scaled % p));
		buf_.append(tmp);
	}

	void dbl(double v)
	{
		char tmp[40];
		const int n = std::snprintf(tmp, sizeof(tmp), "%.9g", v);
		buf_.append(tmp, static_cast<size_t>(n));
	}

	void endl()
	{
		buf_.push_back('\n');
		if (buf_.size() >= kBufSize)
		{
			flush();
		}
	}

   private:
	static constexpr size_t kBufSize = 1 << 22;
	void flush()
	{
		if (!buf_.empty() && std::fwrite(buf_.data(), 1, buf_.size(), f_) != buf_.size())
		{
			throw std::runtime_error("Write error");
		}
		buf_.clear();
	}
	std::FILE* f_ = nullptr;
	bool piped_ = false;
	std::string buf_;
};

// Seconds since the Unix epoch, 7 decimals (MRPT clock ticks are 100 ns).
void stamp(TextOut& o, mrpt::Clock::time_point t)
{
	const int64_t ticks = static_cast<int64_t>(std::llround(mrpt::Clock::toDouble(t) * 1e7));
	o.fixed(ticks, 7);
}

int64_t scaled(double v, double scale) { return static_cast<int64_t>(std::llround(v * scale)); }

void writePose(std::FILE* f, const char* name, const char* label, const mrpt::poses::CPose3D& p)
{
	const mrpt::poses::CPose3DQuat q(p);
	std::fprintf(f, "%s:\n", name);
	std::fprintf(f, "  label: %s\n", label);
	std::fprintf(f, "  parent_frame: base_link\n");
	std::fprintf(f, "  translation_m: [%.9g, %.9g, %.9g]\n", p.x(), p.y(), p.z());
	std::fprintf(
		f, "  rotation_quaternion_xyzw: [%.9g, %.9g, %.9g, %.9g]\n", q.quat().x(), q.quat().y(),
		q.quat().z(), q.quat().r());
	std::fprintf(
		f, "  yaw_pitch_roll_deg: [%.9g, %.9g, %.9g]\n", mrpt::RAD2DEG(p.yaw()),
		mrpt::RAD2DEG(p.pitch()), mrpt::RAD2DEG(p.roll()));
}
}  // namespace

int main(int argc, char** argv)
{
	if (argc < 3 || argc > 5)
	{
		std::fprintf(stderr, "Usage: %s IN.rawlog OUT_DIR [COMPRESS_CMD [EXT]]\n", argv[0]);
		return 2;
	}
	if (argc >= 4)
	{
		compressCmd = argv[3];
	}
	if (argc >= 5)
	{
		compressExt = argv[4];
	}
	const std::string dir = argv[2];
	try
	{
		mrpt::io::CFileGZInputStream fin(argv[1]);
		auto arch = mrpt::serialization::archiveFrom(fin);

		TextOut lidar(dir, "lidar.txt");
		TextOut sweeps(dir, "sweeps.txt");
		TextOut imu(dir, "imu.txt");
		TextOut odom(dir, "odometry.txt");
		lidar.str("# t_ns x y z ring label instance\n");
		sweeps.str("# sweep stamp_s num_points\n");
		imu.str("# stamp_s wx wy wz ax ay az\n");
		odom.str("# stamp_s x y yaw vx vy omega\n");

		bool haveLidarPose = false;
		bool haveImuPose = false;
		mrpt::poses::CPose3D lidarPose;
		mrpt::poses::CPose3D imuPose;
		std::string lidarLabel;
		std::string imuLabel;
		int64_t nSweeps = 0;
		int64_t nPoints = 0;
		int64_t nImu = 0;
		int64_t nOdom = 0;

		for (;;)
		{
			mrpt::serialization::CSerializable::Ptr obj;
			try
			{
				arch >> obj;
			}
			catch (const mrpt::serialization::CExceptionEOF&)
			{
				break;
			}
			if (!obj)
			{
				break;
			}
			if (auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obj); o)
			{
				const auto pc =
					std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(o->pointcloud);
				if (!pc)
				{
					throw std::runtime_error("Point cloud is not a CGenericPointsMap");
				}
				if (!haveLidarPose)
				{
					lidarPose = o->sensorPose;
					lidarLabel = o->sensorLabel;
					haveLidarPose = true;
				}
				const auto& xs = pc->getPointsBufferRef_x();
				const auto& ys = pc->getPointsBufferRef_y();
				const auto& zs = pc->getPointsBufferRef_z();
				const auto* ts = pc->getPointsBufferRef_float_field("t");
				const auto* rings = pc->getPointsBufferRef_uint16_field("ring");
				const auto* labels = pc->getPointsBufferRef_uint16_field("label");
				const auto* insts = pc->getPointsBufferRef_uint16_field("instance");
				const size_t n = xs.size();
				for (size_t i = 0; i < n; i++)
				{
					lidar.integer(ts ? scaled((*ts)[i], 1e9) : 0);
					lidar.chr(' ');
					lidar.fixed(scaled(xs[i], 1e3), 3);
					lidar.chr(' ');
					lidar.fixed(scaled(ys[i], 1e3), 3);
					lidar.chr(' ');
					lidar.fixed(scaled(zs[i], 1e3), 3);
					lidar.chr(' ');
					lidar.integer(rings ? (*rings)[i] : 0);
					lidar.chr(' ');
					lidar.integer(labels ? (*labels)[i] : 0);
					lidar.chr(' ');
					lidar.integer(insts ? (*insts)[i] : 0);
					lidar.endl();
				}
				sweeps.integer(nSweeps);
				sweeps.chr(' ');
				stamp(sweeps, o->timestamp);
				sweeps.chr(' ');
				sweeps.integer(static_cast<int64_t>(n));
				sweeps.endl();
				nSweeps++;
				nPoints += static_cast<int64_t>(n);
			}
			else if (auto m = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obj); m)
			{
				using namespace mrpt::obs;
				if (!haveImuPose)
				{
					imuPose = m->sensorPose;
					imuLabel = m->sensorLabel;
					haveImuPose = true;
				}
				stamp(imu, m->timestamp);
				for (const auto idx : {IMU_WX, IMU_WY, IMU_WZ, IMU_X_ACC, IMU_Y_ACC, IMU_Z_ACC})
				{
					imu.chr(' ');
					imu.dbl(m->rawMeasurements.at(idx));
				}
				imu.endl();
				nImu++;
			}
			else if (auto d = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(obj); d)
			{
				stamp(odom, d->timestamp);
				for (const double v :
					 {d->odometry.x(), d->odometry.y(), d->odometry.phi(), d->velocityLocal.vx,
					  d->velocityLocal.vy, d->velocityLocal.omega})
				{
					odom.chr(' ');
					odom.dbl(v);
				}
				odom.endl();
				nOdom++;
			}
		}
		lidar.close();
		sweeps.close();
		imu.close();
		odom.close();

		const std::string ypath = dir + "/extrinsics.yaml";
		std::FILE* y = std::fopen(ypath.c_str(), "w");
		if (!y)
		{
			throw std::runtime_error("Cannot write " + ypath);
		}
		writePose(y, "lidar", lidarLabel.c_str(), lidarPose);
		writePose(y, "imu", imuLabel.c_str(), imuPose);
		std::fclose(y);

		std::printf(
			"sweeps %lld points %lld imu %lld odometry %lld\n", static_cast<long long>(nSweeps),
			static_cast<long long>(nPoints), static_cast<long long>(nImu),
			static_cast<long long>(nOdom));
	}
	catch (const std::exception& e)
	{
		std::fprintf(stderr, "Error: %s\n", e.what());
		return 1;
	}
	return 0;
}
