/* Copies a rawlog, setting the sensor pose (in the vehicle frame) of all its
 * IMU observations. Every other object is copied unchanged.
 *
 * Usage: set-imu-pose IN.rawlog OUT.rawlog x y z yaw_deg pitch_deg roll_deg
 * Exit code 0 if the input had IMU observations and was read to the end.
 */
#include <mrpt/core/bits_math.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char** argv)
{
	if (argc != 9)
	{
		std::fprintf(
			stderr, "Usage: %s IN.rawlog OUT.rawlog x y z yaw_deg pitch_deg roll_deg\n", argv[0]);
		return 2;
	}
	// Keeps the point map classes linked and registered, for deserialization:
	[[maybe_unused]] const auto* pointsClass = CLASS_ID(mrpt::maps::CGenericPointsMap);
	const auto pose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
		std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]),
		mrpt::DEG2RAD(std::atof(argv[6])), mrpt::DEG2RAD(std::atof(argv[7])),
		mrpt::DEG2RAD(std::atof(argv[8])));
	mrpt::io::CFileGZInputStream fi(argv[1]);
	mrpt::io::CFileGZOutputStream fo(argv[2]);
	auto in = mrpt::serialization::archiveFrom(fi);
	auto out = mrpt::serialization::archiveFrom(fo);
	size_t nObj = 0;
	size_t nImu = 0;
	for (;;)
	{
		mrpt::serialization::CSerializable::Ptr obj;
		try
		{
			in >> obj;
		}
		catch (const mrpt::serialization::CExceptionEOF&)
		{
			break;
		}
		catch (const std::exception& e)
		{
			std::fprintf(stderr, "Read error after %zu objects: %s\n", nObj, e.what());
			return 1;
		}
		if (!obj)
		{
			break;
		}
		if (auto imu = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obj); imu)
		{
			imu->sensorPose = pose;
			nImu++;
		}
		out << *obj;
		nObj++;
	}
	std::printf("objects: %zu, IMU observations: %zu\n", nObj, nImu);
	return nImu > 0 ? 0 : 1;
}
