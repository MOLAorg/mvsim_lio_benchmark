/* Compares the LiDAR point clouds of two rawlogs of the same sequence, one
 * sweep at a time: positions, per-point time "t" and "ring". Prints the label
 * histogram of the second one, if it has labels.
 *
 * Usage: compare-rawlogs A.rawlog B.rawlog
 * Exit code 0 if both have the same sweeps and identical points.
 */
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/serialization/CArchive.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>

namespace
{
class SweepReader
{
   public:
	explicit SweepReader(const std::string& path)
		: f_(path), arch_(mrpt::serialization::archiveFrom(f_))
	{
	}

	[[nodiscard]] bool failed() const { return failed_; }

	mrpt::maps::CGenericPointsMap::Ptr next()
	{
		for (;;)
		{
			mrpt::serialization::CSerializable::Ptr obj;
			try
			{
				arch_ >> obj;
			}
			catch (const mrpt::serialization::CExceptionEOF&)
			{
				return {};
			}
			catch (const std::exception& e)
			{
				std::fprintf(stderr, "Read error: %s\n", e.what());
				failed_ = true;
				return {};
			}
			if (!obj)
			{
				return {};
			}
			if (auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obj); o)
			{
				return std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(o->pointcloud);
			}
		}
	}

   private:
	mrpt::io::CFileGZInputStream f_;
	mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream> arch_;
	bool failed_ = false;
};
}  // namespace

int main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::fprintf(stderr, "Usage: %s A.rawlog B.rawlog\n", argv[0]);
		return 2;
	}
	SweepReader ra(argv[1]);
	SweepReader rb(argv[2]);
	size_t nSweepsA = 0;
	size_t nSweepsB = 0;
	size_t nSizeMismatch = 0;
	size_t nDiff = 0;
	size_t nPts = 0;
	size_t nInst = 0;
	double maxDxyz = 0;
	double maxDt = 0;
	std::map<uint16_t, size_t> hist;
	for (;;)
	{
		const auto a = ra.next();
		const auto b = rb.next();
		nSweepsA += a ? 1 : 0;
		nSweepsB += b ? 1 : 0;
		if (!a || !b)
		{
			if (a || b)
			{
				// Count the rest of the longer file.
				auto& r = a ? ra : rb;
				auto& n = a ? nSweepsA : nSweepsB;
				while (r.next())
				{
					n++;
				}
			}
			break;
		}
		const bool labels = b->hasPointField("label");
		if (labels)
		{
			for (size_t i = 0; i < b->size(); i++)
			{
				hist[b->getPointField_uint16(i, "label")]++;
				nInst += b->getPointField_uint16(i, "instance") != 0 ? 1 : 0;
			}
		}
		nPts += b->size();
		if (a->size() != b->size())
		{
			nSizeMismatch++;
			continue;
		}
		for (size_t i = 0; i < a->size(); i++)
		{
			float xa = 0;
			float ya = 0;
			float za = 0;
			float xb = 0;
			float yb = 0;
			float zb = 0;
			a->getPoint(i, xa, ya, za);
			b->getPoint(i, xb, yb, zb);
			const float ta = a->getPointField_float(i, "t");
			const float tb = b->getPointField_float(i, "t");
			if (xa != xb || ya != yb || za != zb || ta != tb ||
				a->getPointField_uint16(i, "ring") != b->getPointField_uint16(i, "ring"))
			{
				nDiff++;
				maxDxyz = std::max<double>(
					maxDxyz, std::max({std::abs(xa - xb), std::abs(ya - yb), std::abs(za - zb)}));
				maxDt = std::max<double>(maxDt, std::abs(ta - tb));
			}
		}
	}
	std::printf("sweeps: %zu vs %zu, size mismatches: %zu\n", nSweepsA, nSweepsB, nSizeMismatch);
	std::printf(
		"points: %zu, different: %zu, max |dxyz| %.3g m, max |dt| %.3g s\n", nPts, nDiff, maxDxyz,
		maxDt);
	for (const auto& [l, n] : hist)
	{
		std::printf("label %3u: %6.2f %%\n", l, 100.0 * n / std::max<size_t>(1, nPts));
	}
	std::printf("with an instance: %.2f %%\n", 100.0 * nInst / std::max<size_t>(1, nPts));
	const bool same =
		!ra.failed() && !rb.failed() && nDiff == 0 && nSizeMismatch == 0 && nSweepsA == nSweepsB;
	return same ? 0 : 1;
}
