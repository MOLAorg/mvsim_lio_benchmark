/* Ray casting and LiDAR sweep simulation on a triangle mesh, with the LiDAR
 * model of the MVSim dataset generator (mvsim-dataset-gen/LidarSimulator.cpp,
 * compiled in unchanged).
 *
 *  lidar_tool cast <mesh.ply> <rays.bin> <hits.bin> [threads]
 *    rays.bin: N x 6 float64 (origin xyz, direction xyz) in the mesh frame.
 *    hits.bin: N x 4 float32 (range, or 0 if no hit; unit normal xyz).
 *
 *  lidar_tool sim <mesh.ply> <poses.tum> <sweeps.txt> <elev_deg.txt> <opts.txt> <out.bin>
 *    poses.tum: sensor poses in the mesh frame (linear/slerp interpolated).
 *    sweeps.txt: one sweep start time [s] per line.
 *    elev_deg.txt: one ring elevation [deg] per line.
 *    opts.txt: "key value" lines: cols period min_range max_range seed
 *      sigma0 k divergence_deg max_incidence_deg p_mix elev_std_deg
 *      azim_std_deg bias_std calib_seed.
 *    out.bin: per sweep, int32 count, then count x (float32 x y z t,
 *      uint16 ring, uint16 label); label 1 marks a mixed return.
 */

#include <mvsim/raytracer/Lidar3DModel.h>
#include <mvsim/raytracer/RayScene.h>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "LidarSimulator.h"
#include "TrajectorySource.h"

namespace
{
void loadMesh(const std::string& path, mvsim::rt::RayScene& scene)
{
	std::ifstream f(path, std::ios::binary);
	if (!f)
	{
		throw std::runtime_error("cannot open " + path);
	}
	std::string line;
	size_t nVerts = 0;
	size_t nFaces = 0;
	while (std::getline(f, line))
	{
		std::istringstream ls(line);
		std::string a;
		std::string b;
		ls >> a >> b;
		if (a == "element" && b == "vertex")
		{
			ls >> nVerts;
		}
		else if (a == "element" && b == "face")
		{
			ls >> nFaces;
		}
		else if (a == "end_header")
		{
			break;
		}
	}
	std::vector<float> v(3 * nVerts);
	f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(v.size() * sizeof(float)));
	for (size_t i = 0; i < nFaces; i++)
	{
		uint8_t n = 0;
		int32_t idx[3];
		f.read(reinterpret_cast<char*>(&n), 1);
		f.read(reinterpret_cast<char*>(idx), sizeof(idx));
		if (!f || n != 3)
		{
			throw std::runtime_error("bad face record in " + path);
		}
		mvsim::rt::Triangle tri;
		const auto vert = [&](int k)
		{
			return mrpt::math::TPoint3D(v[3 * k], v[3 * k + 1], v[3 * k + 2]);
		};
		tri.v0 = vert(idx[0]);
		tri.v1 = vert(idx[1]);
		tri.v2 = vert(idx[2]);
		mvsim::rt::Primitive prim;
		prim.geometry = tri;
		prim.semanticClass = 2;
		scene.addPrimitive(std::move(prim));
	}
	std::cerr << "mesh: " << nVerts << " vertices, " << nFaces << " faces; building BVH\n";
	scene.build();
}

int cmdCast(int argc, char** argv)
{
	if (argc < 5)
	{
		throw std::runtime_error("usage: cast <mesh.ply> <rays.bin> <hits.bin> [threads]");
	}
	mvsim::rt::RayScene scene;
	loadMesh(argv[2], scene);
	std::ifstream fr(argv[3], std::ios::binary | std::ios::ate);
	const size_t bytes = static_cast<size_t>(fr.tellg());
	fr.seekg(0);
	const size_t n = bytes / (6 * sizeof(double));
	std::vector<double> rays(6 * n);
	fr.read(reinterpret_cast<char*>(rays.data()), static_cast<std::streamsize>(bytes));
	std::vector<float> hits(4 * n, 0.0f);
	const unsigned nThreads = argc > 5 ? static_cast<unsigned>(std::stoi(argv[5])) : 8;
	std::vector<std::thread> pool;
	for (unsigned th = 0; th < nThreads; th++)
	{
		pool.emplace_back(
			[&, th]()
			{
				for (size_t i = th; i < n; i += nThreads)
				{
					mvsim::rt::Ray ray;
					ray.org = {rays[6 * i], rays[6 * i + 1], rays[6 * i + 2]};
					ray.dir = {rays[6 * i + 3], rays[6 * i + 4], rays[6 * i + 5]};
					ray.tMin = 0.05;
					ray.tMax = 200.0;
					const auto hit = scene.castRay(ray);
					if (!hit)
					{
						continue;
					}
					hits[4 * i] = static_cast<float>(hit->t);
					hits[4 * i + 1] = static_cast<float>(hit->normal.x);
					hits[4 * i + 2] = static_cast<float>(hit->normal.y);
					hits[4 * i + 3] = static_cast<float>(hit->normal.z);
				}
			});
	}
	for (auto& t : pool)
	{
		t.join();
	}
	std::ofstream fo(argv[4], std::ios::binary);
	fo.write(reinterpret_cast<const char*>(hits.data()), static_cast<std::streamsize>(hits.size() * sizeof(float)));
	std::cerr << "cast " << n << " rays\n";
	return 0;
}

std::map<std::string, double> loadOptions(const std::string& path)
{
	std::ifstream f(path);
	std::map<std::string, double> out;
	std::string key;
	double value = 0;
	while (f >> key >> value)
	{
		out[key] = value;
	}
	return out;
}

int cmdSim(int argc, char** argv)
{
	if (argc < 8)
	{
		throw std::runtime_error(
			"usage: sim <mesh.ply> <poses.tum> <sweeps.txt> <elev_deg.txt> <opts.txt> <out.bin>");
	}
	mvsim::rt::RayScene scene;
	loadMesh(argv[2], scene);
	mvsim_dataset_gen::TrajectorySource traj;
	traj.loadTum(argv[3]);
	std::vector<double> sweeps;
	{
		std::ifstream f(argv[4]);
		double t = 0;
		while (f >> t)
		{
			sweeps.push_back(t);
		}
	}
	mvsim::rt::Lidar3DModel::Params mp;
	{
		std::ifstream f(argv[5]);
		double e = 0;
		while (f >> e)
		{
			mp.verticalRayAnglesDeg.push_back(e);
		}
	}
	auto o = loadOptions(argv[6]);
	mp.horzNumRays = static_cast<int>(o.at("cols"));
	mp.sensorPeriod = o.at("period");
	mp.minRange = static_cast<float>(o.at("min_range"));
	mp.maxRange = static_cast<float>(o.at("max_range"));
	const mvsim::rt::Lidar3DModel model(mp);

	mvsim_dataset_gen::LidarSimOptions opts;
	opts.shutter = mvsim_dataset_gen::ShutterMode::Rolling;
	opts.generateLabels = true;
	opts.mixedReturnLabel = 1;
	opts.rangeStdNoise = o.at("sigma0");
	opts.rangeStdNoisePerMeter = o.at("k");
	opts.beamDivergenceDeg = o.at("divergence_deg");
	opts.maxIncidenceDeg = o.at("max_incidence_deg");
	opts.mixedPixelProbability = o.at("p_mix");
	// Beam model and detection range (absent: disabled):
	opts.echoSeparation = o.count("echo_separation") ? o.at("echo_separation") : 0.0;
	opts.detectionRange = o.count("detection_range") ? o.at("detection_range") : 0.0;
	mvsim_dataset_gen::drawCalibrationErrors(
		opts, model.vertRays(), o.at("elev_std_deg"), o.at("azim_std_deg"), o.at("bias_std"),
		static_cast<int>(o.at("calib_seed")));

	std::mt19937 rng(static_cast<std::uint32_t>(o.at("seed")));
	std::ofstream fo(argv[7], std::ios::binary);
	const std::vector<mvsim_dataset_gen::MovingObject> noMovingObjects;
	for (const double t0 : sweeps)
	{
		const auto obs = mvsim_dataset_gen::simulateLidarSweep(
			scene, noMovingObjects, model, traj, mrpt::poses::CPose3D(), t0, "lidar", opts, rng);
		const auto& pts = *obs->pointcloud;
		const auto& xs = pts.getPointsBufferRef_x();
		const auto& ys = pts.getPointsBufferRef_y();
		const auto& zs = pts.getPointsBufferRef_z();
		const auto count = static_cast<int32_t>(xs.size());
		fo.write(reinterpret_cast<const char*>(&count), sizeof(count));
		for (int32_t i = 0; i < count; i++)
		{
			const auto k = static_cast<size_t>(i);
			const float rec[4] = {xs[k], ys[k], zs[k], pts.getPointField_float(k, "t")};
			const uint16_t tags[2] = {
				pts.getPointField_uint16(k, "ring"), pts.getPointField_uint16(k, "label")};
			fo.write(reinterpret_cast<const char*>(rec), sizeof(rec));
			fo.write(reinterpret_cast<const char*>(tags), sizeof(tags));
		}
	}
	std::cerr << "simulated " << sweeps.size() << " sweeps\n";
	return 0;
}
}  // namespace

int main(int argc, char** argv)
{
	try
	{
		const std::string cmd = argc > 1 ? argv[1] : "";
		if (cmd == "cast")
		{
			return cmdCast(argc, argv);
		}
		if (cmd == "sim")
		{
			return cmdSim(argc, argv);
		}
		std::cerr << "usage: lidar_tool cast|sim ...\n";
		return 1;
	}
	catch (const std::exception& e)
	{
		std::cerr << "error: " << e.what() << "\n";
		return 1;
	}
}
