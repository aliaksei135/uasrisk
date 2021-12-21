#include <array>
#include <vector>
#include <gtest/gtest.h>

#include "../src/opensky/OpenSkyCsvReader.h"
#include "../src/openaip/OpenAIPReader.h"
#include "../src/VoxelGrid.h"
#include "../src/VoxelGridBuilder.h"

#include "uasgroundrisk/map_gen/PopulationMap.h"
#include "uasgroundrisk/risk_analysis/RiskMap.h"
#include "uasgroundrisk/risk_analysis/aircraft/AircraftDescentModel.h"
#include "uasgroundrisk/risk_analysis/aircraft/AircraftStateModel.h"
#include "uasgroundrisk/risk_analysis/weather/WeatherMap.h"

#include <omp.h>


TEST(GroundRiskIntegrationTests, FullTest)
{
	const int resolution = 100;
	const std::array<float, 4> xyBounds{
		50.9065510f, -1.4500237f, 50.9517765f,
		-1.3419628f
	};
	const std::array<float, 6> xyzBounds{
		50.9065510f, -1.4500237f, 1, 50.9517765f,
		-1.3419628f, 250
	};

	auto* vg = new ur::VoxelGrid(xyzBounds, resolution, 60);
	ur::VoxelGridBuilder vgb(vg);

	const OpenSkyCsvReader osReader;
	const auto trajs = osReader.readFile("opensky_states.csv");

	// const OpenAIPReader oaReader;
	// const auto airspaces = oaReader.readFile("openaip_airspace_uk.aip");

	vgb.handleTrajectory(trajs);
	// vgb.handleBlockingPolygon(airspaces);

	ugr::risk::AircraftDescentModel descent{90, 2.8, 3.2, 28, 0.6 * 0.6, 0.8, 21, 15};

	ugr::risk::WeatherMap weather(xyBounds, resolution);
	weather.addConstantWind(5, 220);
	weather.eval();

	ugr::mapping::PopulationMap population(xyBounds, resolution);
	population.addOSMLayer("Residential", {{"leisure", "park"}}, 10000);
	population.eval();

	const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

	std::ofstream file(
		"population_map.csv");
	if (file.is_open())
	{
		file << population.get("Population Density").format(CSVFormat);
		file.close();
	}

	const auto& vgSize = vg->getSize();
	std::vector<ur::FPScalar> alts;
	alts.reserve(vgSize[2]);
	for (int i = 0; i < vgSize[2]; ++i)
		alts.emplace_back(vg->local2World(0, 0, i)[2]);

	std::vector<ur::FPScalar> lons(vgSize[0]);
	std::vector<ur::FPScalar> lats(vgSize[1]);
	for (int i = 0; i < vgSize[0]; ++i)
		lons[i] = vg->local2World(0, i, 0)[0];

	for (int i = 0; i < vgSize[1]; ++i)
		lats[i] = vg->local2World(i, 0, 0)[1];

	for (int k = 0; k < alts.size(); ++k)
	{
		ugr::risk::AircraftStateModel state;
		state.position << 0, 0, alts[k];
		state.velocity << 20, 0, 0;

		ugr::risk::RiskMap rm(population, descent, state, weather);
		const auto& sm = rm.generateMap({ugr::risk::RiskType::STRIKE});
		const auto& gsr = sm.get("Glide Strike Risk");
		auto& bsr = sm.get("Ballistic Strike Risk");
		bsr = bsr.array().isNaN().select(0, bsr).eval();
		std::cout << "Risk for altitude: " << alts[k] << "\n";
		std::cout << "Glide Risk max: " << gsr.maxCoeff() << "\n";
		std::cout << "Ballistic Risk max: " << bsr.maxCoeff() << "\n";
		std::cout << "Glide Risk mean: " << gsr.mean() << "\n";
		std::cout << "Ballistic Risk mean: " << bsr.mean() << "\n";
		ugr::gridmap::Matrix csr = (gsr + bsr);

		std::ofstream file("strike_map_" + std::to_string(alts[k]) + ".csv");
		if (file.is_open())
		{
			file << (gsr + bsr).format(CSVFormat);
			file.close();
		}

		assert(vgSize[0] == rm.getSize()[0]);
		assert(vgSize[1] == rm.getSize()[1]);
		#pragma omp parallel for collapse(2)
		for (int i = 0; i < vgSize[0]; ++i)
		{
			for (int j = 0; j < vgSize[1]; ++j)
			{
				auto val = csr(i, j);
				if (isnan(val) || val < 0)
					val = 0;
#pragma omp critical
				vg->at("Ground Risk", {i, j, k}) = val;

				// const auto& worldCoord = vg->local2World(i, j, k);
				// const ugr::gridmap::Position pos(worldCoord[0], worldCoord[1]);
				// const ugr::gridmap::Index& rmIdx = rm.world2Local(pos);
				// if (rm.isInBounds(rmIdx))
				// {
				// 	auto& brisk = rm.at("Ballistic Strike Risk", rmIdx);
				// 	if (isnan(brisk))
				// 		brisk = 0;
				// 	const auto& grisk = rm.at("Glide Strike Risk", rmIdx);
				// 	// std::cout << "B: " << brisk << " G: " << grisk << "\n";
				// 	// #pragma omp critical
				// 	vg->at("Ground Risk", {i, j, k}) = brisk + grisk;
				// }
			}
		}
	}

	std::cout << "All Finished!\n";

	const auto& mar = vg->get("Air Risk").maximum();
	std::cout << "Max Air Risk: " << mar << "\n";
	const auto& mgr = vg->get("Ground Risk").maximum();
	std::cout << "Max Ground Risk: " << mgr << "\n";
	const auto& aar = vg->get("Air Risk").mean();
	std::cout << "Mean Air Risk: " << aar << "\n";
	const auto& agr = vg->get("Ground Risk").mean();
	std::cout << "Mean Ground Risk: " << agr << "\n";

	vg->writeToNetCDF("CombinedVoxelGrid.nc");

	std::cout << "Written to netCDF...\n";
}
