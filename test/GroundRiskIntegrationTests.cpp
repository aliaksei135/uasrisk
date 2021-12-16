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

#include <Eigen/Dense>
#include <omp.h>


TEST(GroundRiskIntegrationTests, FullTest)
{
	const int resolution = 60;
	const std::array<float, 4> bounds{
		50.9065510f, -1.4500237f, 50.9517765f,
		-1.3419628f
	};

	auto* vg = new ur::VoxelGrid(120, 150, 12, 60, 60, {-1.4500237, 50.9065510, 5});
	ur::VoxelGridBuilder vgb(vg);

	const OpenSkyCsvReader osReader;
	const auto trajs = osReader.readFile("opensky_states.csv");

	// const OpenAIPReader oaReader;
	// const auto airspaces = oaReader.readFile("openaip_airspace_uk.aip");

	vgb.handleTrajectory(trajs);
	// vgb.handleBlockingPolygon(airspaces);

	ugr::risk::AircraftDescentModel descent{90, 2.8, 3.2, 28, 0.6 * 0.6, 0.8, 21, 15};

	ugr::risk::WeatherMap weather(bounds, resolution);
	weather.addConstantWind(5, 220);
	weather.eval();

	ugr::mapping::PopulationMap population(bounds, resolution);
	population.addOSMLayer("Residential", {{"landuse", "residential"}}, 10000);
	population.eval();

	const auto& vgSize = vg->getSize();
	std::vector<double> alts;
	alts.reserve(vgSize[2]);
	for (int i = 0; i < vgSize[2]; ++i)
		alts.emplace_back(vg->local2World(0, 0, i)[2]);

	for (int k = 0; k < alts.size(); ++k)
	{
		ugr::risk::AircraftStateModel state;
		state.position << 0, 0, alts[k];
		state.velocity << 20, 0, 0;

		ugr::risk::RiskMap rm(population, descent, state, weather);
		const auto& sm = rm.generateMap({ugr::risk::RiskType::STRIKE});
		const auto& gsr = sm.get("Glide Strike Risk");
		const auto& bsr = sm.get("Ballistic Strike Risk");
		std::cout << "Risk for altitude: " << alts[k] << "\n";
		std::cout << "Glide Risk max: " << gsr.maxCoeff() << "\n";
		std::cout << "Ballistic Risk max: " << bsr.maxCoeff() << "\n";
		std::cout << "Glide Risk mean: " << gsr.mean() << "\n";
		std::cout << "Ballistic Risk mean: " << bsr.mean() << "\n";

#pragma omp parallel for collapse(2)
		for (int i = 0; i < vgSize[0]; ++i)
		{
			for (int j = 0; j < vgSize[1]; ++j)
			{
				const auto& idx = vg->getGridIndex(i, j, k);
				const auto& worldCoord = vg->local2World(i, j, k);
				const ugr::gridmap::Position pos(worldCoord[0], worldCoord[1]);
				const ugr::gridmap::Index& rmIdx = rm.world2Local(pos);
				if (rm.isInBounds(rmIdx))
				{
					auto& brisk = rm.at("Ballistic Strike Risk", rmIdx);
					if (isnan(brisk))
						brisk = 0;
					const auto& grisk = rm.at("Glide Strike Risk", rmIdx);
					// std::cout << "B: " << brisk << " G: " << grisk << "\n";
#pragma omp critical
					vg->groundRiskVals[idx] = brisk + grisk;
				}
			}
		}
	}

	// for (int i = 0; i < vgSize[0]; ++i)
	// {
	// 	for (int j = 0; j < vgSize[1]; ++j)
	// 	{
	// 		for (int k = 0; k < vgSize[2]; ++k)
	// 		{
	// 			const auto& idx = vg->getGridIndex(i, j, k);
	// 			const auto& worldCoord = vg->local2World(i, j, k);
	// 			const ugr::gridmap::Position pos(worldCoord[0], worldCoord[1]);;
	// 			const auto& rm = altRiskMaps.at(worldCoord[2]);
	// 			const auto& brisk = rm.atPosition("Ballistic Strike Risk", pos);
	// 			const auto& grisk = rm.atPosition("Glide Strike Risk", pos);
	// 			vg->groundRiskVals[idx] = brisk + grisk;
	// 		}
	// 	}
	// }

	std::cout << "All Finished!\n";

	const auto& mar = vg->getMaxAirRisk();
	std::cout << "Max Air Risk: " << mar << "\n";
	const auto& mgr = vg->getMaxGroundRisk();
	std::cout << "Max Ground Risk: " << mgr << "\n";
	const auto& aar = std::accumulate(vg->airRiskVals.begin(), vg->airRiskVals.end(), 0.0) / vg->airRiskVals.size();
	std::cout << "Mean Air Risk: " << aar << "\n";
	const auto& agr = std::accumulate(vg->groundRiskVals.begin(), vg->groundRiskVals.end(), 0.0) / vg->groundRiskVals.
		size();
	std::cout << "Mean Ground Risk: " << agr << "\n";

	vg->writeToNetCDF("CombinedVoxelGrid.nc");

	std::cout << "Written to netCDF...\n";
}
