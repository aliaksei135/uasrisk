#include <gtest/gtest.h>
#include "uasgroundrisk/risk_analysis/aircraft/AircraftModel.h"
#include "uasgroundrisk/risk_analysis/weather/WeatherMap.h"
#include "uasgroundrisk/map_gen/TemporalPopulationMap.h"
#include "uasgroundrisk/risk_analysis/obstacles/ObstacleMap.h"
#include "uasrisk/ground/IncrementalGroundRiskVoxelGrid.h"
#include "../python/GroundRiskHelpers.h"

TEST(GroundRiskIncrementalTests, EvalTest)
{
	constexpr int xyRes = 60;
	constexpr int zRes = 20;
	const std::array<float, 4> xyBounds{
		50.9065510f, -1.4500237f, 50.9517765f,
		-1.3419628f
	};
	const std::array<float, 6> xyzBounds{
		xyBounds[0], xyBounds[1], 50, xyBounds[2],
		xyBounds[3], 250
	};

	ugr::risk::AircraftModel aircraft;
	// State set inside sweep loop

	aircraft.mass = 50;
	aircraft.length = 5;
	aircraft.width = 5;
	aircraft.failureProb = 5e-3;
	aircraft.state.position << 0, 0, 0;
	aircraft.state.velocity << 20, 20, 1;

	aircraft.addDescentModel<ugr::risk::GlideDescentModel>(21, 15);
	aircraft.addDescentModel<ugr::risk::BallisticDescentModel>(0.6 * 0.6, 0.8);
	// aircraft.descents.emplace_back(
	//     std::unique_ptr<ugr::risk::ParachuteDescentModel>(
	//         new ugr::risk::ParachuteDescentModel(90, 2.8, 3.2, 1.2, 12.5, 2)));

	ugr::risk::WeatherMap weather(xyBounds, xyRes);
	// weather.addGribs();
	weather.addConstantWind(5, 220);
	// weather.addWeatherAvoidanceZone(badWeatherGridIter.begin(), badWeatherGridIter.end());
	weather.eval();

	ugr::mapping::TemporalPopulationMap population(xyBounds, xyRes);
	population.setHourOfDay(12);
	population.eval();

	ugr::risk::ObstacleMap obstacleMap(xyBounds, xyRes);
	obstacleMap.addBuildingHeights();
	obstacleMap.eval();

	ur::IncrementalGroundRiskVoxelGrid grvg(xyzBounds, xyRes, zRes, &population, &aircraft, &obstacleMap, &weather);
	auto pointStrikeRisk = grvg.getPositionPointStrikeProbability({ -1.4f, 50.925f, 90 }, 270);
	auto pointFatalityRisk = grvg.getPositionPointFatalityProbability({ -1.4f, 50.925f, 90 }, 270);
	// print risks
	std::cout << "Strike risk: " << pointStrikeRisk << std::endl;
	std::cout << "Fatality risk: " << pointFatalityRisk << std::endl;
	assert(pointStrikeRisk > 0.0f);
	assert(pointFatalityRisk > 0.0f);

	PyIncrementalGroundRiskVoxelGrid pygrvg(xyzBounds, xyRes, zRes, &aircraft);
	auto pyPointStrikeRisk = pygrvg.getPositionPointStrikeProbability({ -1.4f, 50.925f, 90 }, 270);
	auto pyPointFatalityRisk = pygrvg.getPositionPointFatalityProbability({ -1.4f, 50.925f, 90 }, 270);
	// print risks
	std::cout << "Strike risk: " << pyPointStrikeRisk << std::endl;
	std::cout << "Fatality risk: " << pyPointFatalityRisk << std::endl;
	assert(pyPointStrikeRisk > 0.0f);
	assert(pyPointFatalityRisk > 0.0f);

}