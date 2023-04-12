#include <gtest/gtest.h>

#include "uasgroundrisk/map_gen/TemporalPopulationMap.h"
#include "uasrisk/RiskVoxelGrid.h"

TEST(RiskVoxelGridTests, EvalTest)
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

	ur::RiskVoxelGrid rvg(xyzBounds, xyRes, zRes, "opensky_states.csv", "openaip_airspace_uk.aip", &population,
	                      &aircraft, &obstacleMap, &weather);
	rvg.eval();
}
