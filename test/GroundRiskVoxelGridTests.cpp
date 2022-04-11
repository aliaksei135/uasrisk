#include <gtest/gtest.h>

#include "uasgroundrisk/map_gen/TemporalPopulationMap.h"
#include "uasrisk/ground/GroundRiskVoxelGrid.h"

TEST(GroundRiskVoxelGridTests, EvalTest)
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
	aircraft.mass = 50;
	aircraft.length = 5;
	aircraft.width = 5;
	aircraft.state.position << 0, 0, 0;
	aircraft.state.velocity << 20, 20, 1;

	aircraft.addDescentModel<ugr::risk::GlideDescentModel>(21, 15);
	aircraft.addDescentModel<ugr::risk::BallisticDescentModel>(0.6 * 0.6, 0.8);


	ugr::risk::WeatherMap weather(xyBounds, xyRes);
	weather.addConstantWind(5, 220);
	weather.eval();

	ugr::mapping::TemporalPopulationMap population(xyBounds, xyRes);
	population.setHourOfDay(12);
	population.eval();

	ugr::risk::ObstacleMap obstacleMap(xyBounds, xyRes);
	obstacleMap.addBuildingHeights();
	obstacleMap.eval();

	ur::GroundRiskVoxelGrid grvg(xyzBounds, xyRes, zRes, &population, &aircraft, &obstacleMap, &weather);
	grvg.eval();


}
