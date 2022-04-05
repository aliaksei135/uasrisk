#include <gtest/gtest.h>
#include "uasrisk/air/AirRiskVoxelGrid.h"

TEST(AirRiskVoxelGridTests, EvalTest)
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

	ur::AirRiskVoxelGrid arvg(xyzBounds, xyRes, zRes, "opensky_states.csv", "openaip_airspace_uk.aip");
	arvg.eval();
}
