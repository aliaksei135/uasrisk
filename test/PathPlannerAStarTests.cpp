#include <gtest/gtest.h>

#include "uasrisk/air/AirRiskVoxelGrid.h"
#include "uasrisk/path_planning/AStarPlanner.h"

TEST(PathPlannerAStarTests, EmptyGridTest)
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
	// arvg.eval();

	arvg.add("Air Risk");
	arvg.get("Air Risk").setConstant(1e-8);
	arvg.add("Ground Risk");
	arvg.get("Ground Risk").setConstant(1e-6);

	const ur::Index start{10, 10, 1};
	const ur::Index end{100, 120, 4};

	ur::path::AStarPlanner planner;
	const auto path = planner.planPath(start, end, arvg);
}
