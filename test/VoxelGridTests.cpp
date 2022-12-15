#include <gtest/gtest.h>
#include "uasrisk/environment/VoxelGrid.h"

TEST(VoxelGridTests, ProjectionTests)
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

	ur::VoxelGrid vg(xyzBounds, xyRes, zRes);

	const ur::Position pos1{ -1.4500237f, 50.9065510f, 100 };
	const auto& idx1 = vg.world2Local(pos1);
	const auto pos1t = vg.local2World(idx1);

	ASSERT_NEAR(pos1(0), pos1t(0), 0.001);
	ASSERT_NEAR(pos1(1), pos1t(1), 0.001);
	ASSERT_NEAR(pos1(1), pos1t(1), zRes + 1);

	const ur::Index idx2{ 20, 20, 2 };
	const auto& pos2 = vg.local2World(idx2);
	const auto& idx2t = vg.world2Local(pos2);

	ASSERT_NEAR(idx2(0), idx2t(0), 1.1);
	ASSERT_NEAR(idx2(1), idx2t(1), 1.1);
	ASSERT_NEAR(idx2(2), idx2t(2), 1.1);

	int i = 4;
}