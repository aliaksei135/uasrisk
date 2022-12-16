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
	const auto& size = vg.getSize();

	const ur::Position pos1{ -1.4500237f, 50.9065510f, 50 };
	const auto& idx1 = vg.world2Local(pos1);
	const auto pos1t = vg.local2World(idx1);
	ASSERT_NEAR(pos1(0), pos1t(0), 0.001);
	ASSERT_NEAR(pos1(1), pos1t(1), 0.001);
	ASSERT_NEAR(pos1(1), pos1t(1), zRes + 1);

	const ur::Index idx2{ 0, 0, 0 };
	const auto& pos2 = vg.local2World(idx2);
	const auto& idx2t = vg.world2Local(pos2);
	ASSERT_NEAR(idx2(0), idx2t(0), 1.1);
	ASSERT_NEAR(idx2(1), idx2t(1), 1.1);
	ASSERT_NEAR(idx2(2), idx2t(2), 1.1);

	const ur::Index idx3{ size[0] - 1, size[1] - 1, size[2] - 1 };
	const auto& pos3 = vg.local2World(idx3);
	const auto& idx3t = vg.world2Local(pos3);
	ASSERT_NEAR(idx3(0), idx3t(0), 1.1);
	ASSERT_NEAR(idx3(1), idx3t(1), 1.1);
	ASSERT_NEAR(idx3(2), idx3t(2), 1.1);

	const ur::Index idx4{ size[0] - 1, 0, 0 };
	const auto& pos4 = vg.local2World(idx4);
	const auto& idx4t = vg.world2Local(pos4);
	ASSERT_NEAR(idx4(0), idx4t(0), 1.1);
	ASSERT_NEAR(idx4(1), idx4t(1), 1.1);
	ASSERT_NEAR(idx4(2), idx4t(2), 1.1);

	const ur::Index idx5{ 0, size[1] - 1, 0 };
	const auto& pos5 = vg.local2World(idx5);
	const auto& idx5t = vg.world2Local(pos5);
	ASSERT_NEAR(idx5(0), idx5t(0), 1.1);
	ASSERT_NEAR(idx5(1), idx5t(1), 1.1);
	ASSERT_NEAR(idx5(2), idx5t(2), 1.1);

	const ur::Position pos6{ -1.450f, 50.951f, 240 };
	const auto& idx6 = vg.world2Local(pos6);
	const auto& pos6t = vg.local2World(idx6);
	ASSERT_NEAR(pos6(0), pos6t(0), 0.001);
	ASSERT_NEAR(pos6(1), pos6t(1), 0.001);
	ASSERT_NEAR(pos6(2), pos6t(2), zRes + 1);

	const ur::Position pos7{ -1.340f, 50.951f, 50 };
	const auto& idx7 = vg.world2Local(pos7);
	const auto& pos7t = vg.local2World(idx7);
	ASSERT_NEAR(pos7(0), pos7t(0), 0.001);
	ASSERT_NEAR(pos7(1), pos7t(1), 0.001);
	ASSERT_NEAR(pos7(2), pos7t(2), zRes + 1);

	const ur::Position pos8{ -1.340f, 50.906f, 140 };
	const auto& idx8 = vg.world2Local(pos8);
	const auto& pos8t = vg.local2World(idx8);
	ASSERT_NEAR(pos8(0), pos8t(0), 0.001);
	ASSERT_NEAR(pos8(1), pos8t(1), 0.001);
	ASSERT_NEAR(pos8(2), pos8t(2), zRes + 1);
}

TEST(VoxelGridMapTests, IndexingTest)
{
	constexpr int xyRes = 60;
	constexpr int zRes = 20;
	std::array<float, 4> xyBounds{
		50.703057f, -1.973112f, 50.820251f,
		-1.767941f
	};
	const std::array<float, 6> xyzBounds{
		xyBounds[0], xyBounds[1], 50, xyBounds[2],
		xyBounds[3], 250
	};

	ur::VoxelGrid vg(xyzBounds, xyRes, zRes);
	const auto& size = vg.getSize();

	ur::Matrix testGrid(size.x(), size.y(), size.z());
	testGrid.setConstant(0);
	for (int i = 0; i < size.x(); ++i)
	{
		for (int j = 0; j < size.y(); ++j)
		{
			for (int k = 0; k < size.z(); ++k)
			{
				testGrid(i, j, k) = static_cast<ur::Matrix::Scalar>(i + j + k);
			}
		}
	}

	vg.add("test", testGrid);

	const ur::Position pos1{ -1.973112f, 50.703057f, 50 };
	const auto& val1 = vg.atPosition("test", pos1);
	ASSERT_EQ(val1, 0);

	const ur::Index idx2{ 20, 20, 2 };
	const auto& val2 = vg.at("test", idx2);
	ASSERT_EQ(val2, 42);

	const ur::Position pos3{ -1.8705265f, 50.7616505f, 90 };
	const auto& val3 = vg.atPosition("test", pos3);
	ASSERT_NEAR(val3, (size[0] / 2) + (size[1] / 2) + 2, 1.1);

	const ur::Position pos4{ -1.7687f, 50.8198f, 240 };
	const auto& val4 = vg.atPosition("test", pos4);
	const auto& idx4 = vg.world2Local(pos4);
	ASSERT_NEAR(val4, size[0] + size[1] + 9, 2);

	const ur::Position pos5{ -1.973112f, 50.8198f, 50 };
	const auto& val5 = vg.atPosition("test", pos5);
	ASSERT_NEAR(val5, size[1] - 1, 1);

	const ur::Position pos6{ -1.7689f, 50.703057f, 130 };
	const auto& val6 = vg.atPosition("test", pos6);
	ASSERT_NEAR(val6, size[0] + 3, 1);
}