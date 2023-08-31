#include <fstream>

#include <gtest/gtest.h>

#include "uasrisk/environment/VoxelGrid.h"

TEST(EmptyGridTest1, EmptyGridTests)
{
	const std::array<float, 6> bounds{
		50.9065510f, -1.4500237f, 1, 50.9517765f,
		-1.3419628f, 250
	};

	ur::VoxelGrid vg(bounds, 500, 60);
	const auto size = vg.getSize();
	const int sx = size[0], sy = size[1], sz = size[2];


	ur::Matrix& grm = vg.get("Ground Risk");
	for (int z = 0; z < sz; ++z)
	{
		const auto offset = 10 * z;
		grm(0, 0, z) = 1 + offset;
		grm(sx / 2, sy / 2, z) = 2 + offset;
		grm(sx / 2, 0, z) = 3 + offset;
		grm(0, sy / 2, z) = 4 + offset;
		grm(sx - 1, sy - 1, z) = 9 + offset;
	}


//	vg.writeToNetCDF("empty_test.nc");
}
