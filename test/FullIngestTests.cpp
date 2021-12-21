#include "../src/VoxelGrid.h"
#include "../src/VoxelGridBuilder.h"
#include "../src/opensky/OpenSkyCsvReader.h"
#include "../src/opensky/OpenSkyReader.h"
#include "../src/openaip/OpenAIPReader.h"
#include <gtest/gtest.h>

#include <iostream>

TEST(FullIngestTests, IngestTest)
{
	const int resolution = 100;
	const std::array<float, 6> xyzBounds{
		50.9065510f, -1.4500237f, 1, 50.9517765f,
		-1.3419628f, 250
	};

	auto* vg = new ur::VoxelGrid(xyzBounds, resolution, 61);
	ur::VoxelGridBuilder vgb(vg);

	const OpenSkyCsvReader osReader;
	const auto trajs = osReader.readFile("opensky_states.csv");

	const OpenAIPReader oaReader;
	const auto airspaces = oaReader.readFile("openaip_airspace_uk.aip");

	vgb.handleTrajectory(trajs);
	vgb.handleBlockingPolygon(airspaces);

	vg->writeToNetCDF("test.nc");
}
