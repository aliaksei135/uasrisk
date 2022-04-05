
#include <gtest/gtest.h>

#include <iostream>

#include "uasrisk/air/io/openaip/OpenAIPReader.h"
#include "uasrisk/air/io/opensky/OpenSkyCsvReader.h"
#include "uasrisk/environment/VoxelGrid.h"
#include "../src/VoxelGridBuilder.h"

TEST(FullIngestTests, IngestTest)
{
	const int resolution = 250;
	const std::array<float, 6> xyzBounds{
		50.717606f, -1.718725f, 0, 51.0f,
		-1.1f, 250
	};

	auto* vg = new ur::VoxelGrid(xyzBounds, resolution, 61);
	ur::VoxelGridBuilder vgb(vg);

	const ur::io::OpenSkyCsvReader osReader;
	const auto trajs = osReader.readFile("opensky_states.csv");

	const ur::io::OpenAIPReader oaReader;
	const auto airspaces = oaReader.readFile("openaip_airspace_uk.aip");

	vgb.handleTrajectory(trajs);
	vgb.handleBlockingPolygon(airspaces);

	vg->writeToNetCDF("test.nc");
}
