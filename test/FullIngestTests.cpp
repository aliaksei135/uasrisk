#include "../src/VoxelGrid.h"
#include "../src/VoxelGridBuilder.h"
#include "../src/opensky/OpenSkyCsvReader.h"
#include "../src/opensky/OpenSkyReader.h"
#include "../src/openaip/OpenAIPReader.h"
#include <gtest/gtest.h>

#include <iostream>

TEST(FullIngestTests, IngestTest)
{
	auto* vg = new ur::VoxelGrid(300, 300, 18, 250, 61, {-1.718725, 50.717606, 0});
	ur::VoxelGridBuilder vgb(vg);

	const OpenSkyCsvReader osReader;
	const auto trajs = osReader.readFile("opensky_states.csv");

	const OpenAIPReader oaReader;
	const auto airspaces = oaReader.readFile("openaip_airspace_uk.aip");

	vgb.handleTrajectory(trajs);
	vgb.handleBlockingPolygon(airspaces);

	const auto& m = vg->getMaxAirRisk();
	std::cout << "Max: " << m << "\n";
	vg->writeToNetCDF("test.nc");
}
