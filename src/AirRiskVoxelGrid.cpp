#include "uasrisk/air/AirRiskVoxelGrid.h"

#include "VoxelGridBuilder.h"
#include "uasrisk/air/io/openaip/OpenAIPReader.h"
#include "uasrisk/air/io/opensky/OpenSkyCsvReader.h"

ur::AirRiskVoxelGrid::AirRiskVoxelGrid(const std::array<FPScalar, 6>& bounds, const FPScalar xyRes, const FPScalar zRes,
                                       std::string& trajPath, std::string& airspacePath):
	VoxelGrid(bounds, xyRes, zRes, "EPSG:4326"), trajPath(std::move(trajPath)), airspacePath(std::move(airspacePath))
{
}

void ur::AirRiskVoxelGrid::eval()
{
	const ur::io::OpenSkyCsvReader osReader;
	const auto trajs = osReader.readFile(trajPath);

	const ur::io::OpenAIPReader oaReader;
	const auto airspaces = oaReader.readFile(airspacePath);

	ur::VoxelGridBuilder vgb(this);
	vgb.handleTrajectory(trajs);
	vgb.handleBlockingPolygon(airspaces);
}
