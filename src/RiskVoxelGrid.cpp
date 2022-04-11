#include "uasrisk/RiskVoxelGrid.h"

ur::RiskVoxelGrid::RiskVoxelGrid(const std::array<FPScalar, 6>& bounds, const FPScalar xyRes, const FPScalar zRes,
	const std::string& trajPath, const std::string& airspacePath, ugr::mapping::PopulationMap* const populationMap,
	ugr::risk::AircraftModel* const aircraftModel, ugr::risk::ObstacleMap* const obstacleMap,
	ugr::risk::WeatherMap* const weather):
	VoxelGrid(bounds, xyRes, zRes), arvg(bounds, xyRes, zRes, trajPath, airspacePath),
	grvg(bounds, xyRes, zRes, populationMap, aircraftModel, obstacleMap, weather)
{
}

void ur::RiskVoxelGrid::eval()
{
	arvg.eval();
	grvg.eval();

	add("Ground Risk", 0);
	add("Air Risk", 0);

	get("Ground Risk") = grvg.get("Ground Risk");
	get("Air Risk") = arvg.get("Air Risk");
}
