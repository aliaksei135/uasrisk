#include "uasrisk/ground/GroundRiskVoxelGrid.h"
#include "uasrisk/ground/IncrementalGroundRiskVoxelGrid.h"
#include "uasgroundrisk/risk_analysis/IncrementalRiskMap.h"

#include <cassert>

ur::IncrementalGroundRiskVoxelGrid::IncrementalGroundRiskVoxelGrid(const array<FPScalar, 6>& bounds,
	ur::FPScalar xyRes,
	ur::FPScalar zRes,
	ugr::mapping::PopulationMap* populationMap,
	ugr::risk::AircraftModel* aircraftModel,
	ugr::risk::ObstacleMap* obstacleMap,
	ugr::risk::WeatherMap* weather) :
	VoxelGrid(bounds, xyRes, zRes),
	ugr::risk::IncrementalRiskMap(*populationMap, *aircraftModel, *obstacleMap, *weather)
{
}
