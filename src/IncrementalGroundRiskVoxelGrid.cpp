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
	incrementalRiskMap(*populationMap, *aircraftModel, *obstacleMap, *weather)
{
}

double ur::IncrementalGroundRiskVoxelGrid::getPointRisk(const ugr::gridmap::Position3& pos,
	int heading,
	ugr::risk::RiskType riskType = ugr::risk::RiskType::FATALITY)
{
	if (riskType == ugr::risk::RiskType::FATALITY)
	{
		return incrementalRiskMap.getPointFatalityProbability(pos, heading);
	}
	else if (riskType == ugr::risk::RiskType::STRIKE)
	{
		return incrementalRiskMap.getPointStrikeProbability(pos, heading);
	}
	else
	{
		throw std::runtime_error("Invalid risk type");
	}
}
