#ifndef UR_INCREMENTALGROUNDRISKVOXELGRID_H
#define UR_INCREMENTALGROUNDRISKVOXELGRID_H

#include "uasrisk/environment/VoxelGrid.h"
#include "uasgroundrisk/risk_analysis/RiskMap.h"
#include "uasgroundrisk/risk_analysis/IncrementalRiskMap.h"

namespace ur
{
	class IncrementalGroundRiskVoxelGrid : public VoxelGrid, ugr::risk::IncrementalRiskMap
	{
	 public:

		IncrementalGroundRiskVoxelGrid(const std::array<FPScalar, 6>& bounds, FPScalar xyRes,
			FPScalar zRes,
			ugr::mapping::PopulationMap* populationMap,
			ugr::risk::AircraftModel* aircraftModel,
			ugr::risk::ObstacleMap* obstacleMap,
			ugr::risk::WeatherMap* weather);

		~IncrementalGroundRiskVoxelGrid()
		{
//			delete populationMap;
//			delete aircraftModel;
//			delete obstacleMap;
//			delete weatherMap;
		}

		using ugr::risk::IncrementalRiskMap::getIndexPointStrikeProbability;
		using ugr::risk::IncrementalRiskMap::getIndexPointFatalityProbability;
		using ugr::risk::IncrementalRiskMap::getPositionPointStrikeProbability;
		using ugr::risk::IncrementalRiskMap::getPositionPointFatalityProbability;

//	 protected:
//		ugr::risk::IncrementalRiskMap incrementalRiskMap;

	};
}
#endif // UR_INCREMENTALGROUNDRISKVOXELGRID_H
