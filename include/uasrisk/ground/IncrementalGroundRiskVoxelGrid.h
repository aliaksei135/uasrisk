#ifndef UR_INCREMENTALGROUNDRISKVOXELGRID_H
#define UR_INCREMENTALGROUNDRISKVOXELGRID_H

#include "uasrisk/environment/VoxelGrid.h"
#include "uasgroundrisk/risk_analysis/RiskMap.h"
#include "uasgroundrisk/risk_analysis/IncrementalRiskMap.h"

namespace ur
{
	class IncrementalGroundRiskVoxelGrid : public VoxelGrid
	{
	 public:

		IncrementalGroundRiskVoxelGrid(const std::array<FPScalar, 6>& bounds, FPScalar xyRes,
			FPScalar zRes,
			ugr::mapping::PopulationMap* populationMap,
			ugr::risk::AircraftModel* aircraftModel,
			ugr::risk::ObstacleMap* obstacleMap,
			ugr::risk::WeatherMap* weather) : VoxelGrid(bounds, xyRes, zRes),
											  incrementalRiskMap(*populationMap, *aircraftModel, *obstacleMap, *weather)
		{
		}

		~IncrementalGroundRiskVoxelGrid()
		{
//			delete populationMap;
//			delete aircraftModel;
//			delete obstacleMap;
//			delete weatherMap;
		}

		// Pybind needs a bunch of lambda functions as it does not support reflection, so would need even deeper bindings into ugr
		// No beauty contests here

		double getIndexPointStrikeProbability(const ugr::gridmap::Index& index,
			const double altitude,
			const int heading)
		{
			return incrementalRiskMap.getIndexPointStrikeProbability(index, altitude, heading);
		}

		double getIndexPointFatalityProbability(const ugr::gridmap::Index& index,
			const double altitude,
			const int heading)
		{
			return incrementalRiskMap.getIndexPointFatalityProbability(index, altitude, heading);
		}

		double getPositionPointStrikeProbability(const ugr::gridmap::Position3& position,
			const int heading)
		{
			return incrementalRiskMap.getPositionPointStrikeProbability(position, heading);
		}

		double getPositionPointFatalityProbability(const ugr::gridmap::Position3& position,
			const int heading)
		{
			return incrementalRiskMap.getPositionPointFatalityProbability(position, heading);
		}

	 protected:
		ugr::risk::IncrementalRiskMap incrementalRiskMap;

	};
}
#endif // UR_INCREMENTALGROUNDRISKVOXELGRID_H
