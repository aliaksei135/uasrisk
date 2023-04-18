#ifndef UR_GROUNDRISKVOXELGRID_H
#define UR_GROUNDRISKVOXELGRID_H

#include "uasrisk/environment/VoxelGrid.h"
#include "uasgroundrisk/risk_analysis/RiskMap.h"

namespace ur
{
	class GroundRiskVoxelGrid : public VoxelGrid
	{
	public:

		GroundRiskVoxelGrid(const std::array<FPScalar, 6>& bounds, FPScalar xyRes,
		                    FPScalar zRes,
		                    ugr::mapping::PopulationMap* populationMap,
		                    ugr::risk::AircraftModel* aircraftModel,
		                    ugr::risk::ObstacleMap* obstacleMap,
		                    ugr::risk::WeatherMap* weather);

		~GroundRiskVoxelGrid()
		{
//			delete populationMap;
//			delete aircraftModel;
//			delete obstacleMap;
//			delete weatherMap;
		}

		void eval();
	protected:
		std::map<double, ugr::risk::RiskMap> altitudeRiskMap;
		ugr::mapping::PopulationMap* populationMap;
		ugr::risk::AircraftModel* aircraftModel;
		ugr::risk::ObstacleMap* obstacleMap;
		ugr::risk::WeatherMap* weatherMap;
	};
}
#endif // UR_GROUNDRISKVOXELGRID_H
