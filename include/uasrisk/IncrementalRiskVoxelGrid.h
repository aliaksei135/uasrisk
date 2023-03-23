#ifndef UR_RISKVOXELGRID_H
#define UR_RISKVOXELGRID_H
#include <uasrisk/air/AirRiskVoxelGrid.h>
#include <uasrisk/ground/GroundRiskVoxelGrid.h>
#include <uasrisk/environment/VoxelGrid.h>

namespace ur
{
	class IncrementalRiskVoxelGrid : public VoxelGrid
	{
	public:
		IncrementalRiskVoxelGrid(const std::array<FPScalar, 6>& bounds, const FPScalar xyRes, const FPScalar zRes,
		              const std::string& trajPath, const std::string& airspacePath,
		              ugr::mapping::PopulationMap* const populationMap, ugr::risk::AircraftModel* const aircraftModel,
		              ugr::risk::ObstacleMap* const obstacleMap, ugr::risk::WeatherMap* const weather);

		void eval();

	protected:
		AirRiskVoxelGrid arvg;
		GroundRiskVoxelGrid grvg;
	};
}
#endif // UR_RISKVOXELGRID_H
