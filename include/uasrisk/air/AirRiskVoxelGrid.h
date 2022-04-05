#ifndef UR_AIRRISKVOXELGRID_H
#define UR_AIRRISKVOXELGRID_H
#include "uasrisk/environment/VoxelGrid.h"

namespace ur
{
	class AirRiskVoxelGrid : public VoxelGrid
	{
	public:
		AirRiskVoxelGrid(const std::array<FPScalar, 6>& bounds, const FPScalar xyRes, const FPScalar zRes,
		                 std::string trajPath, std::string airspacePath);

		void eval();

	protected:
		std::string trajPath;
		std::string airspacePath;
	};
}
#endif // UR_AIRRISKVOXELGRID_H
