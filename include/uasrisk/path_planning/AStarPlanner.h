#ifndef UR_THETASTARPLANNER_H
#define UR_THETASTARPLANNER_H
#include "PathPlanner.h"

namespace ur
{
	namespace path
	{
		class AStarPlanner final : public PathPlanner
		{
		public:
			std::vector<ur::Index> planPath(const ur::Index& start, const ur::Index& end, ur::VoxelGrid& grid,
			                                std::function<double(const Index&,
			                                                     const VoxelGrid&)> cellCostFunction) override;
			using PathPlanner::planPath;
		protected:
			double g(const Index& p1, const Index& p2, const VoxelGrid& grid) override;
			double h(const Index& p1, const Index& p2, const VoxelGrid& grid) override;
		};
	}
}
#endif // UR_THETASTARPLANNER_H
