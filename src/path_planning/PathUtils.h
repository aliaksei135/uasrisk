#ifndef UR_PATHUTILS_H
#define UR_PATHUTILS_H

#include <vector>

namespace ur
{
	namespace path
	{
		static std::vector<Position> pathLocal2World(const std::vector<Index>& indices, const VoxelGrid& grid)
		{
			std::vector<Position> out;
			out.reserve(indices.size());
			std::transform(indices.begin(), indices.end(), out.begin(), [grid](const Index& idx)
			{
				return grid.local2World(idx);
			});
			return out;
		}

		static std::vector<Index> pathWorld2Local(const std::vector<Position>& positions, const VoxelGrid& grid)
		{
			std::vector<Index> out;
			out.reserve(positions.size());
			std::transform(positions.begin(), positions.end(), out.begin(), [grid](const Position& pos)
			{
				return grid.world2Local(pos);
			});
			return out;
		}
	}
}
#endif // UR_PATHUTILS_H
