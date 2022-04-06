#ifndef UR_PATHPLANNER_H
#define UR_PATHPLANNER_H
#include <uasrisk/environment/VoxelGrid.h>
#include <unordered_map>
#include <boost/container_hash/hash.hpp>
#include <boost/container_hash/extensions.hpp>

template <>
struct std::hash<ur::Index>
{
	std::size_t operator()(ur::Index const& idx) const noexcept
	{
		// std::size_t hash;
		// boost::hash_combine
		return boost::hash_range(idx.begin(), idx.end());
	}
};

template <>
struct std::equal_to<ur::Index>
{
	bool operator()(const ur::Index& lhs, const ur::Index& rhs) const noexcept
	{
		return lhs.cwiseEqual(rhs).all();
	}
};


inline bool idxPairCmp(const std::pair<double, ur::Index>& pair, const double& val)
{
	return val < pair.first;
}


namespace ur
{
	namespace path
	{
		class PathPlanner
		{
		public:
			virtual ~PathPlanner() = default;
			virtual std::vector<ur::Index> planPath(const ur::Index& start, const ur::Index& end, ur::VoxelGrid& grid,
			                                        std::function<double(const Index&, const VoxelGrid&)>
			                                        cellCostFunction) = 0;
			std::vector<ur::Index> planPath(const ur::Index& start, const ur::Index& end, ur::VoxelGrid& grid);
		protected:
			virtual double g(const Index& p1, const Index& p2, const VoxelGrid& grid) = 0;
			virtual double h(const Index& p1, const Index& p2, const VoxelGrid& grid) = 0;

			std::vector<Index> assemblePath(const Index& start, const Index& end,
			                                const std::unordered_map<Index, Index>& backpointerMap);

			std::vector<Index> directions = {
				{0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}, {1, -1, 0}, {-1, 1, 0},
				{-1, -1, 0}, {1, 1, 0}, {-1, 0, -1}, {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {0, -1, 1}, {0, 1, 1},
				{0, 1, -1}, {0, -1, -1}, {-1, -1, 1}, {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, -1}, {1, 1, -1},
				{-1, 1, -1}, {1, -1, -1},
			};
		};
	}
}
#endif // UR_PATHPLANNER_H
