#include "uasrisk/path_planning/PathPlanner.h"

std::vector<ur::Index> ur::path::PathPlanner::planPath(const ur::Index& start, const ur::Index& end,
                                                       ur::VoxelGrid& grid)
{
	const auto costFunc = [](const Index& idx, const VoxelGrid& g)
	{
		return g.at("Air Risk", idx) + g.at("Ground Risk", idx);
	};
	return planPath(start, end, grid, costFunc);
}

std::vector<ur::Index> ur::path::PathPlanner::assemblePath(const Index& start, const Index& end,
                                                           const std::unordered_map<Index, Index>& backpointerMap)
{
	std::deque<Index> path;
	auto current = end;
	while (!current.cwiseEqual(start).all())
	{
		path.emplace_front(current);
		current = backpointerMap.at(current);
	}
	return {std::make_move_iterator(path.begin()), std::make_move_iterator(path.end())};
}
