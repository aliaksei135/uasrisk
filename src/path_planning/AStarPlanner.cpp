#include "uasrisk/path_planning/AStarPlanner.h"
#include <vector>
#include <algorithm>
#include <unordered_map>


std::vector<ur::Index> ur::path::AStarPlanner::planPath(const ur::Index& start, const ur::Index& end,
                                                        ur::VoxelGrid& grid,
                                                        std::function<double(const Index&, const VoxelGrid&)>
                                                        cellCostFunction)
{
	bool completed = false;
	std::vector<std::pair<double, Index>> open;
	std::unordered_map<Index, Index> parents;

	const std::string closedName = "path_closed";
	const std::string fName = "path_f";
	const std::string gName = "path_g";
	const std::string hName = "path_h";

	// Use the grid itself to keep track of visited nodes
	// 0 is unvisited, anything else +ve is visited
	grid.add(closedName, 0);

	// Add other cost layers
	grid.add(fName, 0);
	grid.add(gName, 0);
	grid.add(hName, 0);

	//TODO calc cost for start to end node and assign to end
	open.insert(std::lower_bound(open.cbegin(), open.cend(), 0, idxPairCmp), std::make_pair(0, start));
	// open.emplace(std::make_pair(0, start));

	while (!open.empty())
	{
		// Get next node
		Index current = open.back().second;
		open.pop_back();

		const auto currentG = grid.at(gName, current);

		// Add to closed list and iterate visited count
		++grid.at(closedName, current);
		// If this is the end node then break out the explore loop
		if (current.cwiseEqual(end).all()) break;

		for (const auto& direction : directions)
		{
			Index successor = current + direction;
			// Check suitability of successor node
			// Should be in grid bounds, not in blocked area and unvisited
			if (grid.isInBounds(successor) && grid.at(closedName, successor) == 0 && grid.at("Blocked", successor) < 1)
			{
				// calc successor cost
				const auto successorG = currentG + cellCostFunction(successor, grid);
				const auto successorH = h(successor, end, grid);
				const auto successorF = successorG + successorH;
				auto inOpen = std::find_if(open.begin(), open.end(), [successor](const std::pair<double, Index>& pair)
				{
					return pair.second.cwiseEqual(successor).all();
				});
				if (inOpen == open.end())
				{
					// Not in open already, so add it
					open.insert(std::lower_bound(open.cbegin(), open.cend(), successorF, idxPairCmp),
					            std::make_pair(successorF, successor));
					// Update backpointer
					parents[successor] = current;

					// Update costs
					grid.at(fName, successor) = successorF;
					grid.at(gName, successor) = successorG;
					grid.at(hName, successor) = successorH;
				}
				else
				{
					// Already in open, check if the cost is better now and if so update it
					if (grid.at(fName, successor) > successorF)
					{
						// Reinsert new 
						open.erase(inOpen);
						open.insert(std::lower_bound(open.cbegin(), open.cend(), successorF, idxPairCmp),
						            std::make_pair(successorF, successor));
						// Update backpointer
						parents[successor] = current;

						grid.at(fName, successor) = successorF;
						grid.at(gName, successor) = successorG;
						grid.at(hName, successor) = successorH;
					}
				}
			}
		}
	}
	return assemblePath(start, end, parents);
}

double ur::path::AStarPlanner::g(const Index& p1, const Index& p2, const VoxelGrid& grid)
{
	throw std::logic_error("A Star g not implemented");
}

double ur::path::AStarPlanner::h(const Index& p1, const Index& p2, const VoxelGrid& grid)
{
	const auto resolution = 30;
	return std::sqrt((p1 - p2).matrix().squaredNorm()) * resolution;
}
