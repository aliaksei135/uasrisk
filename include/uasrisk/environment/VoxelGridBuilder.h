#ifndef VOXELGRIDBUILDERVISITOR_H
#define VOXELGRIDBUILDERVISITOR_H

#include <vector>

#include "TypeDefs.h"


namespace ur
{
	class VoxelGrid;

	class VoxelGridBuilder
	{
	public:
		explicit VoxelGridBuilder(VoxelGrid* grid);
		void handleBlockingPolygon(const std::vector<ExtrudedPolygon>& polys) const;
		void handleBlockingPolygon(const ExtrudedPolygon& poly) const;
		void handleTrajectory(const std::vector<LineString>& lss) const;
		void handleTrajectory(const LineString& ls) const;
		void handleVoxelCoord(const ur::Index& coord) const;

		ur::VoxelGrid* grid;
	};
}
#endif // VOXELGRIDBUILDERVISITOR_H
