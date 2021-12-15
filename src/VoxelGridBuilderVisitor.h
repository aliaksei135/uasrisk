#ifndef VOXELGRIDBUILDERVISITOR_H
#define VOXELGRIDBUILDERVISITOR_H

#include "TypeDefs.h"

class VoxelGrid;

class VoxelGridBuilderVisitor
{
public:
	explicit VoxelGridBuilderVisitor(VoxelGrid* grid);
	// void handleGeometry(osgEarth::Geometry* geom) const;
	void handleBlockingPolygon(const ur::Polygon& poly, double height) const;
	// void handleBlockingGeometry(osgEarth::Geometry* geom, const double height) const;
	void handleTrajectory(const ur::LineString& ls) const;
	// void handlePoint(const osgEarth::Point& p) const;
	void handleVoxelCoord(const Eigen::Vector3i& coord) const;

	VoxelGrid* grid;
};
#endif // VOXELGRIDBUILDERVISITOR_H
