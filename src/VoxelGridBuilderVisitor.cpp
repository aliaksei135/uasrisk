#include "VoxelGridBuilderVisitor.h"

#include "VoxelGrid.h"
#include <numeric>

#include "Bresenham3D.h"
#include <Eigen/Dense>

#include <boost/geometry.hpp>


#define AIR_RISK_COUNTS // Only use the cell intersection counts instead of calculating the length of cell intersections

using namespace boost::geometry;


// VoxelGridBuilderVisitor::VoxelGridBuilderVisitor(VoxelGrid* grid) : grid(grid)
// {
// 	setTraversalMode(TRAVERSE_ALL_CHILDREN);
// }
//
// void VoxelGridBuilderVisitor::apply(osg::Geometry& geometry)
// {
// 	auto* geom = dynamic_cast<osgEarth::Geometry*>(&geometry);
// 	if (geom)
// 	{
// 		handleGeometry(geom);
// 	}
// 	traverse(geometry);
// }
//
// void VoxelGridBuilderVisitor::apply(Node& node)
// {
// 	bool openskyChild = false, airspaceChild = false;
// 	for (const auto& parents : node.getParentalNodePaths())
// 	{
// 		for (const auto& parent : parents)
// 		{
// 			if (parent->getName() == "OpenSky")
// 			{
// 				openskyChild = true;
// 				goto found_valid_parent;
// 			}
// 			if (parent->getName() == "Airspace")
// 			{
// 				airspaceChild = true;
// 				goto found_valid_parent;
// 			}
// 		}
// 	}
//
// found_valid_parent:
// 	if (openskyChild)
// 	{
// 		auto* fnode = dynamic_cast<FeatureNode*>(&node);
// 		if (fnode)
// 		{
// 			const auto& flist = fnode->getFeatures();
// 			for (const auto& feature : flist)
// 			{
// 				handleGeometry(feature->getGeometry());
// 			}
// 			// Mask nodes that are handled
// 			fnode->setNodeMask(0x0);
// 		}
// 		else
// 		{
// 			traverse(node);
// 		}
// 	}
// 	else if (airspaceChild)
// 	{
// 		auto* fnode = dynamic_cast<FeatureNode*>(&node);
// 		if (fnode)
// 		{
// 			const auto& flist = fnode->getFeatures();
// 			for (const auto& feature : flist)
// 			{
// 				handleBlockingGeometry(feature->getGeometry(),
// 				                       feature->style()->getOrCreateSymbol<ExtrusionSymbol>()->height().get());
// 			}
// 			// Mask nodes that are handled
// 			fnode->setNodeMask(0x0);
// 		}
// 		else
// 		{
// 			traverse(node);
// 		}
// 	}
// 	else
// 	{
// 		traverse(node);
// 	}
// }
//
// void VoxelGridBuilderVisitor::handleGeometry(osgEarth::Geometry* geom) const
// {
// 	switch (geom->getType())
// 	{
// 	case osgEarth::Geometry::TYPE_MULTI:
// 		for (const auto& g : static_cast<MultiGeometry*>(geom)->getComponents())
// 		{
// 			handleGeometry(g.get());
// 		}
// 		break;
// 	case osgEarth::Geometry::TYPE_LINESTRING:
// 		handleTrajectory(static_cast<const LineString&>(*geom));
// 		break;
// 	case osgEarth::Geometry::TYPE_POINT:
// 		handlePoint(static_cast<const Point&>(*geom));
// 		break;
// 	default:
// 		;
// 	}
// }
//
// void VoxelGridBuilderVisitor::handleBlockingGeometry(osgEarth::Geometry* geom, const double height) const
// {
// 	switch (geom->getType())
// 	{
// 	case osgEarth::Geometry::TYPE_MULTI:
// 		for (auto g : static_cast<MultiGeometry*>(geom)->getComponents())
// 		{
// 			handleBlockingGeometry(g.get(), height);
// 		}
// 		break;
// 	case osgEarth::Geometry::TYPE_POLYGON:
// 		handleBlockingPolygon(static_cast<const Polygon*>(geom), height);
// 		break;
// 	default:
// 		;
// 	}
// }

void VoxelGridBuilderVisitor::handleBlockingPolygon(const ur::Polygon& poly, const double height) const
{
	ur::Voxel boundsBox;
	envelope(poly, boundsBox);

	const Eigen::Vector3d minCoord(boundsBox.min_corner().x(), boundsBox.min_corner().y(), boundsBox.min_corner().z());
	const Eigen::Vector3d maxCoord(boundsBox.max_corner().x(), boundsBox.max_corner().y(), boundsBox.max_corner().z());
	const auto& gridMinCoord = grid->world2Local(minCoord);
	const auto& gridMaxCoord = grid->world2Local(maxCoord);

	// #pragma omp parallel for collapse(3), shared(bp)
	for (int x = gridMinCoord.x() - 1; x < gridMaxCoord.x() + 1; ++x)
	{
		for (int y = gridMinCoord.y() - 1; y < gridMaxCoord.y() + 1; ++y)
		{
			for (int z = gridMinCoord.z() - 1; z < gridMaxCoord.z() + 1; ++z)
			{
				if (grid->isInBounds({x, y, z}))
				{
					const auto& worldVoxelCoord = grid->local2World(x, y, z);
					// const auto& within2D = poly.contains2D(worldVoxelCoord.x(), worldVoxelCoord.y());
					const auto within2D = within(poly, worldVoxelCoord);
					const auto& withinZ = worldVoxelCoord.z() >= boundsBox.min_corner().z() && worldVoxelCoord.z() <=
						boundsBox.max_corner().z() + height;

					if (within2D && withinZ)
					{
						const auto& idx = grid->getGridIndex(x, y, z);
						// #pragma omp critical
						grid->blockedVals[idx] = true;
					}
				}
			}
		}
	}
}


void VoxelGridBuilderVisitor::handleTrajectory(const ur::LineString& ls) const
{
#ifdef AIR_RISK_COUNTS
	// Iterate through all points in the linestring and rasterise between them
	const auto lsSize = ls.size();
#pragma omp parallel for shared(ls), if(lsSize > 2)
	for (int i = 0; i < lsSize - 1; ++i)
	{
		// Narrow down the possible voxels intersected by passing through bresenham algo
		// This requires projection to local grid coords as bresenham is integer based
		const auto& prevProjP = grid->world2Local(ls[i]);
		const auto& projP = grid->world2Local(ls[i + 1]);
		auto points = Bresenham3D::line3d(prevProjP, projP);
		for (const auto& c : points)
		{
#pragma omp critical
			handleVoxelCoord(c);
		}
	}

#else

	// Make sure the linestring is actually a line
	if (ls.size() < 2) return;

	// osgEarth::LineString outLs;
	// outLs.reserve(ls.size());
	// boost::geometry::simplify(ls, outLs, 0.001);
	// std::cout << "Simplified Linestring by " << (ls.size() - outLs.size()) << "points\n";

	// Reproject the linestring to the grid projection
	LineString projLs;
	projLs.reserve(ls.size());
	for (const auto& l : ls)
	{
		projLs.push_back(grid->world2Proj(l));
	}

	// Iterate through all points in the linestring and rasterise between them
	const auto lsSize = ls.size();
#pragma omp parallel for shared(projLs), if(lsSize > 2)
	for (int i = 0; i < lsSize - 1; ++i)
	{
		// Narrow down the possible voxels intersected by passing through bresenham algo
		// This requires projection to local grid coords as bresenham is integer based
		const auto& prevProjP = grid->world2Local(ls[i]);
		const auto& projP = grid->world2Local(ls[i + 1]);

		// This is effectively a 'thick' bresenham implementation in 3D. This generates a sphere of voxels around each of the endpoints
		// and runs bresenham between each of the correspoinding points to get all intermediate points that are intersected by the
		// "tube" around the central line
		// std::vector<std::vector<Vec3i>> voxelPositionVecs;
		// for (int x = -nSurroundingVoxels; x < nSurroundingVoxels + 1; ++x)
		// {
		// 	for (int y = -nSurroundingVoxels; y < nSurroundingVoxels + 1; ++y)
		// 	{
		// 		for (int z = -nSurroundingVoxels; z < nSurroundingVoxels + 1; ++z)
		// 		{
		// 			voxelPositionVecs.emplace_back(
		// 				Bresenham3D::line3d(prevProjP + Vec3i(x, y, z), projP + Vec3i(x, y, z)));
		// 		}
		// 	}
		// }
		// std::vector<Vec3i> points;
		// // Prealloc. All spheres around points should have the same size,so just pick the first one
		// points.reserve(voxelPositionVecs.size() * voxelPositionVecs[0].size());
		// for (const auto& line : voxelPositionVecs)
		// {
		// 	points.insert(points.end(), line.begin(), line.end());
		// }
		auto points = Bresenham3D::line3d(prevProjP, projP);
		// Remove duplicate voxels
		std::sort(points.begin(), points.end());
		points.erase(std::unique(points.begin(), points.end()), points.end());

		// Iterate possible voxels which intersect
		for (auto& c : points)
		{
			// Return if coord is outside of grid bounds
			if (!grid->isInBounds(c)) continue;

			// Create the bounds in the grid projection coords
			const auto minC = grid->local2Proj(c[0], c[1], c[2]);
			const auto maxC = grid->local2Proj(c[0] + 1, c[1] + 1, c[2] + 1);
			const BoostVoxel bvox(minC, maxC);

			// std::vector<BoostPoly> outPolys;
			// intersection(bvox, bufferedPoly, outPolys);

			// Create container for results and prealloc
			std::vector<LineString> out;
			// Do the intersection with the previously reprojected linestring
			intersection(bvox, projLs, out);
			// Ignore if no intersection
			if (out.empty()) continue;

			// Accumulate the length of intersecting trajectories within the voxel
			double l = 0;
			for (const auto& intersectingLs : out)
			{
				l += intersectingLs.getLength();
			}

			// See if a voxel already exists, if so add to it, otherwise make a new one and insert it
			auto voxel = grid->getVoxelForLocalCoord(c);
			voxel.airRisk += l;
#pragma omp critical
			grid->insertVoxel(c[0], c[1], c[2], voxel);
		}
		// prevProjP = projP;
	}
#endif
}

// void VoxelGridBuilderVisitor::handlePoint(const Point& p) const
// {
// 	// Assuming a point only has a single geom element, which it should...
// 	const auto projP = grid->world2Local(p[0]);
// 	handleVoxelCoord(projP);
// }


void VoxelGridBuilderVisitor::handleVoxelCoord(const Eigen::Vector3i& coord) const
{
	// Return if coord is outside of grid bounds
	if (!grid->isInBounds(coord)) return;


	// See if a voxel already exists, if so add to it, otherwise make a new one and insert it
	// Add one aircraft per cell volume to the cell
	grid->airRiskVals[grid->getGridIndex(coord)] += 1 / (grid->xyRes * grid->xyRes * grid->zRes);
	// auto voxel = grid->getVoxelForLocalCoord(coord);
	// ++voxel.airRisk;
	// grid->insertVoxel(coord[0], coord[1], coord[2], voxel);
}
