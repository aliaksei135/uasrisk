#include "VoxelGridBuilder.h"

#include "uasrisk/environment/VoxelGrid.h"
#include <numeric>

#include "Bresenham3D.h"
#include <Eigen/Dense>

#include <boost/geometry.hpp>


#define AIR_RISK_COUNTS // Only use the cell intersection counts instead of calculating the length of cell intersections

using namespace boost::geometry;


template <typename T>
static bool isInsidePolygon(const ur::Polygon& polygon, const T& position)
{
    unsigned cross = 0;
    const auto& polyOuter = polygon.outer();
    for (size_t i = 0, j = polyOuter.size() - 1; i < polyOuter.size(); j = i++)
    {
        if (polyOuter[i].y() > position.y() != polyOuter[j].y() > position.y()
            && position.x() < (polyOuter[j].x() - polyOuter[i].x()) * (position.y() - polyOuter[i].y()) /
            (polyOuter[j].y() - polyOuter[i].y()) + polyOuter[i].x())

            cross++;
    }
    return cross % 2;
}

template <typename T = ur::FPScalar>
static std::array<T, 6> getPolyBounds(const ur::Polygon& poly)
{
    T minLon = std::numeric_limits<T>::max();
    T maxLon = std::numeric_limits<T>::min();
    T minLat = std::numeric_limits<T>::max();
    T maxLat = std::numeric_limits<T>::min();
    T minAlt = std::numeric_limits<T>::max();
    T maxAlt = std::numeric_limits<T>::min();

    for (const auto& pos : poly.outer())
    {
        const auto& x = pos.x();
        const auto& y = pos.y();
        const auto& z = pos.z();

        if (x > maxLon)
        {
            maxLon = x;
        }
        else if (x < minLon)
        {
            minLon = x;
        }

        if (y > maxLat)
        {
            maxLat = y;
        }
        else if (y < minLat)
        {
            minLat = y;
        }

        if (z > maxAlt)
        {
            maxAlt = z;
        }
        else if (z < minAlt)
        {
            minAlt = z;
        }
    }

    return {minLat, minLon, minAlt, maxLat, maxLon, maxAlt};
}


ur::VoxelGridBuilder::VoxelGridBuilder(VoxelGrid* grid) : grid(grid)
{
}

void ur::VoxelGridBuilder::handleBlockingPolygon(const std::vector<ExtrudedPolygon>& polys) const
{
    // for (const auto& poly : polys)
#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < polys.size(); ++i)
    {
        handleBlockingPolygon(polys[i]);
    }
}

void ur::VoxelGridBuilder::handleBlockingPolygon(const ExtrudedPolygon& poly) const
{
    double minLat = std::numeric_limits<double>::max();
    double minLon = std::numeric_limits<double>::max();
    double maxLat = std::numeric_limits<double>::min();
    double maxLon = std::numeric_limits<double>::min();
    for (const auto& p : poly.footprint.outer())
    {
        if (p.x() < minLon)
        {
            minLon = p.x();
        }
        if (p.x() > maxLon)
        {
            maxLon = p.x();
        }

        if (p.y() < minLat)
        {
            minLat = p.y();
        }
        if (p.y() > maxLat)
        {
            maxLat = p.y();
        }
    }

    const Position minCoord(minLon, minLat, poly.floor);
    const Position maxCoord(maxLon, maxLat, poly.ceiling);
    const auto& gridMinCoord = grid->world2Local(minCoord);
    const auto& gridMaxCoord = grid->world2Local(maxCoord);

    // #pragma omp parallel for collapse(3)
    for (int x = gridMinCoord.x() - 1; x < gridMaxCoord.x() + 1; ++x)
    {
        for (int y = gridMinCoord.y() - 1; y < gridMaxCoord.y() + 1; ++y)
        {
            for (int z = gridMinCoord.z() - 1; z < gridMaxCoord.z() + 1; ++z)
            {
                if (grid->isInBounds({x, y, z}))
                {
                    const auto& worldVoxelCoord = grid->local2World(x, y, z);
                    const auto within2D = isInsidePolygon(poly.footprint, worldVoxelCoord);
                    const auto& withinZ = worldVoxelCoord.z() >= poly.floor && worldVoxelCoord.z() <= poly.ceiling;

                    if (within2D && withinZ)
                    {
#pragma omp critical
                        grid->at("Blocked", {x, y, z});
                    }
                }
            }
        }
    }
}

void ur::VoxelGridBuilder::handleTrajectory(const std::vector<LineString>& lss) const
{
#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < lss.size(); ++i)
    {
        handleTrajectory(lss[i]);
    }
}


void ur::VoxelGridBuilder::handleTrajectory(const LineString& ls) const
{
#ifdef AIR_RISK_COUNTS
    // Iterate through all points in the linestring and rasterise between them
    const auto lsSize = ls.size();
    // #pragma omp parallel for //shared(ls), if(lsSize > 2)
    for (int i = 0; i < lsSize - 1; ++i)
    {
        // Narrow down the possible voxels intersected by passing through bresenham algo
        // This requires projection to local grid coords as bresenham is integer based
        const auto& prevProjP = grid->world2Local(ls[i]);
        const auto& projP = grid->world2Local(ls[i + 1]);
        // This uses grid indices not actual coords
        const FPScalar& angle = ((atan2((prevProjP.y() - projP.y()), (prevProjP.x() - projP.x()))) * 180.0 / M_PI);
        const auto& heading = std::fmod((90 - angle) + 360, 360);
        auto points = util::Bresenham3D::line3d(prevProjP, projP);
        for (const auto& c : points)
        {
            // handleVoxelCoord(c);
            // Return if coord is outside of grid bounds
            if (!grid->isInBounds(c)) continue;

#pragma omp critical
            {
                // See if a voxel already exists, if so add to it, otherwise make a new one and insert it
                // Add one aircraft per cell volume to the cell
                grid->at("Air Risk", c) += 1 / (grid->xyRes * grid->xyRes * grid->zRes);
                grid->at("Counts", c) += 1;
                grid->at("Headings", c) += heading;
            }
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

void ur::VoxelGridBuilder::handleVoxelCoord(const Index& coord) const
{
    // Return if coord is outside of grid bounds
    if (!grid->isInBounds(coord)) return;


    // See if a voxel already exists, if so add to it, otherwise make a new one and insert it
    // Add one aircraft per cell volume to the cell
    grid->at("Air Risk", coord) += 1 / (grid->xyRes * grid->xyRes * grid->zRes);
}
