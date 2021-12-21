#ifndef VOXELGRID_H
#define VOXELGRID_H

#ifndef VOXEL_GRID_BOUNDS_CHECK
#define VOXEL_GRID_BOUNDS_CHECK 1
#include <unordered_map>

#include "TypeDefs.h"
#endif

#include <vector>
#include <Eigen/Dense>
#include <proj.h>

namespace ur
{
	class VoxelGrid
	{
		friend class VoxelGridBuilder;
	public:
		VoxelGrid(const std::array<FPScalar, 6> bounds, FPScalar xyRes, FPScalar zRes,
		          const char* worldSrs = "EPSG:4326");

		VoxelGrid(const VoxelGrid& other) = delete;

		VoxelGrid(VoxelGrid&& other) noexcept: sizeX(other.sizeX),
		                                       sizeY(other.sizeY),
		                                       sizeZ(other.sizeZ),
		                                       xyRes(other.xyRes),
		                                       zRes(other.zRes),
		                                       projectionOrigin(other.projectionOrigin),
		                                       worldSrs(other.worldSrs),
		                                       projectionSrs(other.projectionSrs), reproj(other.reproj),
		                                       projCtx(other.projCtx)
		{
		}

		VoxelGrid& operator=(const VoxelGrid& other) = delete;

		VoxelGrid& operator=(VoxelGrid&& other) noexcept
		{
			if (this == &other)
				return *this;
			sizeX = other.sizeX;
			sizeY = other.sizeY;
			sizeZ = other.sizeZ;
			xyRes = other.xyRes;
			zRes = other.zRes;
			projectionOrigin = other.projectionOrigin;
			worldSrs = other.worldSrs;
			projectionSrs = other.projectionSrs;
			return *this;
		}

		~VoxelGrid();

		void add(const std::string& layerName, const FPScalar constValue);
		void add(const std::string& layerName, const Matrix& data);

		FPScalar at(const std::string& layerName, const Index& idx) const;
		FPScalar& at(const std::string& layerName, const Index& idx);

		FPScalar atPosition(const std::string& layerName, const Position& pos) const;
		FPScalar& atPosition(const std::string& layerName, const Position& pos);

		Matrix get(const std::string& layerName) const;
		Matrix& get(const std::string& layerName);


		/**
		 * @brief Reproject world (EPSG:4326) coordinates to local voxel indices
		 * @param worldCoord the world coordinates to reproject
		 * @return the voxel indices
		*/
		Index world2Local(const Position& worldCoord) const;
		Index world2Local(FPScalar lat, FPScalar lon, FPScalar alt) const;

		/**
		 * @brief Test if the given voxel coord is within bounds
		 * @param localCoord the coord to test
		 * @return whether the coord is in bounds
		*/
		bool isInBounds(const Index& localCoord) const;

		void writeToNetCDF(const std::string& path) const;

		ur::Size getSize() const
		{
			return {sizeX, sizeY, sizeZ};
		}

		// int getGridIndex(unsigned int x, unsigned int y, unsigned int z) const;
		// int getGridIndex(const Index& localCoord) const;

		Position local2World(int x, int y, int z) const;
		Position local2World(const Index& localCoord) const;

		// Eigen::Vector3d world2Proj(double lat, double lon, double alt) const;
		// Eigen::Vector3d world2Proj(const Eigen::Vector3d& worldCoord) const;
		//
		// Eigen::Vector3d local2Proj(int x, int y, int z) const;
		// Eigen::Vector3d local2Proj(const Eigen::Vector3i& localCoord) const;

		// protected:
		int sizeX, sizeY, sizeZ;
		FPScalar xyRes;
		FPScalar zRes;
		std::unordered_map<std::string, ur::Matrix> layers;

		ur::Position projectionOrigin;
		const char* worldSrs;
		const char* projectionSrs;
		PJ* reproj;
		PJ_CONTEXT* projCtx;
		// Voxel** grid;
	};
}
#endif // VOXELGRID_H
