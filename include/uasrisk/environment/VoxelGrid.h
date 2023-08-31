#ifndef VOXELGRID_H
#define VOXELGRID_H

#ifndef VOXEL_GRID_BOUNDS_CHECK
#define VOXEL_GRID_BOUNDS_CHECK 1
#endif

#include <unordered_map>

#include "TypeDefs.h"
#include <vector>
#include <Eigen/Dense>
#include <proj.h>

namespace ur
{
	class VoxelGrid
	{
		friend class VoxelGridBuilder;
	public:
		/**
		 * \brief Create a voxel grid within the given bounds and resolution
		 * \param bounds an array of bounding coordinates in the form: South, West, Bottom, North, East, Top
		 * \param xyRes the lateral resolution in metres
		 * \param zRes the vertical resolution in metres
		 * \param worldSrs the world coordinate system, defaults to EPSG:4326 (WGS84)
		 */
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


		/**
		 * \brief Add a layer to the grid
		 * \param layerName name of the layer
		 * \param constValue a default value to fill the tensor with, defaults to 0
		 */
		void add(const std::string& layerName, const FPScalar constValue = 0);

		/**
		 * \brief Add a layer to the grid
		 * \param layerName name of the layer
		 * \param data predefined data set the tensor to, this must match the Voxel Grid dimensions
		 */
		void add(const std::string& layerName, const Matrix& data);


		/**
		 * \brief Return the value of the layer at a given index
		 * \param layerName the layer name
		 * \param idx the local indices to return
		 * \return the value of the coeff
		 */
		FPScalar at(const std::string& layerName, const Index& idx) const;

		/**
		 * \brief Return an editable reference to the value of the layer at the given index
		 * \param layerName the layer name
		 * \param idx the local indices to return
		 * \return a reference to the coeff at the indices
		 */
		FPScalar& at(const std::string& layerName, const Index& idx);


		/**
		 * \brief Return the value of the layer at the given world coordinates
		 * \param layerName the layer name
		 * \param pos the world coordinates to return
		 * \return the value of the coeff
		 */
		FPScalar atPosition(const std::string& layerName, const Position& pos) const;

		/**
		 * \brief Return an editable reference to the value of the layer at the given world coordinates
		 * \param layerName the layer name
		 * \param pos the world coordinates to return
		 * \return a reference to the coeff at the coordinates
		 */
		FPScalar& atPosition(const std::string& layerName, const Position& pos);


		/**
		 * \brief Return the entire tensor for a layer
		 * \param layerName the layer name
		 * \return the tensor of data 
		 */
		Matrix get(const std::string& layerName) const;
		/**
		 * \brief Return a reference to the entire tensor for a layer
		 * \param layerName the layer name
		 * \return a reference to the tensor
		 */
		Matrix& get(const std::string& layerName);


		/**
		 * @brief Reproject world (EPSG:4326) coordinates to local indices
		 * @param worldCoord the world coordinates to reproject
		 * @return the local indices
		*/
		Index world2Local(const Position& worldCoord) const;

		/*
		 * A per-component overload of above
		 */
		Index world2Local(FPScalar lon, FPScalar lat, FPScalar alt) const;

		/**
		 * @brief Test if the given local indices is within bounds
		 * @param localCoord the indices to test
		 * @return whether they are in bounds
		*/
		bool isInBounds(const Index& localCoord) const;


//		/**
//		 * \brief Write the risk layers to netCDF format for export.
//		 * \details This function uses the COARDS netCDF conventions for geospatial data without the time component.
//		 * This means the data can be displayed in external software such as Panoply.
//		 * \param path file path to write to
//		 */
//		void writeToNetCDF(const std::string& path) const;


		/**
		 * \brief Return the lengths of the grid
		 * \return x,y,z lengths
		 */
		Size getSize() const
		{
			return {sizeX, sizeY, sizeZ};
		}

		std::vector<std::string> getLayers() const;

		/**
		 * \brief Reproject local indices to world coordinates
		 * \param localCoord the local indices to reproject
		 * \return the world coordinates
		 */
		Position local2World(const Index& localCoord) const;

		/*
		 * A per-component overload of above
		 */
		Position local2World(int x, int y, int z) const;

	protected:
		int sizeX, sizeY, sizeZ;
		FPScalar xyRes;
		FPScalar zRes;
		std::unordered_map<std::string, ur::Matrix> layers;

		ur::Position projectionOrigin;
		const char* worldSrs;
		const char* projectionSrs;
		PJ* reproj;
		PJ_CONTEXT* projCtx;
	};
}
#endif // VOXELGRID_H
