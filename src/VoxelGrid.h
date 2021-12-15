#ifndef VOXELGRID_H
#define VOXELGRID_H

#ifndef VOXEL_GRID_BOUNDS_CHECK
#define VOXEL_GRID_BOUNDS_CHECK 1
#include <vector>
#endif

#include <Eigen/Dense>
#include <proj.h>

struct Voxel
{
	Voxel()
	{
	};

	Voxel(const int x, const int y, const int z, const bool blocked, const double air_risk, const double ground_risk)
		: x(x),
		  y(y),
		  z(z),
		  blocked(blocked),
		  airRisk(air_risk),
		  groundRisk(ground_risk)
	{
	}

	int x, y, z;
	bool blocked;
	double airRisk, groundRisk;
};

class VoxelGrid
{
	friend class VoxelGridBuilderVisitor;
public:
	VoxelGrid(unsigned int sizeX, unsigned int sizeY, unsigned int sizeZ, double xyRes, double zRes,
	          const Eigen::Vector3d& worldOrigin,
	          const char* worldSrs);

	VoxelGrid(const VoxelGrid& other) = delete;

	VoxelGrid(VoxelGrid&& other) noexcept: sizeX(other.sizeX),
	                                       sizeY(other.sizeY),
	                                       sizeZ(other.sizeZ),
	                                       max1DIndex(other.max1DIndex),
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
		max1DIndex = other.max1DIndex;
		xyRes = other.xyRes;
		zRes = other.zRes;
		projectionOrigin = other.projectionOrigin;
		worldSrs = other.worldSrs;
		projectionSrs = other.projectionSrs;
		return *this;
	}

	~VoxelGrid();

	/**
	 * @brief Get a voxel object which contains the given coordinates or nullptr if not
	 * @param worldCoord the coordinates to evaluate in EPSG:4326
	 * @return the voxel containing the coordinate or nullptr
	*/
	Voxel getVoxelForWorldCoord(const Eigen::Vector3d& worldCoord) const;
	/**
	 * @brief Get a voxel object at the given indices or nullptr is not
	 * @param localCoord the indices to evaluate
	 * @return the voxel or nullptr
	*/
	Voxel getVoxelForLocalCoord(const Eigen::Vector3i& localCoord) const;
	/**
	 * @brief Get the EPSG:4326 coordinate at the centre point of the given voxel
	 * @param voxel the voxel to evaluate
	 * @return the EPSG:4326 coordinates of the voxel centre
	*/
	Eigen::Vector3d getCoordForVoxel(Voxel* voxel) const;

	/**
	 * @brief Insert a voxel into the given coords. This overwrites any existing voxels in the same location without checking.
	 * @param lat latitude of voxel
	 * @param lon longitude of voxel
	 * @param alt altitude of voxel in metres
	 * @param voxel the voxel to write in
	*/
	void insertVoxel(double lat, double lon, double alt, const Voxel& voxel);
	/**
	 * @brief Insert a voxel into given local indices. This overwrites any existing voxels in the same location without checking.
	 * @param x local x index
	 * @param y local y index
	 * @param z local z index
	 * @param voxel the voxel to write in
	*/
	void insertVoxel(int x, int y, int z, const Voxel& voxel);

	/**
	 * @brief Remove a voxel at the given coords.
	 * @param lat latitude of voxel
	 * @param lon longitude of voxel
	 * @param alt altitude of voxel in metres
	*/
	void removeVoxel(double lat, double lon, double alt);
	/**
	 * @brief Remove a voxel at given indices.
	 * @param x local x index
	 * @param y local y index
	 * @param z local z index
	*/
	void removeVoxel(int x, int y, int z);


	/**
	 * @brief Reproject world (EPSG:4326) coordinates to local voxel indices
	 * @param worldCoord the world coordinates to reproject
	 * @return the voxel indices
	*/
	Eigen::Vector3i world2Local(const Eigen::Vector3d& worldCoord) const;
	Eigen::Vector3i world2Local(double lat, double lon, double alt) const;

	/**
	 * @brief Test if the given voxel coord is within bounds
	 * @param localCoord the coord to test
	 * @return whether the coord is in bounds
	*/
	bool isInBounds(const Eigen::Vector3i& localCoord) const;

	double getMaxAirRisk() const;
	double getMaxGroundRisk() const;

	void writeToNetCDF(const std::string& path) const;

private:
	int getGridIndex(unsigned int x, unsigned int y, unsigned int z) const;
	int getGridIndex(const Eigen::Vector3i& localCoord) const;

	Eigen::Vector3d local2World(int x, int y, int z) const;
	Eigen::Vector3d local2World(const Eigen::Vector3i& localCoord) const;

	Eigen::Vector3d world2Proj(double lat, double lon, double alt) const;
	Eigen::Vector3d world2Proj(const Eigen::Vector3d& worldCoord) const;

	Eigen::Vector3d local2Proj(int x, int y, int z) const;
	Eigen::Vector3d local2Proj(const Eigen::Vector3i& localCoord) const;

protected:
	unsigned int sizeX, sizeY, sizeZ;
	unsigned int max1DIndex;
	float xyRes;
	float zRes;

	std::vector<double> airRiskVals, groundRiskVals;
	std::vector<bool> blockedVals;

	Eigen::Vector3d projectionOrigin;
	const char* worldSrs;
	const char* projectionSrs;
	PJ* reproj;
	PJ_CONTEXT* projCtx;
	// Voxel** grid;
};
#endif // VOXELGRID_H
