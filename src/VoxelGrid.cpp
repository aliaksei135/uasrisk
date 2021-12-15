#include "VoxelGrid.h"

#include <algorithm>
#include <iostream>
#include <omp.h>

#include <netcdf.h>
#include <stdexcept>

VoxelGrid::VoxelGrid(const unsigned sizeX, const unsigned sizeY, const unsigned sizeZ, const double xyRes,
                     const double zRes,
                     const Eigen::Vector3d& worldOrigin,
                     const char* worldSrs = "EPSG:4326"): sizeX(sizeX), sizeY(sizeY), sizeZ(sizeZ),
                                                          max1DIndex(sizeX * sizeY * sizeZ), xyRes(xyRes), zRes(zRes),
                                                          worldSrs(worldSrs),
                                                          projectionSrs("EPSG:3857"), projCtx(proj_context_create())
{
	airRiskVals.resize(max1DIndex, 0);
	groundRiskVals.resize(max1DIndex, 0);
	blockedVals.resize(max1DIndex, false);
	max1DIndex--; // decrement as this isn't MATLAB


#ifdef PROJ_DATA_PATH
	const char* projDataPaths[1];
	projDataPaths[0] = PROJ_DATA_PATH;
	proj_context_set_search_paths(projCtx, 1, projDataPaths);
#endif
	reproj = proj_create_crs_to_crs(projCtx, worldSrs, projectionSrs, nullptr);

	const auto projOrigin = proj_trans(reproj, PJ_FWD, {worldOrigin[0], worldOrigin[1], worldOrigin[2]});
	projectionOrigin = {projOrigin.enu.e, projOrigin.enu.n, projOrigin.enu.u};
}

VoxelGrid::~VoxelGrid() = default;

Voxel VoxelGrid::getVoxelForWorldCoord(const Eigen::Vector3d& worldCoord) const
{
	const auto localCoord = world2Local(worldCoord);
	return getVoxelForLocalCoord(localCoord);
}

Voxel VoxelGrid::getVoxelForLocalCoord(const Eigen::Vector3i& localCoord) const
{
	const auto idx = getGridIndex(localCoord[0], localCoord[1], localCoord[2]);
	return {
		localCoord[0], localCoord[1], localCoord[2],
		blockedVals[idx], airRiskVals[idx], groundRiskVals[idx]
	};
}

Eigen::Vector3d VoxelGrid::getCoordForVoxel(Voxel* voxel) const
{
	const Eigen::Vector3i localCoords(voxel->x, voxel->y, voxel->z);
	const Eigen::Vector3d projCoords((localCoords[0] * xyRes) + projectionOrigin[0],
	                                 (localCoords[1] * xyRes) + projectionOrigin[1],
	                                 (localCoords[2] * zRes) + projectionOrigin[2]);
	const auto out = proj_trans(reproj, PJ_INV, {projCoords[0], projCoords[1], projCoords[2]});
	return {out.enu.e, out.enu.n, out.enu.u};
}

void VoxelGrid::insertVoxel(const double lat, const double lon, const double alt, const Voxel& voxel)
{
	const auto& localCoord = world2Local(lat, lon, alt);
	insertVoxel(localCoord[0], localCoord[1], localCoord[2], voxel);
}

void VoxelGrid::insertVoxel(const int x, const int y, const int z, const Voxel& voxel)
{
	const auto& idx = getGridIndex(x, y, z);
	airRiskVals[idx] = voxel.airRisk;
	groundRiskVals[idx] = voxel.groundRisk;
	blockedVals[idx] = voxel.blocked;
}

void VoxelGrid::removeVoxel(const double lat, const double lon, const double alt)
{
	const auto& localCoord = world2Local(lat, lon, alt);
	removeVoxel(localCoord[0], localCoord[1], localCoord[2]);
}

void VoxelGrid::removeVoxel(const int x, const int y, const int z)
{
	const int idx = getGridIndex(x, y, z);
	airRiskVals[idx] = 0;
	groundRiskVals[idx] = 0;
	blockedVals[idx] = false;
}

int VoxelGrid::getGridIndex(const unsigned int x, const unsigned int y, const unsigned int z) const
{
	// I don't trust this to BODMAS properly...
	// ReSharper disable once CppRedundantParentheses
	const auto& i = (x * sizeY * sizeZ) + (y * sizeZ) + z;
#if VOXEL_GRID_BOUNDS_CHECK == 1
	if (i > max1DIndex)
	{
		throw std::out_of_range("Voxel Index out of bounds");
	}
#endif
	return i;
}

int VoxelGrid::getGridIndex(const Eigen::Vector3i& localCoord) const
{
	return getGridIndex(localCoord[0], localCoord[1], localCoord[2]);
}

bool VoxelGrid::isInBounds(const Eigen::Vector3i& localCoord) const
{
	return localCoord[0] < sizeX && localCoord[1] < sizeY && localCoord[2] < sizeZ
		&& localCoord[0] > -1 && localCoord[1] > -1 && localCoord[2] > -1;
}

Eigen::Vector3i VoxelGrid::world2Local(const double lat, const double lon, const double alt) const
{
	return world2Local({lon, lat, alt});
}

Eigen::Vector3d VoxelGrid::local2Proj(const int x, const int y, const int z) const
{
	return local2Proj({x, y, z});
}

Eigen::Vector3d VoxelGrid::world2Proj(const double lat, const double lon, const double alt) const
{
	return world2Proj({lon, lat, alt});
}

Eigen::Vector3d VoxelGrid::world2Proj(const Eigen::Vector3d& worldCoord) const
{
	const auto out = proj_trans(reproj, PJ_FWD, {worldCoord[0], worldCoord[1], worldCoord[2]});
	return {out.enu.e, out.enu.n, out.enu.u};
}

Eigen::Vector3d VoxelGrid::local2World(const int x, const int y, const int z) const
{
	return local2World({x, y, z});
}

Eigen::Vector3i VoxelGrid::world2Local(const Eigen::Vector3d& worldCoord) const
{
	const auto out = proj_trans(reproj, PJ_FWD, {worldCoord[0], worldCoord[1], worldCoord[2]});

	return {
		static_cast<int>((out.enu.e - projectionOrigin[0]) / xyRes + 0.5f),
		static_cast<int>((out.enu.n - projectionOrigin[1]) / xyRes + 0.5f),
		static_cast<int>((out.enu.u - projectionOrigin[2]) / zRes + 0.5f)
	};
}

Eigen::Vector3d VoxelGrid::local2Proj(const Eigen::Vector3i& localCoord) const
{
	return {
		localCoord[0] * xyRes + projectionOrigin[0], localCoord[1] * xyRes + projectionOrigin[1],
		localCoord[2] * zRes + projectionOrigin[2]
	};
}

Eigen::Vector3d VoxelGrid::local2World(const Eigen::Vector3i& localCoord) const
{
	const Eigen::Vector3d projCoords = local2Proj(localCoord);

	const auto worldCoords = proj_trans(reproj, PJ_INV, {projCoords[0], projCoords[1], projCoords[2]});

	return {worldCoords.enu.e, worldCoords.enu.n, worldCoords.enu.u};
}

double VoxelGrid::getMaxAirRisk() const
{
	return *std::max_element(airRiskVals.begin(), airRiskVals.end());
}

double VoxelGrid::getMaxGroundRisk() const
{
	return *std::max_element(groundRiskVals.begin(), groundRiskVals.end());
}

void VoxelGrid::writeToNetCDF(const std::string& path) const
{
	auto* lats = new float[sizeY];
	auto* lons = new float[sizeX];
	auto* alts = new float[sizeZ];

	const auto fullSize = sizeX * sizeY * sizeZ;
	auto* airRisks = static_cast<float*>(malloc(fullSize * sizeof(float)));
	auto* groundRisks = static_cast<float*>(malloc(fullSize * sizeof(float)));
	auto* blockeds = static_cast<short*>(malloc(fullSize * sizeof(short)));

	for (int i = 0; i < sizeX; ++i)
		lons[i] = local2World(i, 0, 0)[0];

	for (int i = 0; i < sizeY; ++i)
		lats[i] = local2World(0, i, 0)[1];

	for (int i = 0; i < sizeZ; ++i)
		alts[i] = local2World(0, 0, i)[2];

	for (int i = 0; i < fullSize; ++i)
	{
		airRisks[i] = airRiskVals[i];
		groundRisks[i] = groundRiskVals[i];
		blockeds[i] = blockedVals[i] ? 1 : 0;
	}

	int ncID, latDimID, lonDimID, altDimID, airRiskDimID, groundRiskDimID, blockedDimID, latVarID, lonVarID,
	    altVarID, airRiskVarID, groundRiskVarID, blockedVarID;
	// NcFile ncFile(path, NcFile::replace);
	nc_create(path.c_str(), NC_CLOBBER, &ncID);
	nc_def_dim(ncID, "Latitude", sizeY, &latDimID);
	nc_def_dim(ncID, "Longitude", sizeX, &lonDimID);
	nc_def_dim(ncID, "Altitude", sizeZ, &altDimID);
	nc_def_dim(ncID, "Air Risk", sizeX * sizeY * sizeZ, &airRiskDimID);
	nc_def_dim(ncID, "Ground Risk", sizeX * sizeY * sizeZ, &groundRiskDimID);
	nc_def_dim(ncID, "Blocked", sizeX * sizeY * sizeZ, &blockedDimID);

	const int dimArr[3] = {lonDimID, latDimID, altDimID};

	const char* degStr[1] = {"degrees"};
	const char* metreStr[1] = {"metres"};
	nc_def_var(ncID, "Latitude", NC_FLOAT, 1, &latDimID, &latVarID);
	nc_put_att_string(ncID, latVarID, "units", 1, degStr);
	nc_def_var(ncID, "Longitude", NC_FLOAT, 1, &lonDimID, &lonVarID);
	nc_put_att_string(ncID, lonVarID, "units", 1, degStr);
	nc_def_var(ncID, "Altitude", NC_FLOAT, 1, &altDimID, &altVarID);
	nc_put_att_string(ncID, altVarID, "units", 1, metreStr);

	nc_def_var(ncID, "Air Risk", NC_FLOAT, 3, dimArr, &airRiskVarID);
	nc_def_var(ncID, "Ground Risk",NC_FLOAT, 3, dimArr, &groundRiskVarID);
	nc_def_var(ncID, "Blocked",NC_SHORT, 3, dimArr, &blockedVarID);

	nc_put_var_float(ncID, latVarID, lats);
	nc_put_var_float(ncID, lonVarID, lons);
	nc_put_var_float(ncID, altVarID, alts);

	nc_put_var_float(ncID, groundRiskVarID, groundRisks);
	nc_put_var_float(ncID, airRiskVarID, airRisks);
	nc_put_var_short(ncID, blockedVarID, blockeds);

	// const auto latDim = ncFile.addDim("Latitude", sizeY);
	// const auto lonDim = ncFile.addDim("Longitude", sizeX);
	// const auto altDim = ncFile.addDim("Altitude", sizeZ);
	// const auto airRiskDim = ncFile.addDim("Air Risk", sizeX * sizeY * sizeZ);
	// const auto groundRiskDim = ncFile.addDim("Ground Risk", sizeX * sizeY * sizeZ);
	// const auto blockedDim = ncFile.addDim("Blocked", sizeX * sizeY * sizeZ);
	//
	// const std::vector<NcDim> dimVec{lonDim, latDim, altDim};
	//
	// const auto latVar = ncFile.addVar("Latitude", ncFloat, latDim);
	// latVar.putAtt("units", "degrees_north");
	// const auto lonVar = ncFile.addVar("Longitude", ncFloat, lonDim);
	// latVar.putAtt("units", "degrees_east");
	// const auto altVar = ncFile.addVar("Altitude", ncFloat, altDim);
	// latVar.putAtt("units", "metres");
	//
	// const auto airRiskVar = ncFile.addVar("Air Risk", ncFloat, dimVec);
	// const auto groundRiskVar = ncFile.addVar("Ground Risk", ncFloat, dimVec);
	// const auto blockedVar = ncFile.addVar("Blocked", ncShort, dimVec);
	//
	// latVar.putVar(lats);
	//
	// lonVar.putVar(lons);
	// altVar.putVar(alts);
	//
	// airRiskVar.putVar(airRisks);
	// groundRiskVar.putVar(groundRisks);
	// blockedVar.putVar(blockeds);

	// ncFile.close();
	nc_close(ncID);
	delete[] lats, lons, alts, airRisks, groundRisks, blockeds;
}
