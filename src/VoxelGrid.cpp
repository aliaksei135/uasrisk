#include "uasrisk/environment/VoxelGrid.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <omp.h>
#include <cassert>

//#include <netcdf.h>
#include <stdexcept>
#include <iostream>

ur::VoxelGrid::VoxelGrid(const std::array<FPScalar, 6> bounds, const FPScalar xyRes,
	const FPScalar zRes,
	const char* worldSrs) : xyRes(xyRes), zRes(zRes),
							sizeZ(static_cast<int>((bounds[5] - bounds[2]) / zRes)),
							worldSrs(worldSrs),
							projectionSrs("EPSG:3395"), projCtx(proj_context_create())
{
	std::cout << "uasrisk: Creating PROJ obj...\n";
	const auto* envDataDir = std::getenv("PROJ_LIB");
	if (envDataDir == nullptr)
	{
		std::cout << "uasrisk: PROJ_LIB not set. Falling back";
#ifdef PROJ_DATA_PATH
		const char* projDataPaths[1];
		projDataPaths[0] = PROJ_DATA_PATH;
		std::cout << "uasrisk: Using Internally set PROJ data dir: " << projDataPaths[0] << "\n";
		proj_context_set_search_paths(projCtx, 1, projDataPaths);
#endif
	}
	reproj = proj_create_crs_to_crs(projCtx, worldSrs, projectionSrs, nullptr);

	// This reprojects EPSG:4326 to EPSG:3395 by default
	const auto swProjPoint = proj_trans(reproj, PJ_FWD, { bounds[0], bounds[1], bounds[2] });
	this->projectionOrigin = {
		static_cast<FPScalar>(swProjPoint.enu.e),
		static_cast<FPScalar>(swProjPoint.enu.n),
		static_cast<FPScalar>(swProjPoint.enu.u)
	};
	//TODO: Should an altitude be set in the projection origin?
	const auto neProjPoint = proj_trans(reproj, PJ_FWD, { bounds[3], bounds[4], bounds[5] });
	const auto dx = std::abs(swProjPoint.enu.e - neProjPoint.enu.e);
	const auto dy = std::abs(swProjPoint.enu.n - neProjPoint.enu.n);
	const int xLength = static_cast<int>(dx / static_cast<float>(xyRes));
	const int yLength = static_cast<int>(dy / static_cast<float>(xyRes));

	this->sizeX = xLength;
	this->sizeY = yLength;

	add("Blocked");
	add("Ground Risk");
	add("Air Risk");
	add("Counts");
	add("Headings");
}

ur::VoxelGrid::~VoxelGrid() = default;

void ur::VoxelGrid::add(const std::string& layerName, const FPScalar constValue)
{
	Matrix data(sizeX, sizeY, sizeZ);
	data.setConstant(0);
	layers.insert({ layerName, data });
}

void ur::VoxelGrid::add(const std::string& layerName, const Matrix& data)
{
	const auto& size = data.dimensions();
	assert(data.rank() == 3);
	assert(size[0] == sizeX);
	assert(size[1] == sizeY);
	assert(size[2] == sizeZ);
	layers.insert({ layerName, data });
}

ur::FPScalar ur::VoxelGrid::at(const std::string& layerName, const Index& idx) const
{
	if ((idx >= 0).all() && (idx < getSize()).all())
	{
		return layers.at(layerName)(idx(0), idx(1), idx(2));
	}
	else
	{
		std::stringstream ss;
		ss << "Voxel Grid Index (" << idx(0) << "," << idx(1) << "," << idx(2) << " out of range for grid size (" << sizeX << "," << sizeY << "," << sizeZ << ")";
		throw std::out_of_range(ss.str());
	}
}

ur::FPScalar& ur::VoxelGrid::at(const std::string& layerName, const Index& idx)
{
	if ((idx >= 0).all() && (idx < getSize()).all())
	{
		return layers.at(layerName)(idx(0), idx(1), idx(2));
	}
	else
	{
		std::stringstream ss;
		ss << "Voxel Grid Index (" << idx(0) << "," << idx(1) << "," << idx(2) << " out of range for grid size (" << sizeX << "," << sizeY << "," << sizeZ << ")";
		throw std::out_of_range(ss.str());
	}
}

ur::FPScalar ur::VoxelGrid::atPosition(const std::string& layerName, const Position& pos) const
{
	const auto idx = world2Local(pos);
	return at(layerName, idx);
}

ur::FPScalar& ur::VoxelGrid::atPosition(const std::string& layerName, const Position& pos)
{
	const auto idx = world2Local(pos);
	return at(layerName, idx);
}

ur::Matrix ur::VoxelGrid::get(const std::string& layerName) const
{
	return layers.at(layerName);
}

ur::Matrix& ur::VoxelGrid::get(const std::string& layerName)
{
	return layers.at(layerName);
}

bool ur::VoxelGrid::isInBounds(const Index& localCoord) const
{
	return localCoord[0] < sizeX && localCoord[1] < sizeY && localCoord[2] < sizeZ
		&& localCoord[0] > -1 && localCoord[1] > -1 && localCoord[2] > -1;
}

ur::Index ur::VoxelGrid::world2Local(const FPScalar lon, const FPScalar lat, const FPScalar alt) const
{
	const auto out = proj_trans(reproj, PJ_FWD, { lat, lon, alt });

	return {
		static_cast<int>((out.enu.e - projectionOrigin[0]) / xyRes),
		static_cast<int>(((out.enu.n - projectionOrigin[1]) / xyRes)),
		static_cast<int>((out.enu.u - projectionOrigin[2]) / zRes)
	};
}

ur::Index ur::VoxelGrid::world2Local(const Position& worldCoord) const
{
	return world2Local(worldCoord[0], worldCoord[1], worldCoord[2]);
}

ur::Position ur::VoxelGrid::local2World(const int x, const int y, const int z) const
{
	// return local2World({x, y, z});

	const auto worldCoords = proj_trans(reproj, PJ_INV, {
		static_cast<ur::FPScalar>(x) * xyRes + projectionOrigin[0],
		static_cast<ur::FPScalar>(y) * xyRes + projectionOrigin[1],
		static_cast<ur::FPScalar>(z) * zRes + projectionOrigin[2]
	});

	return {
		static_cast<FPScalar>(worldCoords.enu.n), static_cast<FPScalar>(worldCoords.enu.e),
		static_cast<FPScalar>(worldCoords.enu.u)
	};
}

std::vector<std::string> ur::VoxelGrid::getLayers() const
{
	std::vector<std::string> keys;
	keys.reserve(layers.size());
	for (const auto& pair : layers)
	{
		keys.emplace_back(pair.first);
	}
	return keys;
}

ur::Position ur::VoxelGrid::local2World(const Index& localCoord) const
{
	return local2World(localCoord[0], localCoord[1], localCoord[2]);
}

//void ur::VoxelGrid::writeToNetCDF(const std::string& path) const
//{
//	std::vector<float> lats(sizeX);
//	std::vector<float> lons(sizeY);
//	std::vector<float> alts(sizeZ);
//
//	const Matrix& airRiskMat = get("Air Risk");
//	const Matrix& groundRiskMat = get("Ground Risk");
//	const Matrix& blockedMat = get("Blocked");
//
//	const auto fullSize = sizeX * sizeY * sizeZ;
//	std::vector<float> airRisks(fullSize);
//	std::vector<float> groundRisks(fullSize);
//	std::vector<short> blockeds(fullSize);
//
//	for (int i = 0; i < sizeY; ++i)
//		lons[i] = local2World(i, 0, 0)[0];
//
//	for (int i = 0; i < sizeX; ++i)
//		lats[i] = local2World(0, i, 0)[1];
//
//	for (int i = 0; i < sizeZ; ++i)
//		alts[i] = local2World(0, 0, i)[2];
//
//	int c = 0;
//	for (int j = 0; j < sizeY; ++j)
//	{
//		for (int i = 0; i < sizeX; ++i)
//		{
//			for (int k = 0; k < sizeZ; ++k)
//			{
//				airRisks[c] = static_cast<float>(airRiskMat(i, j, k));
//				groundRisks[c] = static_cast<float>(groundRiskMat(i, j, k));
//				blockeds[c] = static_cast<short>(blockedMat(i, j, k));
//				++c;
//			}
//		}
//	}
//
//	int ncID, latDimID, lonDimID, altDimID, airRiskDimID, groundRiskDimID, blockedDimID, latVarID, lonVarID,
//		altVarID, airRiskVarID, groundRiskVarID, blockedVarID;
//	nc_create(path.c_str(), NC_CLOBBER | NC_NETCDF4, &ncID);
//	nc_def_dim(ncID, "Latitude", sizeX, &latDimID);
//	nc_def_dim(ncID, "Longitude", sizeY, &lonDimID);
//	nc_def_dim(ncID, "Altitude", sizeZ, &altDimID);
//	nc_def_dim(ncID, "Air Risk", sizeX * sizeY * sizeZ, &airRiskDimID);
//	nc_def_dim(ncID, "Ground Risk", sizeX * sizeY * sizeZ, &groundRiskDimID);
//	nc_def_dim(ncID, "Blocked", sizeX * sizeY * sizeZ, &blockedDimID);
//
//	const int dimArr[3] = { lonDimID, latDimID, altDimID };
//
//	const char* degEStr[1] = { "degrees_east" };
//	const char* degNStr[1] = { "degrees_north" };
//	const char* metreStr[1] = { "metres" };
//	const char* xStr[1] = { "X" };
//	const char* yStr[1] = { "Y" };
//	const char* zStr[1] = { "Z" };
//	nc_def_var(ncID, "Latitude", NC_FLOAT, 1, &latDimID, &latVarID);
//	nc_put_att_string(ncID, latVarID, "units", 1, degNStr);
//	nc_put_att_string(ncID, latVarID, "axis", 1, yStr);
//
//	nc_def_var(ncID, "Longitude", NC_FLOAT, 1, &lonDimID, &lonVarID);
//	nc_put_att_string(ncID, lonVarID, "units", 1, degEStr);
//	nc_put_att_string(ncID, lonVarID, "axis", 1, xStr);
//
//	nc_def_var(ncID, "Altitude", NC_FLOAT, 1, &altDimID, &altVarID);
//	nc_put_att_string(ncID, altVarID, "units", 1, metreStr);
//	nc_put_att_string(ncID, altVarID, "axis", 1, zStr);
//
//	nc_def_var(ncID, "Air Risk", NC_FLOAT, 3, dimArr, &airRiskVarID);
//	nc_def_var(ncID, "Ground Risk", NC_FLOAT, 3, dimArr, &groundRiskVarID);
//	nc_def_var(ncID, "Blocked", NC_SHORT, 3, dimArr, &blockedVarID);
//
//	nc_put_var_float(ncID, latVarID, lats.data());
//	nc_put_var_float(ncID, lonVarID, lons.data());
//	nc_put_var_float(ncID, altVarID, alts.data());
//
//	nc_put_var_float(ncID, groundRiskVarID, groundRisks.data());
//	nc_put_var_float(ncID, airRiskVarID, airRisks.data());
//	nc_put_var_short(ncID, blockedVarID, blockeds.data());
//
//	nc_close(ncID);
//}
