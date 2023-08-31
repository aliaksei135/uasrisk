#include <iostream>

#include "uasrisk/environment/VoxelGrid.h"

#include <Eigen/Dense>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "pybind11_eigen_tensor.h"

#include "uasrisk/RiskVoxelGrid.h"
#include "uasgroundrisk/map_gen/GeospatialGridMap.h"

#include "GroundRiskHelpers.h"
#include "AirRiskHelpers.h"

namespace py = pybind11;
template<typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

class PyRiskVoxelGrid : public ur::RiskVoxelGrid
{
 public:
	PyRiskVoxelGrid(const std::array<ur::FPScalar, 6>& bounds, const ur::FPScalar xyRes, const ur::FPScalar zRes,
		const std::string& trajPath, const std::string& airspacePath,
		ugr::risk::AircraftModel* aircraftModel)
		: RiskVoxelGrid(bounds, xyRes, zRes, trajPath, airspacePath,
		new ugr::mapping::TemporalPopulationMap({ bounds[0], bounds[1], bounds[3], bounds[4] }, xyRes),
		aircraftModel, new ugr::risk::ObstacleMap({ bounds[0], bounds[1], bounds[3], bounds[4] }, xyRes),
		new ugr::risk::WeatherMap({ bounds[0], bounds[1], bounds[3], bounds[4] }, xyRes))
	{
		try
		{
			auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
			console_sink->set_level(spdlog::level::trace);
			console_sink->set_pattern("[uasrisk] [%^%l%$] %v");

			auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/uasrisk.out.log", true);
			file_sink->set_level(spdlog::level::trace);

			spdlog::sinks_init_list sink_list = { file_sink, console_sink };

			spdlog::logger logger("uasrisk", sink_list.begin(), sink_list.end());
			logger.set_level(spdlog::level::trace);
			logger.warn("this should appear in both console and file");
			logger.info("this message should not appear in the console, only in the file");

			// or you can even set multi_sink logger as default logger
			spdlog::set_default_logger(
				std::make_shared<spdlog::logger>("uasrisk", spdlog::sinks_init_list({ console_sink, file_sink })));
		}
		catch (const spdlog::spdlog_ex& ex)
		{
			std::cout << "Log initialization failed: " << ex.what() << std::endl;
		}
	}

	using ur::RiskVoxelGrid::eval;
};

PYBIND11_MODULE(_pyuasrisk, topModule)
{
	topModule.doc() = "Risk estimation for uncrewed aerial systems";
	topModule.attr("__version__") = "0.0.1"; //TODO grab from CMake defines

	////////////////////////////////////////////////////////////////
	// Base GridMap / VoxelGrid Classes
	////////////////////////////////////////////////////////////////
	// auto voxelGridModule = topModule.def_submodule("environment");
	py::class_<ur::VoxelGrid>(topModule, "VoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar>())
		.def("add", overload_cast_<const std::string&, const ur::FPScalar>()(&ur::VoxelGrid::add),
			"Add a layer with a constant value")
		.def("add", overload_cast_<const std::string&, const ur::Matrix&>()(&ur::VoxelGrid::add),
			"Add a layer with a prefilled grid")
		.def("at", overload_cast_<const std::string&, const ur::Index&>()(&ur::VoxelGrid::at),
			py::return_value_policy::reference_internal, "Return the value of the layer at a given index")
		.def("at", overload_cast_<const std::string&, const ur::Index&>()(&ur::VoxelGrid::at, py::const_),
			"Return the value of the layer at a given index")
		.def("atPosition", overload_cast_<const std::string&, const ur::Position&>()(&ur::VoxelGrid::atPosition),
			py::return_value_policy::reference_internal,
			"Return the value of the layer at the given world coordinates")
		.def("atPosition",
			overload_cast_<const std::string&, const ur::Position&>()(&ur::VoxelGrid::atPosition, py::const_),
			"Return the value of the layer at the given world coordinates")
		.def("get", overload_cast_<const std::string&>()(&ur::VoxelGrid::get),
			py::return_value_policy::reference_internal, "Return the entire tensor for a layer")
//		.def("get", overload_cast_<const std::string&>()(&ur::VoxelGrid::get, py::const_),
//		     "Return the entire tensor for a layer")
		.def("world2Local", overload_cast_<const ur::Position&>()(&ur::VoxelGrid::world2Local, py::const_),
			"Reproject world (EPSG:4326) coordinates to local indices")
		.def("world2Local",
			overload_cast_<ur::FPScalar, ur::FPScalar, ur::FPScalar>()(&ur::VoxelGrid::world2Local, py::const_),
			"Reproject world (EPSG:4326) coordinates to local indices")
		.def("isInBounds", &ur::VoxelGrid::isInBounds, "Test if the given local indices is within bounds")
//		.def("writeToNetCDF", &ur::VoxelGrid::writeToNetCDF, "Write the risk layers to netCDF format for export.")
		.def_property_readonly("size", &ur::VoxelGrid::getSize, "Return the lengths of the grid")
		.def_property_readonly("layers", &ur::VoxelGrid::getLayers, "Return the layers in the map")
		.def("local2World", overload_cast_<const ur::Index&>()(&ur::VoxelGrid::local2World, py::const_),
			"Reproject local indices to world coordinates")
		.def("local2World", overload_cast_<int, int, int>()(&ur::VoxelGrid::local2World, py::const_),
			"Reproject local indices to world coordinates");

	py::class_<ugr::gridmap::GridMap>(topModule, "GridMap")
		.def(py::init<>())
		.def("add", overload_cast_<const std::string&, const double>()(&ugr::gridmap::GridMap::add),
			"Add a layer with a constant value")
		.def("add", overload_cast_<const std::string&, const ugr::gridmap::Matrix&>()(&ugr::gridmap::GridMap::add),
			"Add a layer with a prefilled grid")
		.def("at", overload_cast_<const std::string&, const ugr::gridmap::Index&>()(&ugr::gridmap::GridMap::at),
			py::return_value_policy::reference_internal, "Return the value of the layer at a given index")
		.def("at",
			overload_cast_<const std::string&, const ugr::gridmap::Index&>()(&ugr::gridmap::GridMap::at, py::const_),
			"Return the value of the layer at a given index")
		.def("get", overload_cast_<const std::string&>()(&ugr::gridmap::GridMap::get),
			py::return_value_policy::reference_internal, "Return the entire tensor for a layer")
		.def("get", overload_cast_<const std::string&>()(&ugr::gridmap::GridMap::get, py::const_),
			"Return the entire tensor for a layer")
		.def("isInBounds", &ugr::gridmap::GridMap::isInBounds, "Test if the given local indices is within bounds")
//		.def("writeToNetCDF",
//			&ugr::gridmap::GridMap::writeToNetCDF,
//			"Write the risk layers to netCDF format for export.")
		.def_property_readonly("size", &ugr::gridmap::GridMap::getSize, "Return the lengths of the grid")
		.def_property_readonly("layers", &ugr::gridmap::GridMap::getLayers, "Return the layers in the map");

	py::class_<ugr::mapping::GeospatialGridMap, ugr::gridmap::GridMap>(topModule, "GeospatialGridMap")
		.def(py::init<std::array<float, 4>, float>())
		.def("world2Local",
			overload_cast_<const ugr::mapping::Position&>()(&ugr::mapping::GeospatialGridMap::world2Local, py::const_),
			"Reproject world coordinates to local indices")
		.def("local2World",
			overload_cast_<const ugr::gridmap::Index&>()(&ugr::mapping::GeospatialGridMap::local2World, py::const_),
			"Reproject local indices to world coordinates")
		.def("atPosition",
			overload_cast_<const std::string&,
						   const ugr::gridmap::Position&>()(&ugr::mapping::GeospatialGridMap::atPosition),
			py::return_value_policy::reference_internal,
			"Return the value of the layer at the given world coordinates")
		.def("atPosition",
			overload_cast_<const std::string&,
						   const ugr::gridmap::Position&>()(&ugr::mapping::GeospatialGridMap::atPosition, py::const_),
			"Return the value of the layer at the given world coordinates")
		.def("isInBounds",
			&ugr::mapping::GeospatialGridMap::isInBounds,
			"Test if the given local indices is within bounds")
		.def_property_readonly("size", &ugr::mapping::GeospatialGridMap::getSize, "Return the lengths of the grid")
		.def_property_readonly("layers", &ugr::mapping::GeospatialGridMap::getLayers, "Return the layers in the map")
		.def("eval", &ugr::mapping::GeospatialGridMap::eval, "Evaluate the grid map at a given position");

	////////////////////////////////////////////////////////////////
	// UGR Aircraft Model
	////////////////////////////////////////////////////////////////
	py::class_<ugr::risk::AircraftStateModel>(topModule, "AircraftStateModel")
		.def(py::init<>())
		.def_readwrite("position", &ugr::risk::AircraftStateModel::position, "Aircraft Position")
		.def_readwrite("velocity", &ugr::risk::AircraftStateModel::velocity, "Aircraft Velocity");

	py::class_<PyAircraftModel>(topModule, "AircraftModel")
		.def(py::init<double, double, double, double>())
		.def_readonly("mass", &PyAircraftModel::mass, "Aircraft Mass")
		.def_readonly("length", &PyAircraftModel::length, "Aircraft Length")
		.def_readonly("width", &PyAircraftModel::width, "Aircraft Width")
		.def_readonly("failureProb", &PyAircraftModel::failureProb, "Aircraft Failure Probability Per Flight Hour")
		.def_readwrite("state", &PyAircraftModel::state, "Aircraft State")
		.def("addGlideDescentModel", &PyAircraftModel::addGlideDescentModel, "Add glide descent model")
		.def("addBallisticDescentModel", &PyAircraftModel::addBallisticDescentModel, "Add ballistic descent model")
		.def("addParachuteDescentModel", &PyAircraftModel::addParachuteDescentModel, "Add parachute descent model")
		.def_property_readonly("descentNames",
			&PyAircraftModel::getDescentNames,
			"Get the names of the descent models");

	////////////////////////////////////////////////////////////////
	// UGR Population Mapping
	////////////////////////////////////////////////////////////////
	py::class_<ugr::mapping::PopulationMap, ugr::mapping::GeospatialGridMap>(topModule, "PopulationMap")
		.def(py::init<std::array<float, 4>, float>())
		.def(py::init<const std::array<float, 4>, int>())
		.def("eval", &ugr::mapping::PopulationMap::eval, "Evaluate the population map");

	py::class_<ugr::mapping::TemporalPopulationMap, ugr::mapping::PopulationMap>(topModule, "TemporalPopulationMap")
		.def(py::init<std::array<float, 4>, float>())
		.def("setHourOfDay", &ugr::mapping::TemporalPopulationMap::setHourOfDay,
			"Set the Hour of Day for the underlying population model");

	////////////////////////////////////////////////////////////////
	// UGR Auxiliary Maps
	////////////////////////////////////////////////////////////////

	py::class_<ugr::risk::ObstacleMap, ugr::mapping::GeospatialGridMap>(topModule, "ObstacleMap")
		.def(py::init<std::array<float, 4>, float>())
		.def("addOSMLayer", &ugr::risk::ObstacleMap::addOSMLayer, "Add an OSM layer to the map as an obstacle")
		.def("addBuildingHeights", &ugr::risk::ObstacleMap::addBuildingHeights, "Add OSM building heights to the map");

	py::class_<ugr::risk::WeatherMap, ugr::mapping::GeospatialGridMap>(topModule, "WeatherMap")
		.def(py::init<std::array<float, 4>, float>())
		.def("addConstantWind", &ugr::risk::WeatherMap::addConstantWind, "Add a constant wind to the map");

	////////////////////////////////////////////////////////////////
	// UR Voxel Risk Grids
	////////////////////////////////////////////////////////////////
	py::class_<ur::GroundRiskVoxelGrid, ur::VoxelGrid>(topModule, "FullGroundRiskVoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>,
					  ur::FPScalar,
					  ur::FPScalar,
					  ugr::mapping::PopulationMap*,
					  PyAircraftModel*,
					  ugr::risk::ObstacleMap*,
					  ugr::risk::WeatherMap*>())
		.def("eval", &PyGroundRiskVoxelGrid::eval, "Evaluate the ground risk values of the voxel grid.");

	py::class_<PyGroundRiskVoxelGrid, ur::VoxelGrid>(topModule, "GroundRiskVoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar, PyAircraftModel*>())
		.def("getZLayer",
			&PyGroundRiskVoxelGrid::getZLayer,
			py::return_value_policy::copy,
			"Extract a slice of the risk tensor at the given z Index.")
		.def("eval", &PyGroundRiskVoxelGrid::eval, "Evaluate the ground risk values of the voxel grid.");

	py::class_<PyIncrementalGroundRiskVoxelGrid, ur::VoxelGrid>(topModule, "IncrementalGroundRiskVoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar, PyAircraftModel*>(),
			py::keep_alive<1, 2>())
		.def("getIndexPointStrikeProbability",
			&PyIncrementalGroundRiskVoxelGrid::getIndexPointStrikeProbability,
			"Get the strike risk at a given index")
		.def("getIndexPointFatalityProbability",
			&PyIncrementalGroundRiskVoxelGrid::getIndexPointFatalityProbability,
			"Get the fatality risk at a given index")
		.def("getPositionPointStrikeProbability",
			&PyIncrementalGroundRiskVoxelGrid::getPositionPointStrikeProbability,
			"Get the strike risk at a given position")
		.def("getPositionPointFatalityProbability",
			&PyIncrementalGroundRiskVoxelGrid::getPositionPointFatalityProbability,
			"Get the fatality risk at a given position");

	py::class_<PyAirRiskVoxelGrid, ur::VoxelGrid>(topModule, "AirRiskVoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar, const std::string&, const
		std::string&>())
		.def("eval", &PyAirRiskVoxelGrid::eval, "Evaluate the air risk values of the voxel grid.");

	py::class_<PyRiskVoxelGrid, ur::VoxelGrid>(topModule, "RiskVoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>,
					  ur::FPScalar,
					  ur::FPScalar,
					  const std::string&,
					  const std::string&,
					  PyAircraftModel*>())
		.def("eval", &PyRiskVoxelGrid::eval, "Evaluate the combined risk values of the voxel grid.");
}
