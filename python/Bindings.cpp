#include <iostream>

#include "uasrisk/environment/VoxelGrid.h"
#include "uasrisk/ground/GroundRiskVoxelGrid.h"
#include "uasgroundrisk/risk_analysis/RiskMap.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "pybind11_eigen_tensor.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/pattern_formatter.h"

#include "uasgroundrisk/map_gen/TemporalPopulationMap.h"
#include "uasrisk/RiskVoxelGrid.h"
#include "uasrisk/air/AirRiskVoxelGrid.h"

namespace py = pybind11;
template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

class PyAircraftModel : public ugr::risk::AircraftModel
{
public:
	PyAircraftModel(const double mass, const double width, const double length)
		: AircraftModel(mass, width, length)
	{
	}

	void addGlideDescentModel(double glideAirspeed, double glideRatio)
	{
		addDescentModel<ugr::risk::GlideDescentModel>(glideAirspeed, glideRatio);
	}

	void addBallisticDescentModel(double ballisticFrontalArea, double ballisticDragCoeff)
	{
		addDescentModel<ugr::risk::BallisticDescentModel>(ballisticFrontalArea, ballisticDragCoeff);
	}

	void addParachuteDescentModel(double parachuteDragCoeff, double parachuteArea, double parachuteDeployTime)
	{
		addDescentModel<ugr::risk::ParachuteDescentModel>(parachuteDragCoeff, parachuteArea, parachuteDeployTime);
	}
};

class PyGroundRiskVoxelGrid : public ur::GroundRiskVoxelGrid
{
public:
	PyGroundRiskVoxelGrid(const std::array<ur::FPScalar, 6>& bounds, ur::FPScalar xyRes,
		ur::FPScalar zRes, ugr::risk::AircraftModel* aircraftModel) : GroundRiskVoxelGrid(
			bounds, xyRes, zRes,
			new ugr::mapping::TemporalPopulationMap({ bounds[0], bounds[1], bounds[3], bounds[4] }, xyRes),
			aircraftModel,
			new ugr::risk::ObstacleMap({ bounds[0], bounds[1], bounds[3], bounds[4] }, xyRes),
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

	using ur::GroundRiskVoxelGrid::eval;
};

class PyAirRiskVoxelGrid : public ur::AirRiskVoxelGrid
{
public:
	PyAirRiskVoxelGrid(const std::array<ur::FPScalar, 6>& bounds, const ur::FPScalar xyRes, const ur::FPScalar zRes,
		const std::string& trajPath, const std::string& airspacePath)
		: AirRiskVoxelGrid(bounds, xyRes, zRes, trajPath, airspacePath)
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

	using ur::AirRiskVoxelGrid::eval;
};

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

	// auto voxelGridModule = topModule.def_submodule("environment");
	py::class_<ur::VoxelGrid>(topModule, "VoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar, const char*>())
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
		.def("get", overload_cast_<const std::string&>()(&ur::VoxelGrid::get, py::const_),
			"Return the entire tensor for a layer")
		.def("world2Local", overload_cast_<const ur::Position&>()(&ur::VoxelGrid::world2Local, py::const_),
			"Reproject world (EPSG:4326) coordinates to local indices")
		.def("world2Local",
			overload_cast_<ur::FPScalar, ur::FPScalar, ur::FPScalar>()(&ur::VoxelGrid::world2Local, py::const_),
			"Reproject world (EPSG:4326) coordinates to local indices")
		.def("isInBounds", &ur::VoxelGrid::isInBounds, "Test if the given local indices is within bounds")
		.def("writeToNetCDF", &ur::VoxelGrid::writeToNetCDF, "Write the risk layers to netCDF format for export.")
		.def_property_readonly("size", &ur::VoxelGrid::getSize, "Return the lengths of the grid")
		.def_property_readonly("layers", &ur::VoxelGrid::getLayers, "Return the layers in the map")
		.def("local2World", overload_cast_<const ur::Index&>()(&ur::VoxelGrid::local2World, py::const_),
			"Reproject local indices to world coordinates")
		.def("local2World", overload_cast_<int, int, int>()(&ur::VoxelGrid::local2World, py::const_),
			"Reproject local indices to world coordinates");

	py::class_<ugr::risk::AircraftStateModel>(topModule, "AircraftStateModel")
		.def(py::init<>())
		.def_readwrite("position", &ugr::risk::AircraftStateModel::position, "Aircraft Position")
		.def_readwrite("velocity", &ugr::risk::AircraftStateModel::velocity, "Aircraft Velocity");

	py::class_<PyAircraftModel>(topModule, "AircraftModel")
		.def(py::init<double, double, double>())
		.def_readonly("mass", &PyAircraftModel::mass, "Aircraft Mass")
		.def_readonly("length", &PyAircraftModel::length, "Aircraft Length")
		.def_readonly("width", &PyAircraftModel::width, "Aircraft Width")
		.def_readwrite("state", &PyAircraftModel::state, "Aircraft State")
		.def("addGlideDescentModel", &PyAircraftModel::addGlideDescentModel, "Add glide descent model")
		.def("addBallisticDescentModel", &PyAircraftModel::addBallisticDescentModel, "Add ballistic descent model")
		.def("addParachuteDescentModel", &PyAircraftModel::addParachuteDescentModel, "Add parachute descent model");

	py::class_<ugr::mapping::PopulationMap>(topModule, "PopulationMap")
		.def(py::init<const std::array<float, 4>, int>())
		.def("eval", &ugr::mapping::PopulationMap::eval, "Evaluate the population map");

	py::class_<ugr::mapping::TemporalPopulationMap, ugr::mapping::PopulationMap>(topModule, "TemporalPopulationMap")
		.def("setHourOfDay", &ugr::mapping::TemporalPopulationMap::setHourOfDay,
			"Set the Hour of Day for the underlying population model");

	py::class_<PyGroundRiskVoxelGrid, ur::VoxelGrid>(topModule, "GroundRiskVoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar, PyAircraftModel*>())
		.def("eval", &PyGroundRiskVoxelGrid::eval, "Evaluate the ground risk values of the voxel grid.");

	py::class_<PyAirRiskVoxelGrid, ur::VoxelGrid>(topModule, "AirRiskVoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar, const std::string&, const
			std::string&>())
		.def("eval", &PyAirRiskVoxelGrid::eval, "Evaluate the air risk values of the voxel grid.");

	py::class_<PyRiskVoxelGrid, ur::VoxelGrid>(topModule, "RiskVoxelGrid")
		.def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar, const std::string&, const std::string&, PyAircraftModel*>())
		.def("eval", &PyRiskVoxelGrid::eval, "Evaluate the combined risk values of the voxel grid.");
}
