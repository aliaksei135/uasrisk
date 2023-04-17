#include "uasgroundrisk/risk_analysis/RiskMap.h"
#include "uasgroundrisk/risk_analysis/IncrementalRiskMap.h"
#include "uasgroundrisk/risk_analysis/obstacles/ObstacleMap.h"
#include "uasgroundrisk/risk_analysis/weather/WeatherMap.h"
#include "uasgroundrisk/map_gen/TemporalPopulationMap.h"

#include "uasrisk/ground/GroundRiskVoxelGrid.h"
#include "uasrisk/ground/IncrementalGroundRiskVoxelGrid.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/pattern_formatter.h"

class PyAircraftModel : public ugr::risk::AircraftModel
{
 public:
	PyAircraftModel(const double mass, const double width, const double length, const double failureProb)
		: AircraftModel(mass, width, length, failureProb)
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

	std::vector<std::string> getDescentNames()
	{
		std::vector<std::string> names;
		for (auto& d : descents)
		{
			names.push_back(d->name);
		}
		return names;
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

	using MatrixSliceType = Eigen::Matrix<ur::FPScalar, Eigen::Dynamic, Eigen::Dynamic>;

	MatrixSliceType getZLayer(const std::string& layerName, const int zIndex) const
	{
		const ur::Matrix tensorLayer = get(layerName);

		const Eigen::Tensor<ur::FPScalar, 2> tensorChipR2 = tensorLayer.chip(zIndex, 2);

		MatrixSliceType out = Eigen::Map<const MatrixSliceType>(tensorChipR2.data(), sizeX, sizeY);
		return out;
	}

	using ur::GroundRiskVoxelGrid::eval;
};

class PyIncrementalGroundRiskVoxelGrid : public ur::IncrementalGroundRiskVoxelGrid
{
 public:
	PyIncrementalGroundRiskVoxelGrid(const std::array<ur::FPScalar, 6>& bounds, ur::FPScalar xyRes,
		ur::FPScalar zRes, ugr::risk::AircraftModel* aircraftModel) : IncrementalGroundRiskVoxelGrid(
		bounds, xyRes, zRes,
		new ugr::mapping::TemporalPopulationMap({ bounds[0], bounds[1], bounds[3], bounds[4] }, xyRes),
		aircraftModel,
		new ugr::risk::ObstacleMap({ bounds[0], bounds[1], bounds[3], bounds[4] }, xyRes),
		new ugr::risk::WeatherMap({ bounds[0], bounds[1], bounds[3], bounds[4] }, xyRes))
	{
	}

	using ur::VoxelGrid::world2Local;
	using ur::VoxelGrid::local2World;
	using ur::VoxelGrid::getSize;
	using ur::VoxelGrid::isInBounds;

	double getIndexPointStrikeProbability(const ugr::gridmap::Index& index,
		const double altitude,
		const int heading)
	{
		return IncrementalGroundRiskVoxelGrid::getIndexPointStrikeProbability(index, altitude, heading);
	}

// Pybind needs a bunch of lambda functions as it does not support reflection, so would need even deeper bindings into ugr

	double getIndexPointFatalityProbability(const ugr::gridmap::Index& index,
		const double altitude,
		const int heading)
	{
		return IncrementalGroundRiskVoxelGrid::getIndexPointFatalityProbability(index, altitude, heading);
	}

	double getPositionPointStrikeProbability(const ugr::gridmap::Position3& position,
		const int heading)
	{
		return IncrementalGroundRiskVoxelGrid::getPositionPointStrikeProbability(position, heading);
	}

	double getPositionPointFatalityProbability(const ugr::gridmap::Position3& position,
		const int heading)
	{
		return IncrementalGroundRiskVoxelGrid::getPositionPointFatalityProbability(position, heading);
	}
};
