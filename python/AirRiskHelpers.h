#include "uasrisk/air/AirRiskVoxelGrid.h"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/pattern_formatter.h"

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
