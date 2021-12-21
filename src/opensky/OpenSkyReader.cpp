#include "OpenSkyReader.h"
#include <vector>
#include <omp.h>
#include <fstream>
#include <istream>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include <boost/geometry/geometry.hpp>

using namespace boost::accumulators;

std::vector<ur::LineString> ur::io::OpenSkyReader::readFile(const std::string& file) const
{
	std::ifstream fileStream(file.c_str(), std::ios_base::in | std::ios_base::binary);
	auto open = fileStream.is_open();
	return read(fileStream);
}


std::vector<ur::LineString> ur::io::OpenSkyReader::constructTracks(const FlightMap& flightMap) const
{
	const auto& numFlights = flightMap.size();
	std::vector<Flight> flightVec(numFlights);
	std::transform(flightMap.begin(), flightMap.end(), std::back_inserter(flightVec),
	               [](const FlightMap::value_type& val) { return val.second; });

	std::vector<ur::LineString> trajectoriesOut;
	trajectoriesOut.reserve(numFlights);

	// #pragma omp parallel for shared(mg)
	// for (int f = 0; f < numFlights; ++f)

	for (const auto& flight : flightMap)
	{
		// auto flightSVs = flightVec[f];
		auto flightSVs = flight.second;
		std::sort(flightSVs.begin(), flightSVs.end());
		const auto flightSize = flightSVs.size();

		ur::LineString trajLineString;

		if (flightSize > 3)
		{
			// Filter out spurious altitude spikes by keeping a rolling average and tolerancing incoming point altitudes
			accumulator_set<double, features<tag::rolling_mean>> acc(tag::rolling_window::window_size = 3);
			acc(flightSVs[0].geoaltitude);

			for (unsigned int i = 0; i < flightSize; ++i)
			{
				const auto& sv = flightSVs[i];
				const auto& geoAlt = sv.geoaltitude;
				auto rmVal = rolling_mean(acc);
				if (abs(rmVal - geoAlt) > 200) continue;
				acc(geoAlt);
				trajLineString.emplace_back(sv.lon, sv.lat, geoAlt);
			}
		}
		else
		{
			for (int i = 0; i < flightSize; ++i)
			{
				const auto& sv = flightSVs[i];
				trajLineString.emplace_back(sv.lon, sv.lat, sv.geoaltitude);
			}
		}
		trajectoriesOut.emplace_back(trajLineString);
	}
	return trajectoriesOut;
}
