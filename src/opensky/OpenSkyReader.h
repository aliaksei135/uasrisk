#ifndef READERWRITEROPENSKY_H
#define READERWRITEROPENSKY_H

#include "../TypeDefs.h"

#include <unordered_map>
#include <vector>


#if defined(__SSE4_2__)
#define RAPIDJSON_SSE42
#elif  defined(__SSE2__)
#define RAPIDJSON_SSE2
#elif defined(__ARM__NEON__)
#define RAPIDJSON_NEON
#endif

struct StateVector
{
	unsigned long time;
	char* icao24;
	double lat;
	double lon;
	double velocity;
	double heading;
	double vertrate;
	char* callsign;
	bool onground;
	bool alert;
	bool spi;
	char* squawk;
	double baroaltitude;
	double geoaltitude;
	unsigned long lastposupdate;
	unsigned long lastcontact;

	friend bool operator<(const StateVector& lhs, const StateVector& rhs)
	{
		return lhs.time < rhs.time;
	}

	friend bool operator<=(const StateVector& lhs, const StateVector& rhs)
	{
		return !(rhs < lhs);
	}

	friend bool operator>(const StateVector& lhs, const StateVector& rhs)
	{
		return rhs < lhs;
	}

	friend bool operator>=(const StateVector& lhs, const StateVector& rhs)
	{
		return !(lhs < rhs);
	}
};

typedef std::vector<StateVector> Flight;
typedef std::unordered_map<std::string, Flight> FlightMap;
typedef std::vector<Flight> Traffic;


class OpenSkyReader
{
public:
	virtual ~OpenSkyReader() = default;
	std::vector<ur::LineString> readFile(const std::string& file) const;
	virtual std::vector<ur::LineString> read(std::istream& stream) const = 0;
protected:
	std::vector<ur::LineString> constructTracks(const FlightMap& flightMap) const;
};
#endif // READERWRITEROPENSKY_H
