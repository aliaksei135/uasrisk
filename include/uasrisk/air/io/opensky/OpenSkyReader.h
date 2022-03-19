#ifndef READERWRITEROPENSKY_H
#define READERWRITEROPENSKY_H

#include "uasrisk/environment/TypeDefs.h"

#include <unordered_map>
#include <vector>


#if defined(__SSE4_2__)
#define RAPIDJSON_SSE42
#elif  defined(__SSE2__)
#define RAPIDJSON_SSE2
#elif defined(__ARM__NEON__)
#define RAPIDJSON_NEON
#endif

namespace ur
{
    namespace io
    {
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

        using Flight = std::vector<StateVector>;
        using FlightMap = std::unordered_map<std::string, Flight>;
        using Traffic = std::vector<Flight>;


        class OpenSkyReader
        {
        public:
            virtual ~OpenSkyReader() = default;
            std::vector<LineString> readFile(const std::string& file) const;
            virtual std::vector<LineString> read(std::istream& stream) const = 0;
        protected:
            std::vector<LineString> constructTracks(const FlightMap& flightMap) const;
        };
    }
}
#endif // READERWRITEROPENSKY_H
