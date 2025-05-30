#include "uasrisk/air/io/opensky/OpenSkyCsvReader.h"

#include <unordered_map>
#include <functional>
#include <csv.h>

std::vector<ur::LineString> ur::io::OpenSkyCsvReader::read(std::istream& fin) const
{
    using CsvParser = ::io::CSVReader<16, ::io::trim_chars<' ', '\t'>
                                      ,
                                      ::io::no_quote_escape<','>
                                      ,
                                      ::io::ignore_overflow
                                      ,
                                      ::io::no_comment
    >;

    FlightMap flightMap;

    const std::function<bool(StateVector)> boundingPredicate = [](const StateVector& sv) -> bool
    {
        const auto lat = sv.lat;
        const auto lon = sv.lon;

        //TODO: Work out a thread safe version of this
        const std::time_t t = sv.time;
        const auto hour = gmtime(&t)->tm_hour;

        return lat > 49.7 && lat < 59.7 && lon < 2.1 && lon > -7.2 && sv.geoaltitude < 1524; //5000ft
    };

    CsvParser reader("", fin);
    reader.read_header(::io::ignore_extra_column,
                       "time",
                       "icao24",
                       "lat",
                       "lon",
                       "velocity",
                       "heading",
                       "vertrate",
                       "callsign",
                       "onground",
                       "alert",
                       "spi",
                       "squawk",
                       "baroaltitude",
                       "geoaltitude",
                       "lastposupdate",
                       "lastcontact");

    // Declare corresponding field to StateVector that will be repeatedly rewritten into by read_row
    long time;
    char* icao24;
    double lat;
    double lon;
    double velocity;
    double heading;
    double vertrate;
    char* callsign;
    char* onground;
    char* alert;
    char* spi;
    char* squawk;
    double baroaltitude;
    double geoaltitude;
    double lastposupdate;
    double lastcontact;

    while (reader.read_row(time, icao24, lat, lon, velocity, heading, vertrate, callsign, onground, alert,
                           spi,
                           squawk, baroaltitude, geoaltitude, lastposupdate, lastcontact))
    {
        StateVector stateVector;

        stateVector.time = time;
        stateVector.icao24 = icao24;
        stateVector.lat = lat;
        stateVector.lon = lon;
        stateVector.velocity = velocity;
        stateVector.heading = heading;
        stateVector.vertrate = vertrate;
        stateVector.callsign = callsign;
        stateVector.onground = strcmp(onground, "true") == 0;
        stateVector.alert = strcmp(alert, "true") == 0;
        stateVector.spi = strcmp(spi, "true") == 0;
        stateVector.squawk = squawk;
        stateVector.baroaltitude = baroaltitude;
        stateVector.geoaltitude = geoaltitude;
        stateVector.lastposupdate = lastposupdate;
        stateVector.lastcontact = lastcontact;

        if (!boundingPredicate(stateVector)) continue;
        flightMap[icao24].emplace_back(stateVector);
    }

    return constructTracks(flightMap);
}
