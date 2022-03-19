#include "uasrisk/air/io/opensky/OpenSkyJsonReader.h"

#include <unordered_map>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

using namespace rapidjson;

std::vector<ur::LineString> ur::io::OpenSkyJsonReader::read(std::istream& fin) const
{
    FlightMap flightMap;

    const std::function<bool(StateVector)> boundingPredicate = [](const StateVector& sv) -> bool
    {
        const auto lat = sv.lat;
        const auto lon = sv.lon;
        // return lat > 49.7 && lat < 59.7 && lon < 2.1 && lon > -7.2; // && sv.geoaltitude < 500;
        return lat > 50.8 && lat < 51.1 && lon < -1.15 && lon > -1.6; // && sv.geoaltitude < 500;
    };

    IStreamWrapper jin(fin);
    Document jsonDoc;
    jsonDoc.ParseStream(jin);

    for (auto& sv : jsonDoc.GetArray())
    {
        StateVector stateVector;
        std::string key;
        if (sv.HasMember("time") && sv["time"].IsUint())
        {
            stateVector.time = sv["time"].GetUint();
        }
        else { continue; }
        // if (sv.HasMember("icao24") && sv["icao24"].IsString())
        // {
        // 	key = sv["icao24"].GetString();
        // 	stateVector.icao24 = key;
        // }
        // else { continue; }
        if (sv.HasMember("lat") && sv["lat"].IsDouble())
        {
            stateVector.lat = sv["lat"].GetDouble();
        }
        else { continue; }
        if (sv.HasMember("lon") && sv["lon"].IsDouble())
        {
            stateVector.lon = sv["lon"].GetDouble();
        }
        else { continue; }
        if (sv.HasMember("velocity") && sv["velocity"].IsDouble())
            stateVector.velocity = sv["velocity"].GetDouble();
        if (sv.HasMember("heading") && sv["heading"].IsDouble())
            stateVector.heading = sv["heading"].GetDouble();
        if (sv.HasMember("vertrate") && sv["vertrate"].IsDouble())
            stateVector.vertrate = sv["vertrate"].GetDouble();
        // if (sv.HasMember("callsign") && sv["callsign"].IsString())
        // 	stateVector.callsign = sv["callsign"].GetString();
        if (sv.HasMember("onground") && sv["onground"].IsBool())
            stateVector.onground = sv["onground"].GetBool();
        if (sv.HasMember("alert") && sv["alert"].IsBool())
            stateVector.alert = sv["alert"].GetBool();
        if (sv.HasMember("spi") && sv["spi"].IsBool())
            stateVector.spi = sv["spi"].GetBool();
        // if (sv.HasMember("squawk") && sv["squawk"].IsString())
        // 	stateVector.squawk = sv["squawk"].GetString();
        if (sv.HasMember("baroaltitude") && sv["baroaltitude"].IsDouble())
            stateVector.baroaltitude = sv["baroaltitude"].GetDouble();
        if (stateVector.onground)
        {
            stateVector.geoaltitude = 0;
        }
        else
        {
            if (sv.HasMember("geoaltitude") && sv["geoaltitude"].IsDouble())
                stateVector.geoaltitude = sv["geoaltitude"].GetDouble();
        }
        if (sv.HasMember("lastposupdate") && sv["lastposupdate"].IsDouble())
            stateVector.lastposupdate = static_cast<unsigned long>(sv["lastposupdate"].GetDouble());
        if (sv.HasMember("lastcontact") && sv["lastcontact"].IsDouble())
            stateVector.lastcontact = static_cast<unsigned long>(sv["lastcontact"].GetDouble());

        if (!boundingPredicate(stateVector)) continue;

        flightMap[key].push_back(stateVector);
    }
    jsonDoc.Clear();

    return constructTracks(flightMap);
}
