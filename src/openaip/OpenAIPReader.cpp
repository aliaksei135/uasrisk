#include <istream>

#include "uasrisk/environment/TypeDefs.h"
#include "uasrisk/air/io/openaip/OpenAIPReader.h"

#include <vector>


#define FEET2METRE 0.3048

using namespace pugi;
using namespace boost::algorithm;

std::vector<ur::ExtrudedPolygon> ur::io::OpenAIPReader::readFile(const std::string& file) const
{
	std::ifstream fileStream(file.c_str(), std::ios_base::in | std::ios_base::binary);
	auto open = fileStream.is_open();
	if (!open)
	{
		throw std::ios_base::failure("OpenAIP data file not found");
	}
	return read(fileStream);
}

std::vector<ur::ExtrudedPolygon> ur::io::OpenAIPReader::read(std::istream& fin) const
{
	std::vector<ur::ExtrudedPolygon> polysOut;
	// Load the stream into a buffer
	xml_document doc;
	auto res = doc.load(fin);

	const auto airspaceList = doc.child("OPENAIP").child("AIRSPACES");
	for (const auto& airspace : airspaceList)
	{
		const auto& name = airspace.child("NAME").text();
		const auto& category = airspace.attribute("CATEGORY").as_string();

		const auto& ceilingNode = airspace.child("ALTLIMIT_TOP").child("ALT");
		const std::string& ceilingUnit = ceilingNode.attribute("UNIT").as_string();
		const std::string& ceilingRef = airspace.child("ALTLIMIT_TOP").attribute("REFERENCE").as_string();
		const auto& ceilingAlt = referenceAlt2AbsAlt(ceilingRef, ceilingUnit, ceilingNode.text().as_double());

		const auto& floorNode = airspace.child("ALTLIMIT_BOTTOM").child("ALT");
		const std::string& floorUnit = floorNode.attribute("UNIT").as_string();
		const std::string& floorRef = airspace.child("ALTLIMIT_BOTTOM").attribute("REFERENCE").as_string();
		const auto& floorAlt = referenceAlt2AbsAlt(floorRef, floorUnit, floorNode.text().as_double());

		const std::string& geometry = airspace.child("GEOMETRY").child("POLYGON").text().as_string();

		ur::Polygon poly;

		// Split out coord pairs by comma
		std::vector<std::string> tokens;
		split(tokens, geometry, boost::is_any_of(","));

		std::vector<std::string> coordTokens;
		coordTokens.reserve(3);
		for (auto& token : tokens)
		{
			// Split coord values out
			coordTokens.clear();
			boost::trim(token);
			split(coordTokens, token, boost::is_any_of(" "));
			poly.outer().emplace_back(atof(coordTokens[0].c_str()), atof(coordTokens[1].c_str()), floorAlt);
		}

		polysOut.emplace_back(poly, floorAlt, ceilingAlt);
	}
	return polysOut;
}

double ur::io::OpenAIPReader::referenceAlt2AbsAlt(const std::string& ref, const std::string& units, const double val,
                                                  double pressureHpa)
{
	double out = 0;

	if (units == "FL") //Flight Level
	{
		out = val * 100;
	}
	else if (units == "F") //Feet
	{
		out = val;
	}
	// At this point `out` is in feet


	if (ref == "GND")
	{
	}
	else if (ref == "MSL")
	{
	}
	else if (ref == "STD")
	{
	}

	return out * FEET2METRE;
}
