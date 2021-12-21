#ifndef READEROPENAIP_H
#define READEROPENAIP_H
#include <vector>
#include <fstream>
#include <map>
#include <pugixml.hpp>


#include <boost/algorithm/string.hpp>

namespace ur
{
	class ExtrudedPolygon;

	namespace io
	{
		class OpenAIPReader
		{
		public:
			explicit OpenAIPReader() = default;

			std::vector<ur::ExtrudedPolygon> readFile(const std::string& file) const
			{
				std::ifstream fileStream(file.c_str(), std::ios_base::in | std::ios_base::binary);
				return read(fileStream);
			}

			std::vector<ur::ExtrudedPolygon> read(std::istream& fin) const;

		protected:
			/**
			 * @brief Convert an altitude relative to a reference datum to an absolute altitude
			 * @param ref reference datum standardised name from OpenAIP spec
			 * @param val the value itself
			 * @return an absolute altitude (amsl) in metres
			*/
			static double referenceAlt2AbsAlt(const std::string& ref, const std::string& units, double val,
			                                  double pressureHpa = 1013.25);

			// const std::map<std::string, osgEarth::Color> categoryColourMap = {
			// 	{"A", {1, 0, 0.6, 0.314}},
			// 	{"B", {0.6, 0.2, 0.8, 0.317}},
			// 	{"C", {0.6, 0.2, 0.8, 0.317}},
			// 	{"D", {0, 0.6, 1, 0.314}},
			// 	{"E", {0, 0.6, 0.2, 0.314}},
			// 	{"F", {1, 0.6, 1, 0.314}},
			// 	{"G", {1, 0.6, 1, 0.314}},
			// 	{"GLIDING", {0.886, 0.886, 0, 0.314}},
			// 	{"WAVE", {0.886, 0.886, 0, 0.314}},
			// 	{"RESTRICTED", {1, 0, 0, 0.314}},
			// 	{"DANGER", {0.055, 0.18, 0.957, 0.314}},
			// 	{"PROHIBITED", {0.5, 0, 1, 0.314}},
			// 	{"TMA", {1, 0, 0, 0.314}},
			// 	{"TMZ", {0.671, 0.82, 0.973, 0.314}},
			// 	{"RMZ", {0.216, 0.44, 0.522, 0.314}},
			// 	{"FIR", {0.5, 0.5, 0.5, 0.314}},
			// 	{"UIR", {0.5, 0.5, 0.5, 0.314}},
			// 	{"OTH", {0.5, 0.5, 0.5, 0.314}},
			// };
		};
	}
}
#endif // READEROPENAIP_H
