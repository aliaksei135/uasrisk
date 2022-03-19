// Disable atox inspections, as we want it to default to 0 on error instead of throwing
// ReSharper disable CppClangTidyCertErr34C
#ifndef READERWRITEROPENSKYCSV_H
#define READERWRITEROPENSKYCSV_H
#include "OpenSkyReader.h"
#include <vector>


namespace ur
{
    namespace io
    {
        class OpenSkyCsvReader final : public OpenSkyReader
        {
        public:
            ~OpenSkyCsvReader() override = default;

            std::vector<LineString> read(std::istream& fin) const override;
        };
    }
}
#endif // READERWRITEROPENSKYCSV_H
