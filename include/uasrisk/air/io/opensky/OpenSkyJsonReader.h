#ifndef READERWRITEROPENSKYJSON_H
#define READERWRITEROPENSKYJSON_H

#include "OpenSkyReader.h"

namespace ur
{
    namespace io
    {
        class OpenSkyJsonReader final : public OpenSkyReader
        {
        public:
            std::vector<ur::LineString> read(std::istream& fin) const override;
        };
    }
}
#endif // READERWRITEROPENSKYJSON_H
