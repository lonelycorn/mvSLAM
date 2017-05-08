#pragma once
#include <cstdint>

namespace mvSLAM
{
enum class ApplicationErrorCode : int
{
    NONE = 0,           // eveything ok
    INVALID_ARGS = 1,   // invalid arguments
    BAD_IO = 2,         // IO exceptions
    BAD_DATA = 3,       // input data is bad
    HARDWARE_ERROR = 4, // hardware issues
    UNKNOWN = 0xff,     // unclassified error
};
}
