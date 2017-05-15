#pragma once
#include <cstdint>

namespace mvSLAM
{

using timestamp_ms_t = uint32_t;
using timestamp_us_t = uint64_t;

#define MS_PER_S 1000

#define US_PER_MS 1000
#define US_PER_S (US_PER_MS * MS_PER_S)

#define NS_PER_US 1000
#define NS_PER_MS (NS_PER_US * US_PER_MS)
#define NS_PER_S (NS_PER_MS * MS_PER_S)

/// Get time elapsed since start up, in ms
timestamp_ms_t get_time_ms();

/// Get time elapsed since start up, in us
timestamp_us_t get_time_us();

/// Sleep
bool sleep_ms(timestamp_ms_t delta);

}

