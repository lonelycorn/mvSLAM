#include <base/debug.hpp>
namespace mvSLAM
{
LoggingLevel Logging::m_level = LoggingLevel::DEBUG;
std::ostream * Logging::m_debug_stream = &std::clog;
std::ostream * Logging::m_info_stream = &std::clog;
std::ostream * Logging::m_error_stream = &std::cerr;
}
