#include <cstdio>
namespace mvSLAM
{
#ifdef VERBOSE_LOGGING
    #define __WRITE_TRACE(out) std::fprintf(out, "[%s:%s()] ", __FILE__, __func__);
#else
    #define __WRITE_TRACE(out)
#endif

#define __MVSLAM_WRITE_TO(out, ...) \
{ \
    __WRITE_TRACE(out); \
    std::fprintf(out, __VA_ARGS__);\
    std::fprintf(out, "\n"); \
}

// describe fatal error
#define MVSLAM_ERROR(...)   __MVSLAM_WRITE_TO(stderr, __VA_ARGS__)
// describe what has happend
#define MVSLAM_LOG(...)     __MVSLAM_WRITE_TO(stdout, __VA_ARGS__)
// expain why something happened.
#define MVSLAM_DEBUG(...)   __MVSLAM_WRITE_TO(stdout, __VA_ARGS__)


/*
#define ENABLE_LOGGING(module_name) \
    #define ##module_name_ERROR(...) MVSLAM_ERROR(__VA_ARGS__)
    #define ##module_name_LOG(...)  MVSLAM_LOG(__VA_ARGS__)
    #define ##module_name_DEBUG(...) MVSLAM_DEBUG(__VA_ARGS__)
*/

}
