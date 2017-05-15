#include <os/time.hpp>
#include <cassert>
#include <time.h>

#include <unistd.h>

namespace mvSLAM
{

class Timer
{
public:
    Timer()
    {
        clock_gettime(CLOCK_MONOTONIC, &_initial_time);
    }

    ~Timer()
    {
    }

    timestamp_us_t get_time_us() const
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        time_t sec_delta = now.tv_sec - _initial_time.tv_sec;
        long nsec_delta = now.tv_nsec - _initial_time.tv_nsec;        
        if (nsec_delta < 0)
        {
            nsec_delta += NS_PER_S;
            --sec_delta;
        }
        assert(sec_delta >= 0); // don't go back in time please...
        return (timestamp_us_t)(sec_delta * US_PER_S + nsec_delta / NS_PER_US);
    }

private:    
    struct timespec _initial_time;
};

static Timer timer;

timestamp_ms_t get_time_ms()
{
    return timer.get_time_us() / US_PER_MS;
}

timestamp_us_t get_time_us()
{
    return timer.get_time_us();
}

bool sleep_ms(timestamp_ms_t delta)
{
    return (usleep(delta * US_PER_MS) == 0);
}

}
