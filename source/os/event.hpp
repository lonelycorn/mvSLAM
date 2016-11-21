#pragma once
#include <pthread.h>
#include <cassert>
#include <os/mutex.hpp>
#include <os/time.hpp>

namespace mvSLAM
{
class Event
{
public:
    Event();
    
    ~Event();

    void trigger_all();

    void wait(Mutex &m);

    /**
     * @return true if the event was triggered; false if timed out.
     */
    bool wait_timeout(Mutex &m, timestamp_ms_t timeout_ms);

private:
    pthread_cond_t _cond;
};

}
