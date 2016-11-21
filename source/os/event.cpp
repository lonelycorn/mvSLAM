#include <os/time.hpp>
#include <os/event.hpp>
#include <cassert>
#include <sys/time.h>

using namespace mvSLAM;

Event::Event()
{
    int result;
    pthread_condattr_t attr;
    result = pthread_condattr_init(&attr);
    assert(result == 0);

    result = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    assert(result == 0);

    result = pthread_cond_init(&_cond, &attr);
    assert(result == 0);

    result = pthread_condattr_destroy(&attr);
    assert(result == 0);
}

Event::~Event()
{
    int result = pthread_cond_destroy(&_cond);
    assert(result == 0);
}

void
Event::trigger_all()
{
    int result = pthread_cond_broadcast(&_cond);
    assert(result == 0);
}

void
Event::wait(Mutex &m)
{
    int result = pthread_cond_wait(&_cond, &m._mutex);
    assert(result == 0);
}
bool
Event::wait_timeout(Mutex &m, timestamp_ms_t timeout_ms)
{
    int result;
    struct timespec timeout;

    result = clock_gettime(CLOCK_MONOTONIC, &timeout);
    assert(result == 0);

    timeout.tv_sec += timeout_ms / MS_PER_S;
    timeout.tv_nsec += (timeout_ms % MS_PER_S) * US_PER_MS;

    long t = timeout.tv_nsec / NS_PER_S;
    timeout.tv_sec += t;
    timeout.tv_nsec %= NS_PER_S;
    assert(timeout.tv_sec >= 0);
    assert((timeout.tv_nsec >= 0) && (timeout.tv_nsec < NS_PER_S));
    
    result = pthread_cond_timedwait(&_cond, &m._mutex, &timeout);
    return (result == 0);
}

