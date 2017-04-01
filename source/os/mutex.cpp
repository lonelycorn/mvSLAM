#include <pthread.h>
#include <os/mutex.hpp>

using namespace mvSLAM;

Mutex::Mutex()
{
    int result;
    pthread_mutexattr_t attr;
    result = pthread_mutexattr_init(&attr);
    assert(result == 0);

    // mutex protocol: priority inheritance
    // http://pubs.opengroup.org/onlinepubs/007908775/xsh/pthread_mutexattr_getprotocol.html
    result = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    assert(result == 0);

    // mutex pshared: private per process
    // http://pubs.opengroup.org/onlinepubs/007908775/xsh/pthread_mutexattr_getpshared.html
    result = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_PRIVATE);
    assert(result == 0);

    // mutex prioceiling: using default value now.
    // http://pubs.opengroup.org/onlinepubs/007908775/xsh/pthread_mutexattr_getprioceiling.html

    // mutex type: recursive
    // FIXME: this will hide some problems. but let's use this to speed up development
    // http://pubs.opengroup.org/onlinepubs/007908775/xsh/pthread_mutexattr_gettype.html
    result = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    assert(result == 0);

    result = pthread_mutex_init(&_mutex, &attr);
    assert(result == 0);

    result = pthread_mutexattr_destroy(&attr);
    assert(result == 0);
}

Mutex::~Mutex()
{
    int result = pthread_mutex_destroy(&_mutex);
    assert(result == 0);
}

void
Mutex::lock()
{
    int result = pthread_mutex_lock(&_mutex);
    assert(result == 0);
}

void
Mutex::unlock()
{
    int result = pthread_mutex_unlock(&_mutex);
    assert(result == 0);
}

