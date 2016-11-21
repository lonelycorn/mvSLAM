#pragma once
#include <cassert>

namespace mvSLAM
{
class Event; // forward declaration

class Mutex
{
public:
    Mutex();

    ~Mutex();

    Mutex(const Mutex &) = delete;
    Mutex(const Mutex &&) = delete;
    Mutex &operator=(const Mutex &) = delete;
    Mutex &operator=(const Mutex &&) = delete;

    void lock();

    void unlock();

private:
    friend class Event;
    pthread_mutex_t _mutex;
};

/** RAII style lock
 */
class Lock
{
public:
    /// Ctor. Locks the provided mutex.
    Lock(Mutex &m): _mutex(m)
    {
        _mutex.lock();
    }
    /// Dtor. Unlocks the provided mutex.
    ~Lock()
    {
        _mutex.unlock();
    }
    Lock(const Lock &) = delete;
    Lock(const Lock &&) = delete;
    Lock &operator=(const Lock &) = delete;
    Lock &operator=(const Lock &&) = delete;
private:
    Mutex &_mutex;
};
}
