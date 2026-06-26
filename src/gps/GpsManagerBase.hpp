#ifndef GPS_MANAGER_BASE_HPP
#define GPS_MANAGER_BASE_HPP

#include "NmeaReader.hpp"
#include "GpsUpdate.hpp"

#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

class GpsManagerBase {
public:
    virtual void InstallCallback(std::function<void(const GpsUpdate&)> callback) {};
    virtual void Start() {};
    virtual void Stop() {};
};

#endif