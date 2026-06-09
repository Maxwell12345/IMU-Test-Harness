#ifndef IMU_MANAGER_STATS_HPP
#define IMU_MANAGER_STATS_HPP
#pragma once

#include <atomic>

struct IMUManagerStats {
    std::atomic<uint64_t> imuAccepted;
    std::atomic<uint64_t> imuRejected;
    std::atomic<uint64_t> gpsAccepted;
    std::atomic<uint64_t> gpsRejected;
    std::atomic<uint64_t> dbEnqueueFailures;

    IMUManagerStats() : 
        imuAccepted(0),
        imuRejected(0),
        gpsAccepted(0),
        gpsRejected(0),
        dbEnqueueFailures(0)
    {}

    IMUManagerStats(const IMUManagerStats& other) {
        imuAccepted.store(other.imuAccepted.load());
        imuRejected.store(other.imuRejected.load());
        gpsAccepted.store(other.gpsAccepted.load());
        gpsRejected.store(other.gpsRejected.load());
        dbEnqueueFailures.store(other.dbEnqueueFailures.load());
    }

    IMUManagerStats& operator=(const IMUManagerStats& other) {
        imuAccepted.store(other.imuAccepted.load());
        imuRejected.store(other.imuRejected.load());
        gpsAccepted.store(other.gpsAccepted.load());
        gpsRejected.store(other.gpsRejected.load());
        dbEnqueueFailures.store(other.dbEnqueueFailures.load());

        return *this;
    }
};

#endif