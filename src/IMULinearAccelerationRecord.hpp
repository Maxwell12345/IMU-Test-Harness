#ifndef IMU_LINEAR_ACCELERATION_RECORD_HPP
#define IMU_LINEAR_ACCELERATION_RECORD_HPP

#include <cstdio>
#include <string>
#include <stdint.h>
#include <variant>
#include <stdexcept>

#include "SerialDataModel.hpp"

struct IMULinearAccelerationRecord {
    double timestamp;
    uint64_t hwTimestampUs;
    float x;
    float y;
    float z;

    IMULinearAccelerationRecord(): hwTimestampUs(0),
                                   timestamp(0),
                                   x(0),
                                   y(0),
                                   z(0){};

    IMULinearAccelerationRecord(const LinearAcceleration& v) {
        timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        hwTimestampUs = v.timestamp;
        x = v.x;
        y = v.y;
        z = v.z;
    }

    std::string toString() const {
        char cs[512];
        snprintf(cs, sizeof(cs), "Timestamp: %f, HwTimestamp(us): %ld, x: %.6f, y: %.6f, z: %.6f",
                 timestamp,
                 hwTimestampUs,
                 x,
                 y,
                 z);
        return std::string(cs);
    }
};

#endif