#ifndef IMU_LINEAR_ACCELERATION_RECORD_HPP
#define IMU_LINEAR_ACCELERATION_RECORD_HPP

extern "C" {
    #include <sh2_SensorValue.h>
}

#include <cstdio>
#include <string>
#include <stdint.h>
#include <variant>
#include <stdexcept>

struct IMULinearAccelerationRecord {
    double timestamp;
    uint8_t status;
    uint64_t hwTimestampUs;
    float x;
    float y;
    float z;

    IMULinearAccelerationRecord(): hwTimestampUs(0),
                                   status(0),
                                   timestamp(0),
                                   x(0),
                                   y(0),
                                   z(0){};

    IMULinearAccelerationRecord(const sh2_SensorValue& v) {
        if(v.sensorId != SH2_LINEAR_ACCELERATION) {
            throw std::invalid_argument("SensorID must be SH2_LINEAR_ACCELERATION");
        }

        timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        status = v.status;
        hwTimestampUs = v.timestamp;
        x = v.un.linearAcceleration.x;
        y = v.un.linearAcceleration.y;
        z = v.un.linearAcceleration.z;
    }

    std::string toString() const {
        char cs[512];
        snprintf(cs, sizeof(cs), "Timestamp: %f, Status: %d, HwTimestamp(us): %ld, x: %.6f, y: %.6f, z: %.6f",
                 timestamp,
                 status,
                 hwTimestampUs,
                 x,
                 y,
                 z);
        return std::string(cs);
    }
};

#endif