#ifndef IMU_IMU_ROTATION_VECTOR_RECORD_HPP
#define IMU_IMU_ROTATION_VECTOR_RECORD_HPP

extern "C" {
    #include <sh2_SensorValue.h>
}

#include <string>
#include <cstdio>
#include <stdint.h>
#include <variant>
#include <stdexcept>

struct IMURotationVectorRecord {
    double timestamp;
    uint8_t status;
    uint64_t hwTimestampUs;
    float i;
    float j;
    float k;
    float real;
    float accuracy;

    IMURotationVectorRecord(): timestamp(0),
                               status(0),
                               hwTimestampUs(0),
                               i(0),
                               j(0),
                               k(0),
                               real(0),
                               accuracy(0)
                               {};

    IMURotationVectorRecord(const sh2_SensorValue& v) {
        if(v.sensorId != SH2_ROTATION_VECTOR) {
            throw std::invalid_argument("SensorID must be SH2_ROTATION_VECTOR");
        }

        timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        status = v.status;
        hwTimestampUs = v.timestamp;
        i = v.un.rotationVector.i;
        j = v.un.rotationVector.j;
        k = v.un.rotationVector.k;
        real = v.un.rotationVector.real;
        accuracy = v.un.rotationVector.accuracy;
    }

    std::string toString() const {
        char cs[512];
        snprintf(cs, sizeof(cs), "Timestamp: %f, Status: %d, Timestamp(us): %ld, i: %.6f, j: %.6f, k: %.6f, real: %.6f, acc: %.6f",
                 timestamp,
                 status,
                 hwTimestampUs,
                 i,
                 j,
                 k,
                 real,
                 accuracy);
        return std::string(cs);
    }
};

#endif