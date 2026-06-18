#ifndef IMU_IMU_ROTATION_VECTOR_RECORD_HPP
#define IMU_IMU_ROTATION_VECTOR_RECORD_HPP

#include <string>
#include <cstdio>
#include <stdint.h>
#include <variant>
#include <stdexcept>

#include "SerialDataModel.hpp"

struct IMURotationVectorRecord {
    double timestamp;
    uint64_t hwTimestampUs;
    float i;
    float j;
    float k;
    float real;
    float accuracy;

    IMURotationVectorRecord(): timestamp(0),
                               hwTimestampUs(0),
                               i(0),
                               j(0),
                               k(0),
                               real(0),
                               accuracy(0)
                               {};

    IMURotationVectorRecord(const RotationVectorWAcc& v) {
        timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        hwTimestampUs = v.timestamp;
        i = v.i;
        j = v.j;
        k = v.k;
        real = v.real;
        accuracy = v.accuracy;
    }

    std::string toString() const {
        char cs[512];
        snprintf(cs, sizeof(cs), "Timestamp: %f, Timestamp(us): %ld, i: %.6f, j: %.6f, k: %.6f, real: %.6f, acc: %.6f",
                 timestamp,
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