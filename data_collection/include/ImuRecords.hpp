#ifndef INU_DISPLAY_IMURECORDS_HPP
#define INU_DISPLAY_IMURECORDS_HPP

#include <cstdint>

struct GyroscopeRecord {
    int64_t timestampNs{0};
    uint64_t sensorTimestampUs{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    uint8_t calibAccuracy{0};
};

struct AccelerometerRecord {
    int64_t timestampNs{0};
    uint64_t sensorTimestampUs{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    uint8_t calibAccuracy{0};
};

struct LinearAccelRecord {
    int64_t timestampNs{0};
    uint64_t sensorTimestampUs{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    uint8_t calibAccuracy{0};
};

struct RotationVectorRecord {
    int64_t timestampNs{0};
    uint64_t sensorTimestampUs{0};
    float i{0.0f};
    float j{0.0f};
    float k{0.0f};
    float real{0.0f};
    float accuracyRad{0.0f};
    uint8_t calibAccuracy{0};
};

#endif
