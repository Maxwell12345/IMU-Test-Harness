#ifndef INU_DISPLAY_IMURECORDS_HPP
#define INU_DISPLAY_IMURECORDS_HPP

#include <cstdint>

struct GyroscopeRecord {
    int64_t timestampNs{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct AccelerometerRecord {
    int64_t timestampNs{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

#endif
