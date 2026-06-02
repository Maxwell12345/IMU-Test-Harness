#pragma once

#include <atomic>

struct IMUManagerStats {
    std::atomic<uint64_t> imuAccepted{0};
    std::atomic<uint64_t> imuRejected{0};
    std::atomic<uint64_t> gpsAccepted{0};
    std::atomic<uint64_t> gpsRejected{0};
    std::atomic<uint64_t> ekfCallbackFailures{0};
    std::atomic<uint64_t> dbEnqueueFailures{0};
};