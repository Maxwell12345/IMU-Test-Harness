#ifndef DATABASE_MANAGER_STATS_HPP
#define DATABASE_MANAGER_STATS_HPP

#include <atomic>

struct DatabaseManagerStats {
    std::atomic<uint64_t> gpsQueued{0};
    std::atomic<uint64_t> imuQueued{0};
    std::atomic<uint64_t> ekfQueued{0};
    std::atomic<uint64_t> gpsWritten{0};
    std::atomic<uint64_t> imuWritten{0};
    std::atomic<uint64_t> ekfWritten{0};
    std::atomic<uint64_t> failedWrites{0};
    std::atomic<uint64_t> droppedRecords{0};
    std::atomic<uint64_t> currentQueueDepth{0};

    DatabaseManagerStats(){};

    DatabaseManagerStats(const DatabaseManagerStats& other) {
        gpsQueued = other.gpsQueued.load();
        imuQueued = other.imuQueued.load();
        ekfQueued = other.ekfQueued.load();
        gpsWritten = other.gpsWritten.load();
        imuWritten = other.imuWritten.load();
        ekfWritten = other.ekfWritten.load();
        failedWrites = other.failedWrites.load();
        droppedRecords = other.droppedRecords.load();
        currentQueueDepth = other.currentQueueDepth.load();
    }
};

#endif