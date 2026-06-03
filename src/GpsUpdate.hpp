#pragma once

#include <chrono>

struct GpsUpdate {
    std::chrono::steady_clock::time_point receiveTime;   // monotonic receive timestamp
    std::chrono::system_clock::time_point wallTime;      // wall-clock for DB/logging
    double latitude;                                      // decimal degrees
    double longitude;                                     // decimal degrees
    std::optional<double> heading;                        // course-over-ground degrees (from $GPRMC), nullopt if unavailable
    uint8_t fixQuality;                                   // 0=invalid, 1=GPS, 2=DGPS, etc.
    uint8_t numSatellites;                                // number of satellites in use
    double hdop;                                          // horizontal dilution of precision
    uint32_t gpsTimestampMs;                              // time-of-day from GPS sentence (ms since midnight UTC)
    bool valid;                                           // overall validity flag

    GpsUpdate& operator=(const GpsUpdate& other) {
        receiveTime = other.receiveTime;
        wallTime = other.wallTime;
        latitude = other.latitude;
        longitude = other.longitude;
        heading = other.heading;
        fixQuality = other.fixQuality;
        numSatellites = other.numSatellites;
        hdop = other.hdop;
        gpsTimestampMs = other.gpsTimestampMs;
        valid = other.valid;

        return *this;
    }
};