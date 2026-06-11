#ifndef INU_DISPLAY_GPSRECORDS_HPP
#define INU_DISPLAY_GPSRECORDS_HPP

#include <cstdint>
#include <optional>
#include <string>

struct GpsRecord {
    int64_t timestampNs{0};
    std::string deviceId;
    std::string nmeaCode;
    std::string rawSentence;
    std::optional<double> latitude;
    std::optional<double> longitude;
    std::optional<double> altitude;
    std::optional<int> satellites;
};

#endif
