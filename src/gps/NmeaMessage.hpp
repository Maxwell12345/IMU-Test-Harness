#ifndef NMEA_MESSAGE_HPP
#define NMEA_MESSAGE_HPP

#include <string>

struct NmeaMessage {
    std::string type;
    std::string raw;
    bool validChecksum = false;
    bool validFix = false;
    double lat = 0.0;
    double lon = 0.0;
    double speedKnots = 0.0;
    double courseDeg = 0.0;
    int fixQuality = 0;
    int numSatellites = 0;
    double hdop = 0.0;
    double altitudeM = 0.0;
};

#endif