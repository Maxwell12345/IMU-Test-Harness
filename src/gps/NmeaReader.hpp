#pragma once

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

class NmeaReader {
public:
    explicit NmeaReader(const std::string& port, int baud);
    ~NmeaReader();

    bool open();
    void close();
    bool readMessage(NmeaMessage& out);

private:
    std::string m_port;
    int m_baud;
    int m_fd = -1;

    bool readLine(std::string& line);
    NmeaMessage parse(const std::string& line);
    bool validChecksum(const std::string& line);
    double parseDeg(const std::string& raw, const std::string& hemi);
};