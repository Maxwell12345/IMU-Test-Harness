#include "NmeaReader.hpp"

#include <fcntl.h>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <memory>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <termios.h>
#endif

NmeaReader::NmeaReader(const std::string& path, int baud): m_nmeaMessageReady(false) {
    auto f = [nmeaReader = this](boost::asio::serial_port& serial){
        nmeaReader->Callback(serial);
    };

    m_serialComService = std::make_unique<SerialComService>(path,
                                                            baud,
                                                            std::make_unique<BoostSerialPort>(f));
}

static std::vector<std::string> split(const std::string& s, char d) {
    std::vector<std::string> out;
    std::string item;
    std::stringstream ss(s);
    while (std::getline(ss, item, d)) out.push_back(item);
    return out;
}

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

NmeaReader::~NmeaReader() {
    m_serialComService->Stop();
}

void NmeaReader::Start() {
    m_serialComService->Start();
}

void NmeaReader::Stop() {
    m_serialComService->Stop();
}

bool NmeaReader::GetNmeaMessageReady() const {
    return m_nmeaMessageReady;
}

NmeaMessage NmeaReader::GetNmeaMessage() const {
    std::lock_guard msgGuard(m_nmeaMessageMutex);
    m_nmeaMessageReady = false;
    return m_nmeaMessage;
}

void NmeaReader::Callback(boost::asio::serial_port& serial) {
    boost::asio::streambuf buf;
    boost::asio::read_until(serial, buf, "\n");
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);

    printf("[DEBUG] %s\n", line.c_str());

    if (!line.empty() && line[0] == '$') {
        m_nmeaMessage = Parse(line);
        if(m_nmeaMessage.validChecksum == true) {
            m_nmeaMessageReady = true;
        }
    }
}

bool NmeaReader::ValidChecksum(const std::string& line) {
    if (line.size() < 4 || line[0] != '$') return false;

    size_t star = line.find('*');
    if (star == std::string::npos || star + 2 >= line.size()) return false;

    unsigned char chk = 0;
    for (size_t i = 1; i < star; ++i) chk ^= static_cast<unsigned char>(line[i]);

    int h1 = hexval(line[star + 1]);
    int h2 = hexval(line[star + 2]);

    if (h1 < 0 || h2 < 0) return false;

    int given = (h1 << 4) | h2;
    return given == chk;
}

double NmeaReader::ParseDeg(const std::string& raw, const std::string& hemi) {
    if (raw.empty()) return 0.0;

    double v = std::atof(raw.c_str());
    int deg = static_cast<int>(v / 100.0);
    double mins = v - deg * 100.0;
    double out = deg + mins / 60.0;

    if (hemi == "S" || hemi == "W") out *= -1.0;
    return out;
}

NmeaMessage NmeaReader::Parse(const std::string& line) {
    NmeaMessage msg;
    msg.raw = line;
    msg.validChecksum = ValidChecksum(line);

    if (!msg.validChecksum) return msg;

    size_t star = line.find('*');
    std::string body = line.substr(1, star - 1);
    auto f = split(body, ',');

    if (f.empty()) return msg;

    msg.type = f[0];

    if ((msg.type == "GPRMC" || msg.type == "GNRMC") && f.size() >= 10) {
        msg.validFix = f[2] == "A";
        msg.lat = ParseDeg(f[3], f[4]);
        msg.lon = ParseDeg(f[5], f[6]);
        msg.speedKnots = f[7].empty() ? 0.0 : std::atof(f[7].c_str());
        msg.courseDeg = f[8].empty() ? 0.0 : std::atof(f[8].c_str());
    }

    if ((msg.type == "GPGGA" || msg.type == "GNGGA") && f.size() >= 10) {
        msg.lat = ParseDeg(f[2], f[3]);
        msg.lon = ParseDeg(f[4], f[5]);
        msg.fixQuality = f[6].empty() ? 0 : std::atoi(f[6].c_str());
        msg.validFix = msg.fixQuality > 0;
        msg.numSatellites = f[7].empty() ? 0 : std::atoi(f[7].c_str());
        msg.hdop = f[8].empty() ? 0.0 : std::atof(f[8].c_str());
        msg.altitudeM = f[9].empty() ? 0.0 : std::atof(f[9].c_str());
    }

    return msg;
}