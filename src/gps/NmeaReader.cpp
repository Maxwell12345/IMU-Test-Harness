#include "NmeaReader.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <vector>
#include <cstdlib>

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

NmeaReader::NmeaReader(const std::string& port, int baud)
    : m_port(port), m_baud(baud) {}

NmeaReader::~NmeaReader() {
    close();
}

bool NmeaReader::open() {
    m_fd = ::open(m_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_fd < 0) return false;

    termios tty{};
    if (tcgetattr(m_fd, &tty) != 0) return false;

    speed_t speed = B9600;
    if (m_baud == 4800) speed = B4800;
    else if (m_baud == 19200) speed = B19200;
    else if (m_baud == 38400) speed = B38400;
    else if (m_baud == 57600) speed = B57600;
    else if (m_baud == 115200) speed = B115200;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    return tcsetattr(m_fd, TCSANOW, &tty) == 0;
}

void NmeaReader::close() {
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool NmeaReader::readLine(std::string& line) {
    line.clear();

    char c;
    while (true) {
        int n = ::read(m_fd, &c, 1);
        if (n <= 0) return false;
        if (c == '\n') return !line.empty();
        if (c != '\r') line.push_back(c);
    }
}

bool NmeaReader::readMessage(NmeaMessage& out) {
    std::string line;

    while (readLine(line)) {
        if (!line.empty() && line[0] == '$') {
            out = parse(line);
            return true;
        }
    }

    return false;
}

bool NmeaReader::validChecksum(const std::string& line) {
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

double NmeaReader::parseDeg(const std::string& raw, const std::string& hemi) {
    if (raw.empty()) return 0.0;

    double v = std::atof(raw.c_str());
    int deg = static_cast<int>(v / 100.0);
    double mins = v - deg * 100.0;
    double out = deg + mins / 60.0;

    if (hemi == "S" || hemi == "W") out *= -1.0;
    return out;
}

NmeaMessage NmeaReader::parse(const std::string& line) {
    NmeaMessage msg;
    msg.raw = line;
    msg.validChecksum = validChecksum(line);

    if (!msg.validChecksum) return msg;

    size_t star = line.find('*');
    std::string body = line.substr(1, star - 1);
    auto f = split(body, ',');

    if (f.empty()) return msg;

    msg.type = f[0];

    if ((msg.type == "GPRMC" || msg.type == "GNRMC") && f.size() >= 10) {
        msg.validFix = f[2] == "A";
        msg.lat = parseDeg(f[3], f[4]);
        msg.lon = parseDeg(f[5], f[6]);
        msg.speedKnots = f[7].empty() ? 0.0 : std::atof(f[7].c_str());
        msg.courseDeg = f[8].empty() ? 0.0 : std::atof(f[8].c_str());
    }

    if ((msg.type == "GPGGA" || msg.type == "GNGGA") && f.size() >= 10) {
        msg.lat = parseDeg(f[2], f[3]);
        msg.lon = parseDeg(f[4], f[5]);
        msg.fixQuality = f[6].empty() ? 0 : std::atoi(f[6].c_str());
        msg.validFix = msg.fixQuality > 0;
        msg.numSatellites = f[7].empty() ? 0 : std::atoi(f[7].c_str());
        msg.hdop = f[8].empty() ? 0.0 : std::atof(f[8].c_str());
        msg.altitudeM = f[9].empty() ? 0.0 : std::atof(f[9].c_str());
    }

    return msg;
}