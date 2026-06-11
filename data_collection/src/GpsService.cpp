#include "GpsService.hpp"

#include "BrokerService.hpp"
#include "GpsRecords.hpp"
#include "Logger.hpp"
#include "UsbSerialLocator.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <optional>
#include <poll.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

std::string usbIdString(uint16_t vendorId, uint16_t productId) {
    char buffer[16];
    std::snprintf(buffer, sizeof(buffer), "%04x:%04x", vendorId, productId);
    return buffer;
}

int64_t nowEpochNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

bool baudToSpeed(int baud, speed_t& speed) {
    switch (baud) {
        case 4800:   speed = B4800;   return true;
        case 9600:   speed = B9600;   return true;
        case 19200:  speed = B19200;  return true;
        case 38400:  speed = B38400;  return true;
        case 57600:  speed = B57600;  return true;
        case 115200: speed = B115200; return true;
        default:     return false;
    }
}

bool configureSerialPort(int fd, int baud) {
    speed_t speed{};
    if (!baudToSpeed(baud, speed)) {
        LOG_ERROR("GpsService") << "Unsupported baud rate: " << baud;
        return false;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        return false;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CLOCAL;
    tty.c_cflag |= CREAD;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

std::string trimLineEndings(std::string line) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
        line.pop_back();
    }
    return line;
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    return -1;
}

bool checksumValid(const std::string& sentence) {
    if (sentence.empty() || sentence[0] != '$') {
        return false;
    }
    const std::size_t star = sentence.find('*');
    if (star == std::string::npos || star + 2 >= sentence.size()) {
        return false;
    }

    const int hi = hexValue(sentence[star + 1]);
    const int lo = hexValue(sentence[star + 2]);
    if (hi < 0 || lo < 0) {
        return false;
    }

    const auto expected = static_cast<unsigned char>((hi << 4) | lo);
    unsigned char actual = 0;
    for (std::size_t i = 1; i < star; ++i) {
        actual ^= static_cast<unsigned char>(sentence[i]);
    }
    return actual == expected;
}

std::vector<std::string> splitFields(const std::string& sentence) {
    std::vector<std::string> fields;
    std::string token;
    for (char c : sentence) {
        if (c == ',' || c == '*') {
            fields.push_back(std::move(token));
            token.clear();
            if (c == '*') {
                break;
            }
        } else {
            token += c;
        }
    }
    fields.push_back(std::move(token));
    return fields;
}

std::string sentenceCode(const std::string& sentence) {
    const std::size_t comma = sentence.find(',');
    const std::size_t end = (comma == std::string::npos) ? sentence.size() : comma;
    if (end <= 1) {
        return {};
    }
    return sentence.substr(1, end - 1);
}

std::optional<double> parseDegMin(const std::string& value, const std::string& hemi) {
    if (value.empty()) {
        return std::nullopt;
    }
    try {
        const double raw = std::stod(value);
        const int degrees = static_cast<int>(raw / 100.0);
        const double minutes = raw - degrees * 100.0;
        double decimal = degrees + minutes / 60.0;
        if (hemi == "S" || hemi == "W") {
            decimal = -decimal;
        }
        return decimal;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::optional<double> parseDouble(const std::string& value) {
    if (value.empty()) {
        return std::nullopt;
    }
    try {
        return std::stod(value);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::optional<int> parseInt(const std::string& value) {
    if (value.empty()) {
        return std::nullopt;
    }
    try {
        return std::stoi(value);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace

GpsService::GpsService(BrokerService& broker, std::string deviceId, uint16_t vendorId,
                       uint16_t productId, int baud)
    : m_broker(broker),
      m_deviceId(std::move(deviceId)),
      m_vendorId(vendorId),
      m_productId(productId),
      m_baud(baud) {}

GpsService::~GpsService() { stop(); }

void GpsService::start() {
    if (m_running.exchange(true)) {
        return;
    }
    LOG_INFO("GpsService") << m_deviceId << ": starting, searching for USB device "
                           << usbIdString(m_vendorId, m_productId);
    m_thread = std::thread(&GpsService::readerLoop, this);
}

void GpsService::stop() {
    if (!m_running.exchange(false)) {
        return;
    }
    if (m_thread.joinable()) {
        m_thread.join();
    }
    LOG_INFO("GpsService") << m_deviceId << ": stopped";
}

void GpsService::sleepInterruptible(std::chrono::milliseconds total) {
    const auto deadline = std::chrono::steady_clock::now() + total;
    while (m_running.load(std::memory_order_relaxed) &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int GpsService::openPort(std::string& resolvedPort) {
    const std::optional<std::string> port = FindSerialPortByUsbId(m_vendorId, m_productId);
    if (!port) {
        LOG_DEBUG("GpsService") << m_deviceId << ": USB device "
                                << usbIdString(m_vendorId, m_productId) << " not present";
        return -1;
    }
    resolvedPort = *port;

    const int fd = ::open(port->c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        LOG_WARN("GpsService") << m_deviceId << ": failed to open " << *port << ": "
                               << std::strerror(errno);
        return -1;
    }
    if (!configureSerialPort(fd, m_baud)) {
        LOG_WARN("GpsService") << m_deviceId << ": failed to configure " << *port;
        ::close(fd);
        return -1;
    }
    return fd;
}

void GpsService::readerLoop() {
    while (m_running.load(std::memory_order_relaxed)) {
        std::string port;
        const int fd = openPort(port);
        if (fd < 0) {
            sleepInterruptible(std::chrono::milliseconds(1000));
            continue;
        }

        LOG_INFO("GpsService") << m_deviceId << ": connected on " << port << " (USB "
                               << usbIdString(m_vendorId, m_productId) << ")";
        readFromPort(fd);
        ::close(fd);

        if (m_running.load(std::memory_order_relaxed)) {
            LOG_WARN("GpsService") << m_deviceId << ": link lost on " << port
                                   << ", attempting to reconnect";
            sleepInterruptible(std::chrono::milliseconds(1000));
        }
    }
}

void GpsService::readFromPort(int fd) {
    std::string line;
    char buffer[256];

    while (m_running.load(std::memory_order_relaxed)) {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        const int ready = ::poll(&pfd, 1, 100);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (ready == 0) {
            continue;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            break;
        }

        const ssize_t count = ::read(fd, buffer, sizeof(buffer));
        if (count < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            break;
        }
        if (count == 0) {
            continue;
        }

        for (ssize_t i = 0; i < count; ++i) {
            const char ch = buffer[i];
            if (ch == '\n') {
                std::string sentence = trimLineEndings(line);
                line.clear();

                const std::size_t start = sentence.find('$');
                if (start != std::string::npos) {
                    sentence.erase(0, start);
                    processSentence(sentence);
                }
            } else if (ch != '\0') {
                if (line.size() < 1024) {
                    line += ch;
                } else {
                    line.clear();
                }
            }
        }
    }
}

void GpsService::processSentence(const std::string& sentence) {
    if (!checksumValid(sentence)) {
        return;
    }

    const std::string code = sentenceCode(sentence);
    if (code.size() < 3) {
        return;
    }
    const std::string type = code.substr(code.size() - 3);
    if (type != "GGA" && type != "RMC" && type != "GLL") {
        return;
    }

    const std::vector<std::string> fields = splitFields(sentence);

    GpsRecord record{};
    record.timestampNs = nowEpochNs();
    record.deviceId = m_deviceId;
    record.nmeaCode = code;
    record.rawSentence = sentence;

    if (type == "GGA" && fields.size() >= 11) {
        record.latitude = parseDegMin(fields[2], fields[3]);
        record.longitude = parseDegMin(fields[4], fields[5]);
        record.satellites = parseInt(fields[7]);
        record.altitude = parseDouble(fields[9]);
    } else if (type == "RMC" && fields.size() >= 7) {
        record.latitude = parseDegMin(fields[3], fields[4]);
        record.longitude = parseDegMin(fields[5], fields[6]);
    } else if (type == "GLL" && fields.size() >= 5) {
        record.latitude = parseDegMin(fields[1], fields[2]);
        record.longitude = parseDegMin(fields[3], fields[4]);
    }

    LOG_DEBUG("GpsService") << m_deviceId << ": " << code
                            << " lat=" << (record.latitude ? *record.latitude : 0.0)
                            << " lon=" << (record.longitude ? *record.longitude : 0.0)
                            << " sats=" << (record.satellites ? *record.satellites : 0);

    m_broker.enqueueGps(record);
}
