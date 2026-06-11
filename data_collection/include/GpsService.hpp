#ifndef INU_DISPLAY_GPSSERVICE_HPP
#define INU_DISPLAY_GPSSERVICE_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

class BrokerService;

class GpsService {
public:
    GpsService(BrokerService& broker, std::string deviceId, uint16_t vendorId,
               uint16_t productId, int baud = 9600);
    ~GpsService();

    GpsService(const GpsService&) = delete;
    GpsService& operator=(const GpsService&) = delete;

    void start();
    void stop();

private:
    void readerLoop();
    int openPort(std::string& resolvedPort);
    void readFromPort(int fd);
    void processSentence(const std::string& sentence);
    void sleepInterruptible(std::chrono::milliseconds total);

    BrokerService& m_broker;
    std::string m_deviceId;
    uint16_t m_vendorId;
    uint16_t m_productId;
    int m_baud;
    std::thread m_thread;
    std::atomic<bool> m_running{false};
};

#endif
