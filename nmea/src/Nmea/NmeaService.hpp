#ifndef NMEA_SERVICE_HPP
#define NMEA_SERVICE_HPP

#include <memory>
#include <string>
#include <thread>
#include <atomic>

class SerialPortBase;
class SerialComService;
class MulticastService;

class NmeaService {
public:
    NmeaService(std::unique_ptr<SerialComService> serialComService,
                std::unique_ptr<MulticastService> multicastService);

    ~NmeaService();

    void Start();

    void Stop();

private:
    void Callback(SerialPortBase& port);

    std::atomic<bool> m_running;

    std::mutex m_nmeaMutex;
    std::string m_nmea;

    std::unique_ptr<SerialComService> m_serialComService;
    std::unique_ptr<MulticastService> m_multicastService;
};

#endif