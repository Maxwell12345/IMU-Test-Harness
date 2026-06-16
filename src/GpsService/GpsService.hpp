#ifndef SERIAL_GPS_SERVICE_HPP
#define SERIAL_GPS_SERVICE_HPP

#include <string>
#include <atomic>

#include <boost/asio.hpp>

class GpsService {
public:
    GpsService(std::string port, unsigned int baudRate);
    ~GpsService();

    void Start();
    void Stop();

    void ConfigureSerialPort();

private:
    
    std::string ReadFromPort();

    std::jthread m_runThread;
    std::atomic<bool> m_running;

    boost::asio::io_context m_io;
    boost::asio::serial_port m_serial;

    std::string m_port;
    unsigned int m_baudRate;
};

#endif