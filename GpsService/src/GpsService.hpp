#ifndef SERIAL_GPS_SERVICE_HPP
#define SERIAL_GPS_SERVICE_HPP

#include <string>
#include <atomic>
#include <thread>

#include <boost/asio.hpp>

class SerialPortBase;

class GpsService {
public:
    GpsService(std::string port,
               unsigned int baudRate,
               std::unique_ptr<SerialPortBase> serialPort);

    ~GpsService();

    void Start();
    void Stop();

private:
    void ConfigureSerialPort();
    std::string ReadFromPort();

    std::jthread m_runThread;
    std::atomic<bool> m_running;

    boost::asio::io_context m_io;
    // boost::asio::serial_port m_serial;
    std::unique_ptr<SerialPortBase> m_serial;

    std::string m_path;
    unsigned int m_baudRate;
};

#endif