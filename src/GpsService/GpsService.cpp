#include <iostream>
#include <thread>

#include <boost/filesystem.hpp>

#include "Utils.hpp"
#include "GpsService.hpp"

GpsService::GpsService(std::string port, unsigned int baudRate) : m_serial(m_io),
                                                                  m_running(false),
                                                                  m_port(port),
                                                                  m_baudRate(baudRate) {
    try {
        ConfigureSerialPort();
    } catch (std::runtime_error& e) {
        utils::LOG_ERROR(e.what());
    }
}

GpsService::~GpsService() {
    Stop();
}

void GpsService::Start() {
    m_running = true;

    m_runThread = std::jthread([this](std::stop_token st){
        while(m_running) {
            std::string nmeaSentence = ReadFromPort();
            utils::LOG_INFO(nmeaSentence, true);
        }
    
        m_serial.close();
    });
}

void GpsService::Stop() {
    m_running = false;
    m_runThread.request_stop();

    if (m_runThread.joinable()) {
        m_runThread.join();
        utils::LOG_INFO("Joined Serial Reading thread");
    }
}

void GpsService::ConfigureSerialPort() {
    boost::system::error_code ec;
    m_serial.open(m_port, ec);
    if (ec) {
        throw std::runtime_error(
            "Failed to open serial port: " + ec.message()
        );
    }
    m_serial.set_option(boost::asio::serial_port_base::baud_rate(m_baudRate));
}

std::string GpsService::ReadFromPort() {
    boost::asio::streambuf buf;
    boost::asio::read_until(m_serial, buf, "\n");;
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);

    return line;
}