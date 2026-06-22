#include "BoostSerialPort.hpp"

BoostSerialPort::BoostSerialPort(std::function<void(boost::asio::serial_port& m_serial)> callback) :
                                                                   m_serial(m_io),
                                                                   m_callback(callback) {
    
}

void BoostSerialPort::Open(const std::string& port) {
    boost::system::error_code ec;
    m_serial.open(port, ec);

    if (ec) {
        throw std::runtime_error("Failed to open: " + ec.message() + port);
    }
}

void BoostSerialPort::SetBaudRate(unsigned int rate) {
    m_serial.set_option(boost::asio::serial_port_base::baud_rate(rate));
}

void BoostSerialPort::Callback() {
    m_callback(m_serial);
}

void BoostSerialPort::Close() { 
    m_serial.close();
}