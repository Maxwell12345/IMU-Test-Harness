#include "BoostSerialPort.hpp"

BoostSerialPort::BoostSerialPort() : m_serial(m_io) {
    
}

void BoostSerialPort::Open(const std::string& port) {
    boost::system::error_code ec;
    m_serial.open(port, ec);

    if (ec) {
        throw std::runtime_error("Failed to open: " + ec.message());
    }
}

void BoostSerialPort::SetBaudRate(unsigned int rate) {
    m_serial.set_option(boost::asio::serial_port_base::baud_rate(rate));
}

std::string BoostSerialPort::ReadLine() {
    boost::asio::streambuf buf;
    boost::asio::read_until(m_serial, buf, "\n");
    std::istream is(&buf);
    std::string line;
    std::getline(is, line);
    return line;
}

void BoostSerialPort::Close() { 
    m_serial.close();
}