#include "BoostSerialPort.hpp"

BoostSerialPort::BoostSerialPort() : m_serial(m_io){}

void BoostSerialPort::InstallCallback(std::function<void(SerialPortBase& port)> callback) {
    m_callback = callback;
}

void BoostSerialPort::Open(const std::string& port) {
    boost::system::error_code ec;
    m_serial.open(port, ec);

    if (ec) {
        throw std::runtime_error("Failed to open: " + ec.message() + " " + port);
    }
}

void BoostSerialPort::ReadExact(unsigned char* dst, std::size_t len) {
    boost::system::error_code ec;

    boost::asio::read(m_serial, boost::asio::buffer(dst, len), boost::asio::transfer_exactly(len), ec);

    if (ec) {
        throw std::runtime_error("Failed to read: " + ec.message());
    }
}

void BoostSerialPort::ReadUntil(std::string& dst, const std::string& delim) {
    boost::asio::streambuf buf;
    boost::asio::read_until(m_serial, buf, "\n");
    std::istream is(&buf);
    std::getline(is, dst);
}

void BoostSerialPort::SetBaudRate(unsigned int rate) {
    m_serial.set_option(boost::asio::serial_port_base::baud_rate(rate));
}

void BoostSerialPort::Callback() {
    m_callback(*this);
}

void BoostSerialPort::Close() { 
    m_serial.close();
}