#ifndef BOOST_SERIAL_PORT_HPP
#define BOOST_SERIAL_PORT_HPP

#include "SerialPortBase.hpp"

class BoostSerialPort : public SerialPortBase {
public:
    BoostSerialPort();
    void Open(const std::string& port) override;
    void SetBaudRate(unsigned int rate) override;
    std::string ReadLine() override;
    void Close() override;

private:
    boost::asio::io_context m_io;
    boost::asio::serial_port m_serial;
};

#endif