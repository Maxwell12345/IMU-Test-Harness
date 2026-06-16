#ifndef MOCK_SERIAL_PORT_HPP
#define MOCK_SERIAL_PORT_HPP

#include "SerialPort/SerialPortBase.hpp"

class MockSerialPort : public SerialPortBase {
public:
    void Open(const std::string& port) override;
    void SetBaudRate(unsigned int rate) override;
    std::string ReadLine() override;
    void Close() override;
};

#endif