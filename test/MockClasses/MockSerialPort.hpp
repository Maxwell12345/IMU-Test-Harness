#ifndef MOCK_SERIAL_PORT_HPP
#define MOCK_SERIAL_PORT_HPP

<<<<<<< HEAD
#include "SerialPort/SerialPortBase.hpp"
=======
#include "SerialPortBase.hpp"
>>>>>>> main

class MockSerialPort : public SerialPortBase {
public:
    void Open(const std::string& port) override;
    void SetBaudRate(unsigned int rate) override;
    void Callback() override;
    void Close() override;
};

#endif