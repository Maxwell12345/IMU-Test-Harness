#ifndef SERIAL_PORT_HP
#define SERIAL_PORT_HP

#include <string>

/**
 * @brief Serial Port Base class for DI. To be inherited by Boost wrapper and mock class
 */
class SerialPortBase {
public:
    virtual ~SerialPortBase() = default;
    virtual void Open(const std::string& port) = 0;
    virtual void SetBaudRate(unsigned int rate) = 0;
    virtual std::string ReadLine() = 0;
    virtual void Close() = 0;
};

#endif