#ifndef SERIAL_PORT_HP
#define SERIAL_PORT_HP

#include <string>
#include <functional>

#include <boost/asio.hpp>

/**
 * @brief Serial Port Base class for DI. To be inherited by Boost wrapper and mock class
 */
class SerialPortBase {
public:
    virtual ~SerialPortBase() = default;
    virtual void InstallCallback(std::function<void(SerialPortBase&)> callback) {};
    virtual void ReadExact(unsigned char* data, std::size_t len) {};
    virtual void ReadUntil(std::string& dst, const std::string& delim) {};
    virtual void Open(const std::string& port) {};
    virtual void SetBaudRate(unsigned int rate) {};
    virtual void Callback() {};
    virtual void Close() {};
};

#endif