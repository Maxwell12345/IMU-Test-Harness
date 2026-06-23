#ifndef BOOST_SERIAL_PORT_HPP
#define BOOST_SERIAL_PORT_HPP

#include <utility>
#include <functional>
#include <boost/asio.hpp>

#include "SerialPortBase.hpp"

/**
 * @brief Boost serial thin wrapper class to support DI. Refer to Boost for documentation.
 *      Only custom logic will be noted.
 */
class BoostSerialPort : public SerialPortBase {
public:
    /**
     * @brief constructor
     *
     * @param [in] callback installs a function that handles the read operation and processing of the data
     */
    BoostSerialPort(std::function<void(boost::asio::serial_port& m_serial)> callback);

    void Open(const std::string& port) override;

    void SetBaudRate(unsigned int rate) override;

    /**
     * @brief invoke the installed callback m_callback
     *
     * @return
     */
    void Callback() override;

    void Close() override;

private:
    boost::asio::io_context m_io;
    boost::asio::serial_port m_serial;
    std::function<void(boost::asio::serial_port& m_serial)> m_callback;
};

#endif