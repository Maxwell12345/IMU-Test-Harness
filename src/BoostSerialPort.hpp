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
    BoostSerialPort();

    /**
     * @brief installs a callback to the port
     * 
     * @remark the callback will pass in a parameter typed boost::asio::serial_port. This is the real asio port
     *      that can be used to read data from
     *
     * @param [in] callback installs a function that handles the read operation and processing of the data
     * 
     * @return
     */
    void InstallCallback(std::function<void(SerialPortBase&)> callback) override;

    void Open(const std::string& port) override;

    /**
     * @brief read COM port exactly len bytes and store read data in dst
     * 
     * @param [out] dst read data stored destination
     * @param [in] read length in bytes
     * 
     * @return
     */
    void ReadExact(unsigned char* dst, std::size_t len) override;

    /**
     * @brief read COM port until pointer encounters a delimiter and store to dst as std::string
     * 
     * @param [out] dst read data stored destination
     * @param [in] delim delimiter string. Stops reading when delim is encountered
     * 
     * @return
     */
    void ReadUntil(std::string& dst, const std::string& delim) override;

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
    std::function<void(SerialPortBase&)> m_callback;
};

#endif