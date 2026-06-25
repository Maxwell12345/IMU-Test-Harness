#ifndef MULTICAST_SERVICE_HPP
#define MULTICAST_SERVICE_HPP

#include <iostream>
#include <sstream>
#include <string>

#include <boost/asio.hpp>

class MulticastService {
public:
    MulticastService(const std::string& ip, const unsigned int& port);

    /**
     * @brief Blocking method. Sends data to the configured multicast group
     *
     * @param [in] message
     *
     * @return number of bytes sent
     *
     * @throws boost::system::system_error on send failure
     */
    size_t Send(const std::string& message);

private:
    boost::asio::io_context m_ioContext;
    boost::asio::ip::udp::endpoint m_endpoint;
    boost::asio::ip::udp::socket m_socket;

};

#endif