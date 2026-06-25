#include "MulticastService.hpp"

MulticastService::MulticastService(const std::string& ip, const unsigned int& port):
    m_endpoint(boost::asio::ip::make_address(ip), port),
    m_socket(m_ioContext, m_endpoint.protocol()){}

size_t MulticastService::Send(const std::string& message) {
    return m_socket.send_to(boost::asio::buffer(message),
                            m_endpoint);
}
