/******************************************************************************
 * File:             GpsManager.cpp
 *
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 *
 ******************************************************************************/
#include "GpsManager.hpp"

void GpsManager::start() {
  this->m_serialPort.emplace(this->m_ioContext, this->m_comPort);

  this->m_serialPort.set_option(boost::asio::serial_port_base::baud_rate(this->m_baudRate));
  this->m_serialPort.set_option(boost::asio::serial_port_base::character_size(8));
  this->m_serialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  this->m_serialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  this->m_serialPort.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  this->m_running.store(true);

  this->m_readThread = std::jthread([this](std::stop_token st) {
    this->readLoop(st);
    // TODO: set running to false
  });
}

void GpsManager::stop() {}

bool GpsManager::isRunning() const {}

GpsManagerStats GpsManager::getStats() const {}

void GpsManager::readLoop(std::stop_token stopToken) {
  std::stop_callback onStop(stopToken, [this]() {
    boost::system::error_code ec;
    if (this->m_serialPort && this->m_serialPort->is_open()) {
      this->m_serialPort->cancel(ec);
    }
  });

  while (!stopToken.stop_requested()) {
    boost::system::error_code ec;
    boost::asio::read_until(*this->m_serialPort, this->m_readBuf, '\n', ec);
    if (ec)
      break;

    std::istream serialStream(&this->m_readBuf);
    std::string sentence;
    std::getline(stream, sentence);
    if (!sentence.empty() && sentence.back() == '\r') {
      sentence.pop_back();
      {
        std::lock_guard(this->m_queueMutex);
        this->m_sentenceQueue.push(std::move(sentence));
      }
      this->m_queueCv.notify_one();
    }
  }

  std::optional<IMUUtils::GpsUpdate> GpsManager::parseNmea(const std::string &sentence) {}

  bool GpsManager::validateFix(const IMUUtils::GpsUpdate &update) const {}

  void GpsManager::publishUpdate(IMUUtils::GpsUpdate update) {}
