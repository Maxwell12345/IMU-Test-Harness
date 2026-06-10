/******************************************************************************
 * File:             GpsManager.hpp
 *
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 * Description:      Briefly describe the purpose of this file and its role within
 *                   the project. Mention key functions or classes it Contains.
 *
 ******************************************************************************/
#ifndef INU_DISPLAY_GPSMANAGER_HPP
#define INU_DISPLAY_GPSMANAGER_HPP

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <thread>

#include "utils.hpp"

class DatabaseManager;

struct GpsManagerStats {
  std::atomic<uint64_t> totalSentencesReceived{0};
  std::atomic<uint64_t> acceptedFixes{0};
  std::atomic<uint64_t> parseFailures{0};
  std::atomic<uint64_t> checksumFailures{0};
  std::atomic<uint64_t> validationRejected{0};
  std::atomic<uint64_t> dbEnqueueFailures{0};
};

class GpsManager {
public:
  GpsManager(const std::string &comPort, unsigned int baudRate) {
    if (comPort.empty()) {
      throw std::invalid_argument("GpsManager: specified com port cannot be empty");
    }
    if (baudRate == 0) {
      throw std::invalid_argument("GpsManager: specified baud rate cannot be zero");
    }
    this->m_comPort = comPort;
    this->m_baudRate = baudRate;
  };

  GpsManager(const std::string &comPort, unsigned int baudRate, boost::shared_ptr<DatabaseManager> databaseManager) {
    if (comPort.empty()) {
      throw std::invalid_argument("GpsManager: specified com port cannot be empty");
    }
    if (baudRate == 0) {
      throw std::invalid_argument("GpsManager: specified baud rate cannot be zero");
    }
    if (databaseManager == nullptr) {
      throw std::invalid_argument("GpsManager: specified database manager cannot be null");
    }
    this->m_comPort = comPort;
    this->m_baudRate = baudRate;
    this->m_databaseManager = std::move(databaseManager);
  };

  ~GpsManager() = default;

  void start();

  void stop();

  bool isRunning() const;

  GpsManagerStats getStats() const;

private:
  void readLoop(std::stop_token stopToken);

  std::optional<IMUUtils::GpsUpdate> parseNmea(const std::string &sentence);

  bool validateFix(const IMUUtils::GpsUpdate &update) const;

  void publishUpdate(IMUUtils::GpsUpdate update);

private:
  std::string m_comPort;
  boost::asio::serial_port m_serialPort;
  unsigned int m_baudRate;
  boost::shared_ptr<DatabaseManager> m_databaseManager;
  std::optional<boost::asio::io_context> m_ioContext;
  boost::asio::streambuf m_readBuf;
  std::queue<std::string> m_sentenceQueue;
  std::jthread m_readThread;
  std::atomic<bool> m_running;
  std::mutex m_statsMutex;
  std::mutex m_queueMutex;
  std::condition_variable m_queueCv;
  GpsManagerStats m_stats;
};

#endif // INU_DISPLAY_GPSMANAGER_HPP
