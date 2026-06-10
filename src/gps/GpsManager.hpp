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

#include <condition_variable>
#include <functional>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <queue>
#include <string>
#include <thread>

#include "utils.hpp"
// #include "NMEAParserWrapper.hpp"
#include "NMEAParser.hpp"

class DatabaseManager;
class GpsUpdate;

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
  GpsManager(const std::string &comPort,
    unsigned int baudRate,
    std::function<void(const GpsUpdate&)> gpsUpdateCallback):m_gpsUpdateCallback(gpsUpdateCallback) {
    if (comPort.empty()) {
      throw std::invalid_argument("GpsManager: specified com port cannot be empty");
    }
    if (baudRate == 0) {
      throw std::invalid_argument("GpsManager: specified baud rate cannot be zero");
    }
    this->m_comPort = comPort;
    this->m_baudRate = baudRate;
  };

  GpsManager(const std::string &comPort,
    unsigned int baudRate,
    boost::shared_ptr<DatabaseManager> databaseManager,
    std::function<void(const GpsUpdate&)> gpsUpdateCallback):m_gpsUpdateCallback(gpsUpdateCallback) {
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

  void processQueue();

  bool validateFix(const IMUUtils::GpsUpdate &update) const;

  void publishUpdate(IMUUtils::GpsUpdate update);

private:
  std::atomic<bool> m_running;

  std::optional<boost::asio::io_context> m_ioContext;
  std::string m_comPort;
  boost::asio::serial_port m_serialPort;
  unsigned int m_baudRate;

  // read data from serial port into some buffer for future processing
  std::jthread m_readThread;
  boost::asio::streambuf m_readBuf;

  // clear data from serial/read buffer, process into parsed NMEA data, send updates
  std::jthread m_queueThread;
  std::mutex m_queueMutex;
  std::condition_variable m_queueCv;
  std::queue<std::string> m_sentenceQueue;

  NMEAParser m_nmeaParser;

  std::function<void(const GpsUpdate&)> m_gpsUpdateCallback;

  std::mutex m_statsMutex;
  GpsManagerStats m_stats;

  boost::shared_ptr<DatabaseManager> m_databaseManager;

};

#endif // INU_DISPLAY_GPSMANAGER_HPP
