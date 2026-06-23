#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <thread>

#include "utils.hpp"
#include <iostream>

#include "SerialComService.hpp"
#include "SerialPortBase.hpp"

SerialComService::SerialComService(std::string path, unsigned int baudRate, std::shared_ptr<SerialPortBase> serialPort)
    : m_running(false), m_path(path), m_baudRate(baudRate), m_serial(std::move(serialPort)) {
  if (VerifyPath(path) == false) {
    throw std::invalid_argument("The serial port is invalid. Path should be \"/dev/...\" with no space");
  }
  //   if (!serialPort) {
  //     throw std::invalid_argument("SerialCommService requires a non-null serial port");
  //   }
  // ConfigureSerialPort();
  // FIX
}

SerialComService::~SerialComService() { Stop(); }

void SerialComService::Start() {
  if (m_running == true) {
    return;
  }

  m_running = true;

  m_runThread = std::jthread([this](std::stop_token st) {
    while (m_running) {
      m_serial->Callback();
    }

    m_serial->Close();
  });
}

void SerialComService::Stop() {
  m_running = false;
  m_runThread.request_stop();

  if (m_runThread.joinable()) {
    m_runThread.join();
    // LOG_INFO HERE
    std::cout << "[INFO]" << "Joined Serial Com thread" << std::endl;
  }
}

void SerialComService::ConfigureSerialPort() {
  m_serial->Open(m_path);
  m_serial->SetBaudRate(m_baudRate);
}

bool SerialComService::VerifyPath(const std::string &path) {
  const boost::regex pattern(R"(^/dev/\S+$)");
  return boost::regex_match(path, pattern);
}