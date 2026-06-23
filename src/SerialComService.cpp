#include <thread>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include "utils.hpp"
#include <iostream>

#include "SerialComService.hpp"
#include "SerialPortBase.hpp"

SerialComService::SerialComService(std::string path,
                                   unsigned int baudRate,
                                   std::unique_ptr<SerialPortBase> serialPort) :
                                   m_running(false),
                                   m_path(path),
                                   m_baudRate(baudRate),
                                   m_serial(std::move(serialPort)) {
    if(VerifyPath(path) == false) {
        std::string str;
        #ifdef _WIN32
            str = "\\\\.\\COM...";
        #else
            str = "/dev/...";
        #endif
        throw std::invalid_argument("The serial port is invalid. Path should be " + str + " with no space");
    }
    ConfigureSerialPort();
}

SerialComService::~SerialComService() {
    Stop();
}

void SerialComService::Start() {
    if(m_running == true) {
        return;
    }

    m_running = true;

    m_runThread = std::jthread([this](std::stop_token st){
        while(m_running) {
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

void SerialComService::InstallCallback(std::function<void(SerialPortBase&)> callback) {
    m_serial->InstallCallback(callback);
}

void SerialComService::ConfigureSerialPort() {
    m_serial->Open(m_path);
    m_serial->SetBaudRate(m_baudRate);
}

bool SerialComService::VerifyPath(const std::string& path) {
    const boost::regex pattern(
        R"(^(?:/dev/\S+|(?:\\\\\.\\)?COM[1-9][0-9]*)$)",
        boost::regex::icase
    );
    return boost::regex_match(path, pattern);
}