#include <iostream>
#include <thread>

#include <boost/filesystem.hpp>

#include "Utils.hpp"
#include "GpsService.hpp"
#include "SerialPort/SerialPortBase.hpp"

GpsService::GpsService(std::string path,
                       unsigned int baudRate,
                       std::unique_ptr<SerialPortBase> serialPort) :
                       m_running(false),
                       m_path(path),
                       m_baudRate(baudRate),
                       m_serial(std::move(serialPort)) {
    try {
        ConfigureSerialPort();
    } catch (std::runtime_error& e) {
        utils::LOG_ERROR(e.what());
        throw;
    }
}

GpsService::~GpsService() {
    Stop();
}

void GpsService::Start() {
    m_running = true;

    m_runThread = std::jthread([this](std::stop_token st){
        while(m_running) {
            std::string nmeaSentence = m_serial->ReadLine();
            utils::LOG_INFO(nmeaSentence, true);
        }
    
        m_serial->Close();
    });
}

void GpsService::Stop() {
    m_running = false;
    m_runThread.request_stop();

    if (m_runThread.joinable()) {
        m_runThread.join();
        utils::LOG_INFO("Joined Serial Reading thread");
    }
}

void GpsService::ConfigureSerialPort() {
    m_serial->Open(m_path);
    m_serial->SetBaudRate(m_baudRate);
}