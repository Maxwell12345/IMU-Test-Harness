#include <thread>
#include <boost/filesystem.hpp>
#include <iostream>

#include "SerialComService.hpp"
#include "SerialPort/SerialPortBase.hpp"

SerialComService::SerialComService(std::string path,
                                   unsigned int baudRate,
                                   std::unique_ptr<SerialPortBase> serialPort) :
                                   m_running(false),
                                   m_path(path),
                                   m_baudRate(baudRate),
                                   m_serial(std::move(serialPort)) {
    // TODO: Remove try catch in both serial com service classes and add exception to docs regarding constructor and ConfigureSerialPort
    try {
        ConfigureSerialPort();
    } catch (std::runtime_error& e) {
        throw;
    }
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
        std::cout << "Joined Serial Reading thread" << std::endl;
    }
}

void SerialComService::ConfigureSerialPort() {
    m_serial->Open(m_path);
    m_serial->SetBaudRate(m_baudRate);
}