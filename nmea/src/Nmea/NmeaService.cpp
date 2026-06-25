#include <iostream>

#include "NmeaService.hpp"

#include "SerialPortBase.hpp"
#include "SerialComService.hpp"

NmeaService::NmeaService(std::string path,
                         unsigned int baudRate,
                         std::unique_ptr<SerialPortBase> port):
                         m_running(false),
                         m_nmea("") {
    auto f = [this](SerialPortBase& port){
        this->Callback(port);
    };

    m_serialComService = std::make_unique<SerialComService>(path,
                                                            baudRate,
                                                            std::move(port));
    m_serialComService->InstallCallback(f);
}

NmeaService::~NmeaService() {
    Stop();
}

void NmeaService::Start() {
    if(m_running.exchange(true)) {
        return;
    }

    m_serialComService->Start();
}

void NmeaService::Stop() {
    if(!m_running.exchange(false)) {
        return;
    }

    m_serialComService->Stop();
}

void NmeaService::Callback(SerialPortBase& port) {
    std::lock_guard nmeaGuard(m_nmeaMutex);
    port.ReadUntil(m_nmea, "\n");
    std::cout << "[DEBUG] " << m_nmea << std::endl;
}