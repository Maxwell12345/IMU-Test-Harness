#include <iostream>

#include "NmeaService.hpp"

#include "SerialPortBase.hpp"
#include "SerialComService.hpp"
#include "MulticastService.hpp"

NmeaService::NmeaService(std::unique_ptr<SerialComService> serialComService,
                         std::unique_ptr<MulticastService> multicastService):
                         m_running(false),
                         m_nmea("") {
    auto f = [this](SerialPortBase& port){
        this->Callback(port);
    };

    m_serialComService = std::move(serialComService);
    m_serialComService->InstallCallback(f);

    m_multicastService = std::move(multicastService);
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
    std::cout << "[DEBUG] " << m_multicastService->Send(m_nmea) << " bytes sent\n";
}