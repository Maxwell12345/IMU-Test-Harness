#include <iostream>
#include <string>
#include <boost/regex.hpp>

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

    // NMEA Standard 0183 sentence ends with CRLF
    port.ReadUntil(m_nmea, "\r\n");

    if(!IsValidNmea(m_nmea)) {
        return;
    }
    if(!IsValidChecksum(m_nmea)) {
        return;
    }
    std::cout << "[DEBUG] " << m_multicastService->Send(m_nmea) << " bytes sent\n";
}

bool NmeaService::IsValidChecksum(const std::string& nmea) {
    char checksum = nmea[1];
    for(size_t i = 2; i < nmea.size(); i++) {
        if(nmea[i] == '*') {
            break;
        }
        checksum ^= nmea[i];
    }

    int realChecksum = std::stoi(nmea.substr(nmea.length() - 2, 2), nullptr, 16);

    return realChecksum == checksum;
}

bool NmeaService::IsValidNmea(const std::string& nmea) {
    std::string patternStr = R"(^\$([A-Z]{2}[A-Z0-9]{1,5}),([^*]*)\*([0-9A-Fa-f]{2})$)";
    const boost::regex pattern(patternStr, boost::regex::icase);
    return boost::regex_match(nmea, pattern);
}