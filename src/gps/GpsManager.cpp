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
    this->m_serialPort(this->m_ioContext, this->m_comPort);
    this->m_serialPort.set_option(boost::asio::serial_port_base::baud_rate(this->m_baudRate));

    this->m_readThread = std::jthread([this]() {
        
    });

    this->m_running.store(true);
}

void GpsManager::stop() {
}

bool GpsManager::isRunning() const {
}

GpsManagerStats GpsManager::getStats() const {
}

void GpsManager::readLoop(std::stop_token stopToken) {
}

std::optional<IMUUtils::GpsUpdate> GpsManager::parseNmea(const std::string &sentence) {
}

bool GpsManager::validateFix(const IMUUtils::GpsUpdate &update) const {
}

void GpsManager::publishUpdate(IMUUtils::GpsUpdate update) {
}
