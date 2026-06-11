/******************************************************************************
 * File:             GpsManager.cpp
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 *
 ******************************************************************************/

#include "GpsManager.hpp"
#include "DatabaseManager.hpp"

#define NMEA_PORT "/dev/ttyACM0"

GpsManager::GpsManager(boost::shared_ptr<DatabaseManager> databaseManager):
    m_nmeaReader(NMEA_PORT, 9600),
    m_databaseManager(databaseManager) {};

void GpsManager::InstallCallback(std::function<void(const GpsUpdate&)> callback) {
    std::lock_guard<std::mutex> lock(m_callbackMutex);
    m_callback = callback;
    m_isCallbackInstalled = true;
}

void GpsManager::Start() {
    if (m_running.exchange(true)) return;

    if (!m_nmeaReader.open()) {
        m_running = false;
        return;
    }

    m_thread = std::thread([this]() {
        NmeaMessage msg;

        while (m_running) {
            if (!m_nmeaReader.readMessage(msg)) continue;
            if (!msg.validChecksum) continue;

            GpsUpdate update = BuildGpsUpdate(msg);
            m_databaseManager->EnqueueGpsUpdate(update);

            std::function<void(const GpsUpdate&)> callback;

            {
                std::lock_guard<std::mutex> lock(m_callbackMutex);
                if (!m_isCallbackInstalled) continue;
                callback = m_callback;
            }

            callback(update);
        }
    });
}

void GpsManager::Stop() {
    if (!m_running.exchange(false)) return;

    m_nmeaReader.close();

    if (m_thread.joinable()) {
        m_thread.join();
    }
}

GpsUpdate GpsManager::BuildGpsUpdate(const NmeaMessage& msg) {
    GpsUpdate update;

    update.receiveTime = std::chrono::steady_clock::now();
    update.latitude = msg.lat;
    update.longitude = msg.lon;
    update.fixQuality = static_cast<uint8_t>(msg.fixQuality);
    update.numSatellites = static_cast<uint8_t>(msg.numSatellites);
    update.hdop = msg.hdop;
    update.valid = msg.validChecksum && msg.validFix;

    if (msg.type == "GPRMC" || msg.type == "GNRMC") {
        update.heading = msg.courseDeg;
    }

    return update;
}
