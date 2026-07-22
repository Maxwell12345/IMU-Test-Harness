/******************************************************************************
 * File:             GpsManager.cpp
 * 
 * Author:           Atkinson Maj Brian R.
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 *
 ******************************************************************************/

#include "GpsManager.hpp"

#define NMEA_COM_PORT "/dev/ttyUSB0"

GpsManager::GpsManager()
    : m_nmeaReader(NMEA_COM_PORT, 115200) {}

void GpsManager::InstallCallback(std::function<void(const GpsUpdate&)> callback) {
    std::lock_guard<std::mutex> lock(m_callbackMutex);
    m_callback = callback;
    m_isCallbackInstalled = true;
}

#include <iostream>
void GpsManager::Start() {
    if (m_running.exchange(true)) return;

    m_nmeaReader.Start();

    m_thread = std::thread([this]() {
        NmeaMessage msg;

        while (m_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (m_nmeaReader.GetNmeaMessageReady() == false) continue;

            msg = m_nmeaReader.GetNmeaMessage();
            // if (msg.validChecksum == false) continue;

            GpsUpdate update = BuildGpsUpdate(msg);

            if (update.latitude == 0 || update.longitude == 0) {
                continue;
            }

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

    m_nmeaReader.Stop();

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
