/******************************************************************************
 * File:             GpsManager.cpp
 * 
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       6/10/26 9:47 AM
 * Description:      Delivers live GPS updates and maps parsed NMEA messages
 *                   with caller-provided measurement timestamps.
 *
 ******************************************************************************/

#include "GpsManager.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>

#define NMEA_COM_PORT "/dev/ttyTEST"

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

            const uint64_t receiveTimestampNs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            GpsUpdate update = BuildGpsUpdate(msg, receiveTimestampNs);

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

GpsUpdate GpsManager::BuildGpsUpdate(const NmeaMessage& msg, uint64_t databaseTimestampNs) {
    const uint64_t databaseTimestampMs = databaseTimestampNs / 1'000'000ULL;
    if (databaseTimestampMs > std::numeric_limits<uint32_t>::max()) {
        throw std::out_of_range("Database GPS timestamp does not fit gpsTimestampMs");
    }

    GpsUpdate update{};

    update.receiveTime = std::chrono::steady_clock::now();
    update.timestamp = static_cast<double>(databaseTimestampNs) / 1'000'000'000.0;
    update.latitude = msg.lat;
    update.longitude = msg.lon;
    update.fixQuality = static_cast<uint8_t>(msg.fixQuality);
    update.numSatellites = static_cast<uint8_t>(msg.numSatellites);
    update.hdop = msg.hdop;
    update.gpsTimestampMs = static_cast<uint32_t>(databaseTimestampMs);
    update.valid = msg.validChecksum && msg.validFix;

    if (msg.type == "GPRMC" || msg.type == "GNRMC") {
        update.heading = msg.courseDeg;
    }

    return update;
}
