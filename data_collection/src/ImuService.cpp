#include "ImuService.hpp"

#include "BrokerService.hpp"
#include "ImuRecords.hpp"
#include "Logger.hpp"
#include "bno085_hal.hpp"

extern "C" {
#include "sh2_SensorValue.h"
}

#include <chrono>
#include <cstdint>
#include <stdexcept>

namespace {

constexpr uint32_t kAccelIntervalUs = 2'500;
constexpr uint32_t kGyroIntervalUs = 2'500;
constexpr uint32_t kLinearAccelIntervalUs = 2'500;
constexpr uint32_t kRotationVectorIntervalUs = 2'500;

int64_t nowEpochNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void enableSensor(sh2_SensorId_t sensorId, uint32_t intervalUs) {
    sh2_SensorConfig_t config{};
    config.reportInterval_us = intervalUs;
    if (sh2_setSensorConfig(sensorId, &config) != SH2_OK) {
        LOG_WARN("ImuService") << "Failed to enable sensor id=" << static_cast<int>(sensorId);
    } else {
        LOG_DEBUG("ImuService") << "Enabled sensor id=" << static_cast<int>(sensorId)
                                << " interval=" << intervalUs << "us";
    }
}

}  // namespace

ImuService::ImuService(BrokerService& broker) : m_broker(broker) {}

ImuService::~ImuService() { stop(); }

void ImuService::start() {
    if (m_running.exchange(true)) {
        return;
    }

    LOG_INFO("ImuService") << "Opening BNO085 over I2C";
    m_hal = bno085_hal_create();
    if (sh2_open(&m_hal, nullptr, nullptr) != SH2_OK) {
        m_running.store(false);
        LOG_ERROR("ImuService") << "sh2_open failed — check wiring and I2C address";
        throw std::runtime_error("sh2_open failed — check wiring and I2C address");
    }

    sh2_setSensorCallback(&ImuService::sensorCallback, this);

    enableSensor(SH2_ACCELEROMETER, kAccelIntervalUs);
    enableSensor(SH2_GYROSCOPE_CALIBRATED, kGyroIntervalUs);
    enableSensor(SH2_LINEAR_ACCELERATION, kLinearAccelIntervalUs);
    enableSensor(SH2_ROTATION_VECTOR, kRotationVectorIntervalUs);

    m_serviceThread = std::thread(&ImuService::serviceLoop, this);
    LOG_INFO("ImuService") << "Started, streaming accelerometer, gyroscope, "
                              "linear acceleration, rotation vector";
}

void ImuService::stop() {
    if (!m_running.exchange(false)) {
        return;
    }
    if (m_serviceThread.joinable()) {
        m_serviceThread.join();
    }
    sh2_close();
    LOG_INFO("ImuService") << "Stopped";
}

void ImuService::serviceLoop() {
    while (m_running.load(std::memory_order_relaxed)) {
        sh2_service();
    }
}

void ImuService::sensorCallback(void* cookie, sh2_SensorEvent_t* event) {
    auto* self = static_cast<ImuService*>(cookie);
    if (self != nullptr) {
        self->handleSensorEvent(event);
    }
}

void ImuService::handleSensorEvent(sh2_SensorEvent_t* event) {
    sh2_SensorValue_t value{};
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        return;
    }

    const int64_t timestampNs = nowEpochNs();
    const uint64_t sensorTimestampUs = value.timestamp;
    const uint8_t calibAccuracy = static_cast<uint8_t>(value.status & 0x03);

    switch (value.sensorId) {
        case SH2_ACCELEROMETER: {
            AccelerometerRecord record{};
            record.timestampNs = timestampNs;
            record.sensorTimestampUs = sensorTimestampUs;
            record.x = value.un.accelerometer.x;
            record.y = value.un.accelerometer.y;
            record.z = value.un.accelerometer.z;
            record.calibAccuracy = calibAccuracy;
            m_broker.enqueueAccelerometer(record);
            break;
        }
        case SH2_GYROSCOPE_CALIBRATED: {
            GyroscopeRecord record{};
            record.timestampNs = timestampNs;
            record.sensorTimestampUs = sensorTimestampUs;
            record.x = value.un.gyroscope.x;
            record.y = value.un.gyroscope.y;
            record.z = value.un.gyroscope.z;
            record.calibAccuracy = calibAccuracy;
            m_broker.enqueueGyroscope(record);
            break;
        }
        case SH2_LINEAR_ACCELERATION: {
            LinearAccelRecord record{};
            record.timestampNs = timestampNs;
            record.sensorTimestampUs = sensorTimestampUs;
            record.x = value.un.linearAcceleration.x;
            record.y = value.un.linearAcceleration.y;
            record.z = value.un.linearAcceleration.z;
            record.calibAccuracy = calibAccuracy;
            m_broker.enqueueLinearAccel(record);
            break;
        }
        case SH2_ROTATION_VECTOR: {
            RotationVectorRecord record{};
            record.timestampNs = timestampNs;
            record.sensorTimestampUs = sensorTimestampUs;
            record.i = value.un.rotationVector.i;
            record.j = value.un.rotationVector.j;
            record.k = value.un.rotationVector.k;
            record.real = value.un.rotationVector.real;
            record.accuracyRad = value.un.rotationVector.accuracy;
            record.calibAccuracy = calibAccuracy;
            m_broker.enqueueRotationVector(record);
            break;
        }
        default:
            break;
    }
}
