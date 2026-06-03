#include <cmath>

#include "utils.hpp"
#include "IMUManager.hpp"

bool IMUManager::s_gpsSentToEkf = false;
std::mutex IMUManager::s_gpsMutex;
IMUManager* IMUManager::s_instance = nullptr;
std::optional<GpsUpdate> IMUManager::s_latestGps = std::nullopt;
std::chrono::steady_clock::time_point IMUManager::s_lastImuTimestamp;
IMUUtils::KineticState IMUManager::s_kineticState(0.0, 0.0, 0.0, 0.0);

bool IMUManager::ValidateImuEvent(const sh2_SensorValue_t& sensorValue) {
    // if(event == nullptr) {
    //     return false;
    // } 

    // sh2_SensorValue_t val{};
    // if (sh2_decodeSensorEvent(&val, event) != SH2_OK) {
    //     return false;
    // };

    switch (sensorValue.sensorId) {
        case SH2_ACCELEROMETER:
            if( IsInvalidRange(sensorValue.timestamp) ||
                IsInvalidRange(sensorValue.un.accelerometer.x) ||
                IsInvalidRange(sensorValue.un.accelerometer.y) ||
                IsInvalidRange(sensorValue.un.accelerometer.z) ||
                IsInvalidRange(static_cast<uint8_t>(sensorValue.status & 0x03))) {
                return false;
            }
            break;

        case SH2_LINEAR_ACCELERATION:
            if( IsInvalidRange(sensorValue.timestamp) ||
                IsInvalidRange(sensorValue.un.linearAcceleration.x) ||
                IsInvalidRange(sensorValue.un.linearAcceleration.y) ||
                IsInvalidRange(sensorValue.un.linearAcceleration.z) ||
                IsInvalidRange(static_cast<uint8_t>(sensorValue.status & 0x03))) {
                return false;
            }
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            if( IsInvalidRange(sensorValue.timestamp) ||
                IsInvalidRange(sensorValue.un.gyroscope.x) ||
                IsInvalidRange(sensorValue.un.gyroscope.y) ||
                IsInvalidRange(sensorValue.un.gyroscope.z)) {
                return false;
            }
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            if( IsInvalidRange(sensorValue.timestamp) ||
                IsInvalidRange(sensorValue.un.magneticField.x) ||
                IsInvalidRange(sensorValue.un.magneticField.y) ||
                IsInvalidRange(sensorValue.un.magneticField.z) ||
                IsInvalidRange(static_cast<uint8_t>(sensorValue.status & 0x03))) {
                return false;
            }
            break;

        case SH2_ROTATION_VECTOR:
            if( IsInvalidRange(sensorValue.timestamp) ||
                IsInvalidRange(sensorValue.un.rotationVector.i) ||
                IsInvalidRange(sensorValue.un.rotationVector.j) ||
                IsInvalidRange(sensorValue.un.rotationVector.k) ||
                IsInvalidRange(sensorValue.un.rotationVector.real) ||
                IsInvalidRange(sensorValue.un.rotationVector.accuracy)) {
                return false;
            }
            break;

        case SH2_GAME_ROTATION_VECTOR:
            if( IsInvalidRange(sensorValue.timestamp) ||
                IsInvalidRange(sensorValue.un.gameRotationVector.i) ||
                IsInvalidRange(sensorValue.un.gameRotationVector.j) ||
                IsInvalidRange(sensorValue.un.gameRotationVector.k) ||
                IsInvalidRange(sensorValue.un.gameRotationVector.real)) {
                return false;
            }
            break;

        default:
            return false;
    }

    return true;
};

Vector6d IMUManager::BuildGpsMeasurementVector(const GpsUpdate& gps) {
    Vector6d gpsVector = {gps.longitude, gps.latitude, 0, 0, 0, 0};
    return gpsVector;
}

Vector6d IMUManager::BuildImuMeasurementVector(const sh2_SensorValue_t& sensorValue) {
    if(sensorValue.sensorId != SH2_ACCELEROMETER) {
        throw std::invalid_argument("Sensor Value is not from Accelerometer");
    }

    if(!s_latestGps.has_value()) {
        throw std::runtime_error("s_latestGps is nullopt");
    }

    if(!s_latestGps.value().heading.has_value()) {
        throw std::runtime_error("s_latestGps has no heading");
    }

    GpsUpdate currentGps;
    {
        std::lock_guard guard(s_gpsMutex);
        currentGps = s_latestGps.value();
    }

    double latitude = currentGps.latitude;
    double longitude = currentGps.longitude;
    double heading = currentGps.heading.value();

    double globalLinearAccelerationX = IMUUtils::InertialToGlobal_X(heading,
                                                                    sensorValue.un.accelerometer.x,
                                                                    sensorValue.un.accelerometer.y);
    double globalLinearAccelerationY = IMUUtils::InertialToGlobal_Y(heading,
                                                                    sensorValue.un.accelerometer.x,
                                                                    sensorValue.un.accelerometer.y);

    double globalGeoAccelerationX = IMUUtils::Convert_Global_X_to_DegPerS2(latitude,
                                                                           globalLinearAccelerationX);
    double globalGeoAccelerationY = IMUUtils::Convert_Global_Y_to_DegPerS2(globalLinearAccelerationY);
    s_kineticState = IMUUtils::CalculateKineticUpdate(s_kineticState,
                                                      globalGeoAccelerationX,
                                                      globalGeoAccelerationY);

    Vector6d imuVector = {
        0.0,
        0.0,
        s_kineticState.speedEastWest,
        s_kineticState.speedNorthSouth,
        s_kineticState.accelerationEastWest,
        s_kineticState.accelerationNorthSouth
    };
    return imuVector;
}

void IMUManager::UpdateLatestGps(const GpsUpdate& update) {
    std::lock_guard guard(s_gpsMutex);

    if(update.valid == false) {
        s_instance->m_stats.gpsRejected++;
        return;
    }

    double deltaTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - update.receiveTime).count();
    if(deltaTime > 5) {
        s_instance->m_stats.gpsRejected++;
        return;
    }

    if(s_latestGps.has_value() && update.gpsTimestampMs <= s_latestGps->gpsTimestampMs){
        s_instance->m_stats.gpsRejected++;
        return;
    }

    s_gpsSentToEkf = false;
    s_latestGps = update;
    s_instance->m_stats.gpsAccepted++;
}