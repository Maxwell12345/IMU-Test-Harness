#include <cmath>

#include "utils.hpp"
#include "IMUManager.hpp"

constexpr auto steadyMin = std::chrono::steady_clock::time_point::min();

std::mutex IMUManager::s_gpsMutex;
std::mutex IMUManager::s_kineticStateMutex;
std::mutex IMUManager::s_latImuTimestampMutex;

bool IMUManager::s_gpsSentToEkf = false;
IMUManager* IMUManager::s_instance = nullptr;
std::optional<GpsUpdate> IMUManager::s_latestGps = std::nullopt;
std::chrono::steady_clock::time_point IMUManager::s_lastImuTimestamp = steadyMin;
IMUUtils::KineticState IMUManager::s_kineticState(0.0, 0.0, 0.0, 0.0);

IMUManager::IMUManager(boost::shared_ptr<DatabaseManager> databaseManager,
                       std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> ekfCallbackImuOnly,
                       std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> ekfCallbackWithGps) {
    if(databaseManager == nullptr) {
        throw std::invalid_argument("databaseManager is nullptr");
    }

    if(!ekfCallbackImuOnly) {
        throw std::invalid_argument("ekfCallbackImuOnly is nullptr");
    }

    if(!ekfCallbackWithGps) {
        throw std::invalid_argument("ekfCallbackWithGps is nullptr");
    }

    m_databaseManager = databaseManager;
    m_ekfCallbackImuOnly = ekfCallbackImuOnly;
    m_ekfCallbackWithGps = ekfCallbackWithGps;
}

void IMUManager::Initialize(boost::shared_ptr<DatabaseManager> databaseManager,
                            std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> ekfCallbackImuOnly,
                            std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> ekfCallbackWithGps) {
    if(s_instance != nullptr) {
        throw std::runtime_error("IMUManager instance already exist");
    }

    s_instance = new IMUManager(databaseManager,
                                ekfCallbackImuOnly,
                                ekfCallbackWithGps);
}

IMUManager& IMUManager::Instance() {
    if(s_instance == nullptr) {
        throw std::runtime_error("IMUManager does not exist");
    }

    return *s_instance;
}

IMUManagerStats IMUManager::GetStats() const {
    std::lock_guard imuGuard(s_instance->m_statsMutex);

    IMUManagerStats statSnapshot;
    statSnapshot = s_instance->m_stats;
    return statSnapshot;
}

std::optional<GpsUpdate> IMUManager::GetLatestGps() {
    std::optional<GpsUpdate> gpsSnapshot = s_latestGps;
    return gpsSnapshot;
}

void IMUManager::UpdateLatestGps(const GpsUpdate& update) {
    std::lock_guard gpsGuard(s_gpsMutex);
    std::lock_guard statsGuard(s_instance->m_statsMutex);

    if(update.valid == false) {
        s_instance->m_stats.gpsRejected++;
        return;
    }

    const unsigned int STALE_TIME_OUT = 5;
    double deltaTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - update.receiveTime).count();
    if(deltaTime > STALE_TIME_OUT) {
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

void IMUManager::SensorCallback(void* cookie, sh2_SensorEvent* event) {
    (void) cookie;

    try {
        if(s_instance == nullptr) {
            throw std::runtime_error("IMUManager instance not initialized");
        }

        if(event == nullptr) {
            std::lock_guard imuGuard(s_instance->m_statsMutex);
            s_instance->m_stats.imuRejected++;
            throw std::invalid_argument("Sh2_SensorEvent was a nullptr");
        }

        sh2_SensorValue_t val{};
        if (sh2_decodeSensorEvent(&val, event) != SH2_OK) {
            std::lock_guard imuGuard(s_instance->m_statsMutex);
            s_instance->m_stats.imuRejected++;
            throw std::runtime_error("There was an error trying to decode Sh2_SensorEvent");
        };

        if(ValidateImuEvent(val) == false) {
            std::lock_guard imuGuard(s_instance->m_statsMutex);
            s_instance->m_stats.imuRejected++;
            throw std::runtime_error("Sensor value out of range or Report type not supported");
        }

        double dtSeconds;
        auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard lastImuTimestampGuard(s_latImuTimestampMutex);
            if (s_lastImuTimestamp == steadyMin) {
                s_lastImuTimestamp = now;
                return;
            } else {
                dtSeconds = std::chrono::duration<double>(now - s_lastImuTimestamp).count();
                s_lastImuTimestamp = now;
            }
        }
        
        std::optional<GpsUpdate> gpsUpdateSnapshot;
        bool gpsSentToEkfSnapshot;
        {
            std::lock_guard gpsMutex(s_gpsMutex);
            gpsUpdateSnapshot = GetLatestGps();
            gpsSentToEkfSnapshot = s_gpsSentToEkf;
        }

        if(!gpsUpdateSnapshot.has_value()) {
            throw std::runtime_error("gps is nullopt");
        }

        Vector6d zImu = BuildImuMeasurementVector(val, gpsUpdateSnapshot.value());

        if(gpsUpdateSnapshot.has_value() && gpsSentToEkfSnapshot == false) {
            Vector6d zGps = BuildGpsMeasurementVector(gpsUpdateSnapshot.value());
            s_instance->m_ekfCallbackWithGps(dtSeconds, zImu, zGps);

            std::lock_guard gpsMutex(s_gpsMutex);
            s_gpsSentToEkf = true;
        } else {
            s_instance->m_ekfCallbackImuOnly(dtSeconds, zImu);
        }

        std::lock_guard imuGuard(s_instance->m_statsMutex);
        s_instance->m_stats.imuAccepted++;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}

bool IMUManager::ValidateImuEvent(const sh2_SensorValue& sensorValue) {
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

Vector6d IMUManager::BuildImuMeasurementVector(const sh2_SensorValue& sensorValue, const GpsUpdate& gps) {
    if(sensorValue.sensorId != SH2_ACCELEROMETER) {
        throw std::invalid_argument("Sensor Value is not from Accelerometer");
    }

    double latitude = gps.latitude;
    double longitude = gps.longitude;
    double heading = gps.heading.value();

    double globalLinearAccelerationX = IMUUtils::InertialToGlobal_X(heading,
                                                                    sensorValue.un.accelerometer.x,
                                                                    sensorValue.un.accelerometer.y);
    double globalLinearAccelerationY = IMUUtils::InertialToGlobal_Y(heading,
                                                                    sensorValue.un.accelerometer.x,
                                                                    sensorValue.un.accelerometer.y);

    double globalGeoAccelerationX = IMUUtils::Convert_Global_X_to_DegPerS2(latitude,
                                                                           globalLinearAccelerationX);
    double globalGeoAccelerationY = IMUUtils::Convert_Global_Y_to_DegPerS2(globalLinearAccelerationY);

    IMUUtils::KineticState kineticState;
    {
        std::lock_guard kineticStateGuard(s_kineticStateMutex);
        kineticState = IMUUtils::CalculateKineticUpdate(s_kineticState,
                                                        globalGeoAccelerationX,
                                                        globalGeoAccelerationY);
        s_kineticState = kineticState;
    }

    Vector6d imuVector = {
        0.0,
        0.0,
        kineticState.speedEastWest,
        kineticState.speedNorthSouth,
        kineticState.accelerationEastWest,
        kineticState.accelerationNorthSouth
    };
    return imuVector;
}
