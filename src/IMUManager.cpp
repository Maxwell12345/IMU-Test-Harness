#include <cmath>

#include "utils.hpp"
#include "IMUManager.hpp"
#include "MagneticDeclination.hpp"

constexpr auto steadyMin = std::chrono::steady_clock::time_point::min();

std::mutex IMUManager::s_gpsMutex;
std::mutex IMUManager::s_kineticStateMutex;
std::mutex IMUManager::s_lastImuEkfInvocationMutex;
std::mutex IMUManager::s_imuValueMutex;

sh2_RotationVectorWAcc IMUManager::s_imuRotationVector;    
sh2_Accelerometer IMUManager::s_imuLinearAcceleration;

bool IMUManager::s_gpsSentToEkf = false;
bool IMUManager::s_imuRotationVectorReady = false;               
bool IMUManager::s_imuLinearAccelerationReady = false;  

IMUManager* IMUManager::s_instance = nullptr;
std::optional<GpsUpdate> IMUManager::s_latestGps = std::nullopt;
std::chrono::steady_clock::time_point IMUManager::s_lastImuEkfIvocation = steadyMin;

IMUUtils::KineticState IMUManager::s_kineticState(std::chrono::steady_clock::now(), 0.0, 0.0, 0.0, 0.0);

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

    m_magneticDeclination.LoadCOF("./WMM.COF");
}

IMUManager::~IMUManager() {
    s_imuRotationVectorReady = false;
    s_imuLinearAccelerationReady = false;
    s_imuRotationVector = {};
    s_imuLinearAcceleration = {};
    s_lastImuEkfIvocation = {};
    s_gpsSentToEkf = false;
    s_latestGps = std::nullopt;
    s_kineticState = {};
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

void IMUManager::Deinitialize() {
    delete s_instance;
    s_instance = nullptr;
}

IMUManager& IMUManager::Instance() {
    if(s_instance == nullptr) {
        throw std::runtime_error("s_instance does not exist, need to run IMUManager::Initialize()");
    }

    return *s_instance;
}

IMUManagerStats IMUManager::GetStats() const {
    if(s_instance == nullptr) {
        throw std::runtime_error("s_instance does not exist, need to run IMUManager::Initialize()");
    }

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
    if(s_instance == nullptr) {
        throw std::runtime_error("s_instance does not exist, need to run IMUManager::Initialize()");
    }

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
            throw std::runtime_error("s_instance does not exist, need to run IMUManager::Initialize()");
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

        StoreImuValue(val);

        {
            std::lock_guard lastImuTimestampGuard(s_lastImuEkfInvocationMutex);
            if (s_lastImuEkfIvocation == steadyMin) {
                s_lastImuEkfIvocation = std::chrono::steady_clock::now();
                return;
            }
        }

        sh2_RotationVectorWAcc rotationVectorSnapshot;
        sh2_Accelerometer linearAccelerationSnapshot;
        {
            std::lock_guard imuValueGuard(s_imuValueMutex);
            if(s_imuRotationVectorReady == false || s_imuLinearAccelerationReady == false) {
                return;
            }

            rotationVectorSnapshot.i = s_imuRotationVector.i;
            rotationVectorSnapshot.j = s_imuRotationVector.j;
            rotationVectorSnapshot.k = s_imuRotationVector.k;
            rotationVectorSnapshot.real = s_imuRotationVector.real;

            linearAccelerationSnapshot.x = s_imuLinearAcceleration.x;
            linearAccelerationSnapshot.y = s_imuLinearAcceleration.y;
            linearAccelerationSnapshot.z = s_imuLinearAcceleration.z;
        }
        
        std::optional<GpsUpdate> gpsUpdateSnapshot;
        bool gpsSentToEkfSnapshot;
        {
            std::lock_guard gpsMutex(s_gpsMutex);
            gpsUpdateSnapshot = GetLatestGps();
            gpsSentToEkfSnapshot = s_gpsSentToEkf;
        }

        // This ymdNow is the obtain current year to calculate Magnetic declination
        auto ymdNow = std::chrono::system_clock::now();
        const std::chrono::year_month_day ymd{std::chrono::floor<std::chrono::days>(ymdNow)};
        Vector6d zImu = BuildImuMeasurementVector(rotationVectorSnapshot,
                                                  linearAccelerationSnapshot,
                                                  gpsUpdateSnapshot.value(),
                                                  static_cast<int>(ymd.year()));

        auto now = std::chrono::steady_clock::now();
        double dtSeconds = std::chrono::duration<double>(now - s_lastImuEkfIvocation).count();
        s_lastImuEkfIvocation = now;

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
        // case SH2_ACCELEROMETER:
        //     if( IsInvalidRange(sensorValue.timestamp) ||
        //         IsInvalidRange(sensorValue.un.accelerometer.x) ||
        //         IsInvalidRange(sensorValue.un.accelerometer.y) ||
        //         IsInvalidRange(sensorValue.un.accelerometer.z) ||
        //         IsInvalidRange(static_cast<uint8_t>(sensorValue.status & 0x03))) {
        //         return false;
        //     }
        //     break;

        case SH2_LINEAR_ACCELERATION:
            if( IsInvalidRange(sensorValue.un.linearAcceleration.x) ||
                IsInvalidRange(sensorValue.un.linearAcceleration.y) ||
                IsInvalidRange(sensorValue.un.linearAcceleration.z)) {
                return false;
            }
            break;

        // case SH2_GYROSCOPE_CALIBRATED:
        //     if( IsInvalidRange(sensorValue.timestamp) ||
        //         IsInvalidRange(sensorValue.un.gyroscope.x) ||
        //         IsInvalidRange(sensorValue.un.gyroscope.y) ||
        //         IsInvalidRange(sensorValue.un.gyroscope.z)) {
        //         return false;
        //     }
        //     break;

        // case SH2_MAGNETIC_FIELD_CALIBRATED:
        //     if( IsInvalidRange(sensorValue.timestamp) ||
        //         IsInvalidRange(sensorValue.un.magneticField.x) ||
        //         IsInvalidRange(sensorValue.un.magneticField.y) ||
        //         IsInvalidRange(sensorValue.un.magneticField.z) ||
        //         IsInvalidRange(static_cast<uint8_t>(sensorValue.status & 0x03))) {
        //         return false;
        //     }
        //     break;

        case SH2_ROTATION_VECTOR:
            if( IsInvalidRange(sensorValue.un.rotationVector.i) ||
                IsInvalidRange(sensorValue.un.rotationVector.j) ||
                IsInvalidRange(sensorValue.un.rotationVector.k) ||
                IsInvalidRange(sensorValue.un.rotationVector.real) ||
                IsInvalidRange(sensorValue.un.rotationVector.accuracy)) {
                return false;
            }
            break;

        // case SH2_GAME_ROTATION_VECTOR:
        //     if( IsInvalidRange(sensorValue.timestamp) ||
        //         IsInvalidRange(sensorValue.un.gameRotationVector.i) ||
        //         IsInvalidRange(sensorValue.un.gameRotationVector.j) ||
        //         IsInvalidRange(sensorValue.un.gameRotationVector.k) ||
        //         IsInvalidRange(sensorValue.un.gameRotationVector.real)) {
        //         return false;
        //     }
        //     break;

        default:
            return false;
    }

    return true;
};

void IMUManager::StoreImuValue(const sh2_SensorValue& sensorValue) {
    std::lock_guard sensorValueGuard(s_imuValueMutex);
    switch (sensorValue.sensorId) {
        case SH2_ACCELEROMETER:
            break;

        case SH2_LINEAR_ACCELERATION:
            s_imuLinearAccelerationReady = true;
            s_imuLinearAcceleration.x = sensorValue.un.linearAcceleration.x;
            s_imuLinearAcceleration.y = sensorValue.un.linearAcceleration.y;
            s_imuLinearAcceleration.z = sensorValue.un.linearAcceleration.z;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            break;

        case SH2_ROTATION_VECTOR:
            s_imuRotationVectorReady = true;
            s_imuRotationVector.i = sensorValue.un.rotationVector.i;
            s_imuRotationVector.j = sensorValue.un.rotationVector.j;
            s_imuRotationVector.k = sensorValue.un.rotationVector.k;
            s_imuRotationVector.real = sensorValue.un.rotationVector.real;
            break;

        case SH2_GAME_ROTATION_VECTOR:
            break;
    }
}

Vector6d IMUManager::BuildGpsMeasurementVector(const GpsUpdate& gps) {
    Vector6d gpsVector = {gps.longitude, gps.latitude, 0, 0, 0, 0};
    return gpsVector;
}

Vector6d IMUManager::BuildImuMeasurementVector(const sh2_RotationVectorWAcc& rv, const sh2_Accelerometer& la, const GpsUpdate& gps, int currentYear) {
    if(s_instance == nullptr) {
        throw std::runtime_error("s_instance does not exist, need to run IMUManager::Initialize()");
    }

    double latitude = gps.latitude;
    double longitude = gps.longitude;
    const double RADAR_HEIGHT_M = 10;

    double magneticHeading = IMUUtils::Calculate_Magnetic_Heading(rv.real,
                                                                  rv.i,
                                                                  rv.j,
                                                                  rv.k);
    double magneticDeclination = s_instance->m_magneticDeclination.CalculateDeclination(longitude,
                                                                                        latitude,
                                                                                        RADAR_HEIGHT_M,
                                                                                        currentYear);
    double trueHeading = IMUUtils::MagneticToTrueHeading(magneticHeading,
                                                         magneticDeclination);
                                                        
    double trueHeadingRadians = IMUUtils::DegreesToRadians(trueHeading);
    double globalLinearAccelerationX = IMUUtils::InertialToGlobal_X(trueHeadingRadians,
                                                                    la.x,
                                                                    la.y);

    double globalLinearAccelerationY = IMUUtils::InertialToGlobal_Y(trueHeadingRadians,
                                                                    la.x,
                                                                    la.y);

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
