#include <cmath>
#include <cstdio>

#include "utils.hpp"
#include "IMUManager.hpp"
#include "MagneticDeclination.hpp"

constexpr auto steadyMin = std::chrono::steady_clock::time_point::min();

std::mutex IMUManager::m_sGpsMutex;
std::mutex IMUManager::m_sKineticStateMutex;

std::atomic<sh2_RotationVectorWAcc> IMUManager::m_sImuRotationVector;    
std::atomic<sh2_Accelerometer> IMUManager::m_sImuLinearAcceleration;

std::atomic<bool> IMUManager::m_sGpsSentToEkf = false;
std::atomic<bool> IMUManager::m_sImuRotationVectorReady = false;               
std::atomic<bool> IMUManager::m_sImuLinearAccelerationReady = false;  

std::optional<GpsUpdate> IMUManager::m_sLatestGps = std::nullopt;

std::atomic<uint64_t> IMUManager::m_sLastAccelerationVectorMachineTime = 0;
std::atomic<uint64_t> IMUManager::m_sLastRotationVectorMachineTime = 0;
std::atomic<uint64_t> IMUManager::m_sLastEKFMachineTime = 1e-4;

IMUUtils::KineticState IMUManager::m_sKineticState(steadyMin, 0.0, 0.0, 0.0, 0.0);

IMUManagerStats IMUManager::m_sStats;

MagneticDeclination IMUManager::m_sMagneticDeclination;
boost::shared_ptr<DatabaseManager> IMUManager::m_sDatabaseManager;

std::atomic<bool> IMUManager::m_sEkfInstalled = false; 
std::function<void(double, Vector6d&)> IMUManager::m_sEkfCallbackImuOnly;
std::function<void(double, Vector6d&, Vector6d&)> IMUManager::m_sEkfCallbackWithGps;

namespace {
    std::FILE *file = std::fopen("output.csv", "w");
}

IMUManager::IMUManager(boost::shared_ptr<DatabaseManager> databaseManager) {
    if(databaseManager == nullptr) {
        throw std::invalid_argument("databaseManager is nullptr");
    }

    m_sDatabaseManager = databaseManager;

    m_sMagneticDeclination.LoadCOF("./WMM.COF");

    m_sImuRotationVector = {0, 0, 0, 0, 0};    
    m_sImuLinearAcceleration = {0, 0, 0};
}

IMUManager::~IMUManager() {
    m_sImuRotationVectorReady = false;
    m_sImuLinearAccelerationReady = false;
    m_sGpsSentToEkf = false;
    m_sLatestGps = std::nullopt;
    m_sKineticState = {};
    
    IMUManagerStats reset;
    m_sStats = reset;
}

void IMUManager::InstallEkf(std::function<void(double, Vector6d&)> ekfCallbackImuOnly,
                std::function<void(double, Vector6d&, Vector6d&)> ekfCallbackWithGps) {

    if(!ekfCallbackImuOnly) {
        throw std::invalid_argument("ekfCallbackImuOnly is nullptr");
    }

    if(!ekfCallbackWithGps) {
        throw std::invalid_argument("ekfCallbackWithGps is nullptr");
    }

    m_sEkfCallbackImuOnly = ekfCallbackImuOnly;
    m_sEkfCallbackWithGps = ekfCallbackWithGps;
    m_sEkfInstalled = true;
}

IMUManagerStats IMUManager::GetStats() {

    IMUManagerStats statSnapshot;
    statSnapshot = m_sStats;

    return statSnapshot;
}

std::optional<GpsUpdate> IMUManager::GetLatestGps() {
    std::lock_guard gpsMutex(m_sGpsMutex);
    std::optional<GpsUpdate> gpsSnapshot = m_sLatestGps;
    return gpsSnapshot;
}

void IMUManager::UpdateLatestGps(const GpsUpdate& update) {
    std::lock_guard gpsGuard(m_sGpsMutex);

    if(update.valid == false) {
        m_sStats.gpsRejected++;
        return;
    }

    const unsigned int STALE_TIME_OUT = 5;
    double deltaTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - update.receiveTime).count();
    if(deltaTime > STALE_TIME_OUT) {
        m_sStats.gpsRejected++;
        return;
    }

    if(m_sLatestGps.has_value() && update.gpsTimestampMs <= m_sLatestGps->gpsTimestampMs){
        m_sStats.gpsRejected++;
        return;
    }

    m_sGpsSentToEkf = false;
    m_sLatestGps = update;
    m_sStats.gpsAccepted++;
}

void IMUManager::SensorCallback(void* cookie, sh2_SensorEvent* event) {
    (void) cookie;

    try {
        if(event == nullptr) {
            m_sStats.imuRejected++;
            throw std::invalid_argument("Sh2_SensorEvent was a nullptr");
        }

        sh2_SensorValue_t val{};
        if (sh2_decodeSensorEvent(&val, event) != SH2_OK) {
            m_sStats.imuRejected++;
            throw std::runtime_error("There was an error trying to decode Sh2_SensorEvent");
        };

        if(ValidateImuEvent(val) == false) {
            m_sStats.imuRejected++;
            throw std::runtime_error("Sensor value out of range or Report type not supported");
        }

        StoreImuValue(val);
        m_sStats.imuAccepted++;
        
        if(!(m_sImuRotationVectorReady && m_sImuLinearAccelerationReady)) {
            return;
        }

        if(m_sEkfInstalled == false) {
            return;
        }

        sh2_RotationVectorWAcc rotationVectorSnapshot = m_sImuRotationVector.load();
        sh2_Accelerometer linearAccelerationSnapshot = m_sImuLinearAcceleration.load();

        bool gpsSentToEkfSnapshot;
        std::optional<GpsUpdate> gpsUpdateSnapshot;
        {
            std::lock_guard sentToEkfGuard(m_sGpsMutex);
            gpsUpdateSnapshot    = m_sLatestGps;
            gpsSentToEkfSnapshot = m_sGpsSentToEkf.load();
        }

        if(gpsUpdateSnapshot.has_value() == false)
        {
            throw std::runtime_error("No GPS data ever recorded");
            return;
        }

        // This ymdNow is the obtain current year to calculate Magnetic declination
        auto ymdNow = std::chrono::system_clock::now();
        const std::chrono::year_month_day ymd{std::chrono::floor<std::chrono::days>(ymdNow)};
        Vector6d zImu = BuildImuMeasurementVector(rotationVectorSnapshot,
                                                  linearAccelerationSnapshot,
                                                  gpsUpdateSnapshot.value(),
                                                  static_cast<int>(ymd.year()));

        
        uint64_t accHwTime = m_sLastAccelerationVectorMachineTime.load();
        uint64_t rotHwTime = m_sLastRotationVectorMachineTime.load();
        uint64_t lastEKFHwTime = m_sLastEKFMachineTime.load();
        uint64_t oldestHwTime = std::min(accHwTime, rotHwTime);
        
        double dtSeconds = static_cast<double>(oldestHwTime - lastEKFHwTime) / 1e6;

        m_sLastEKFMachineTime.store(oldestHwTime);

        if(gpsSentToEkfSnapshot == false) {
            Vector6d zGps = BuildGpsMeasurementVector(gpsUpdateSnapshot.value());
            m_sEkfCallbackWithGps(dtSeconds, zImu, zGps);

            std::lock_guard gpsMutex(m_sGpsMutex);
            m_sGpsSentToEkf = true;
        } else {
            m_sEkfCallbackImuOnly(dtSeconds, zImu);
        }

        m_sImuRotationVectorReady = false;               
        m_sImuLinearAccelerationReady = false;  
    } catch (const std::exception& e) {
        // TODO: Needs a way to log these errors, not to cerr in prod
        std::cerr << e.what() << std::endl;
    }
}

void IMUManager::TESTSensorCallback(const sh2_SensorValue& val) {
    try {
        if(ValidateImuEvent(val) == false) {
            m_sStats.imuRejected++;
        }

        StoreImuValue(val);
        m_sStats.imuAccepted++;
        
        if(!(m_sImuRotationVectorReady && m_sImuLinearAccelerationReady)) {
            return;
        }

        if(m_sEkfInstalled == false) {
            return;
        }

        sh2_RotationVectorWAcc rotationVectorSnapshot = m_sImuRotationVector.load();
        sh2_Accelerometer linearAccelerationSnapshot = m_sImuLinearAcceleration.load();

        bool gpsSentToEkfSnapshot;
        std::optional<GpsUpdate> gpsUpdateSnapshot;
        {
            std::lock_guard sentToEkfGuard(m_sGpsMutex);
            gpsUpdateSnapshot    = m_sLatestGps;
            gpsSentToEkfSnapshot = m_sGpsSentToEkf.load();
        }

        if(gpsUpdateSnapshot.has_value() == false)
        {
            return;
        }

        // This ymdNow is the obtain current year to calculate Magnetic declination
        auto ymdNow = std::chrono::system_clock::now();
        const std::chrono::year_month_day ymd{std::chrono::floor<std::chrono::days>(ymdNow)};
        Vector6d zImu = BuildImuMeasurementVector(rotationVectorSnapshot,
                                                  linearAccelerationSnapshot,
                                                  gpsUpdateSnapshot.value(),
                                                  static_cast<int>(ymd.year()));

        
        uint64_t accHwTime = m_sLastAccelerationVectorMachineTime.load();
        uint64_t rotHwTime = m_sLastRotationVectorMachineTime.load();
        uint64_t lastEKFHwTime = m_sLastEKFMachineTime.load();
        uint64_t oldestHwTime = std::min(accHwTime, rotHwTime);
        
        double dtSeconds = static_cast<double>(oldestHwTime - lastEKFHwTime) / 1e6;

        double nowMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count();

        m_sLastEKFMachineTime.store(oldestHwTime);

        if(gpsSentToEkfSnapshot == false) {
            Vector6d zGps = BuildGpsMeasurementVector(gpsUpdateSnapshot.value());
            fprintf(file, "%.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f\n", nowMs, zGps[0], zGps[1], zImu[2], zImu[3], zImu[4], zImu[5]);
            m_sEkfCallbackWithGps(dtSeconds, zImu, zGps);

            std::lock_guard gpsMutex(m_sGpsMutex);
            m_sGpsSentToEkf = true;
        } else {
            fprintf(file, "%.10f, %.10f, %.10f, %.10f, %.10f, %.10f, %.10f\n", nowMs, zImu[0], zImu[1], zImu[2], zImu[3], zImu[4], zImu[5]);
            m_sEkfCallbackImuOnly(dtSeconds, zImu);
        }

        m_sImuRotationVectorReady = false;               
        m_sImuLinearAccelerationReady = false;  
    } catch (const std::exception& e) {
        // TODO: Needs a way to log these errors, not to cerr in prod
        std::cerr << e.what() << std::endl;
    }
        
}

bool IMUManager::ValidateImuEvent(const sh2_SensorValue& sensorValue) {
    switch (sensorValue.sensorId) {

        case SH2_LINEAR_ACCELERATION:
            if( IsInvalidRange(sensorValue.un.linearAcceleration.x) ||
                IsInvalidRange(sensorValue.un.linearAcceleration.y) ||
                IsInvalidRange(sensorValue.un.linearAcceleration.z)) {
                return false;
            }
            break;

        case SH2_ROTATION_VECTOR:
            if( IsInvalidRange(sensorValue.un.rotationVector.i) ||
                IsInvalidRange(sensorValue.un.rotationVector.j) ||
                IsInvalidRange(sensorValue.un.rotationVector.k) ||
                IsInvalidRange(sensorValue.un.rotationVector.real) ||
                IsInvalidRange(sensorValue.un.rotationVector.accuracy)) {
                return false;
            }
            break;

        default:
            return false;
    }

    return true;
};

void IMUManager::StoreImuValue(const sh2_SensorValue& sensorValue) {
    switch (sensorValue.sensorId) {
        case SH2_LINEAR_ACCELERATION: {
            m_sImuLinearAccelerationReady = true;

            sh2_Accelerometer imuAcc = m_sImuLinearAcceleration;
            imuAcc.x = sensorValue.un.linearAcceleration.x;
            imuAcc.y = sensorValue.un.linearAcceleration.y;
            imuAcc.z = sensorValue.un.linearAcceleration.z;
            m_sImuLinearAcceleration = imuAcc;
            m_sLastAccelerationVectorMachineTime.store(sensorValue.timestamp);
            break;
        }
        case SH2_ROTATION_VECTOR:
            m_sImuRotationVectorReady = true;

            sh2_RotationVectorWAcc imuRot = m_sImuRotationVector;
            imuRot.i = sensorValue.un.rotationVector.i;
            imuRot.j = sensorValue.un.rotationVector.j;
            imuRot.k = sensorValue.un.rotationVector.k;
            imuRot.real = sensorValue.un.rotationVector.real;
            imuRot.accuracy = sensorValue.un.rotationVector.accuracy;
            m_sImuRotationVector = imuRot;
            m_sLastRotationVectorMachineTime.store(sensorValue.timestamp);
            break;
    }


}

Vector6d IMUManager::BuildGpsMeasurementVector(const GpsUpdate& gps) {
    Vector6d gpsVector = {gps.longitude, gps.latitude, 0, 0, 0, 0};
    return gpsVector;
}

Vector6d IMUManager::BuildImuMeasurementVector(const sh2_RotationVectorWAcc& rv, const sh2_Accelerometer& la, const GpsUpdate& gps, int currentYear) {
    double latitude = gps.latitude;
    double longitude = gps.longitude;
    const double RADAR_HEIGHT_M = 10;

    double magneticHeading = IMUUtils::Calculate_Magnetic_Heading(rv.real,
                                                                  rv.i,
                                                                  rv.j,
                                                                  rv.k);
    double magneticDeclination = m_sMagneticDeclination.CalculateDeclination(longitude,
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
        std::lock_guard kineticStateGuard(m_sKineticStateMutex);
        if(m_sKineticState.timestamp == steadyMin)
        {
            m_sKineticState.timestamp = std::chrono::steady_clock::now();
        }
        kineticState = IMUUtils::CalculateKineticUpdate(m_sKineticState,
                                                        globalGeoAccelerationX,
                                                        globalGeoAccelerationY);
        m_sKineticState = kineticState;
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
