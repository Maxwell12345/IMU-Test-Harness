/******************************************************************************
 * File:             IMUManager.cpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Synchronizes IMU payloads, computes payload-based filter
 *                   timing, and dispatches GPS/IMU measurements to the EKF.
 *
 ******************************************************************************/

#include <cmath>
#include <cstdio>

#include "IMUManager.hpp"
#include "MagneticDeclination.hpp"
#include "utils.hpp"

constexpr auto steadyMin = std::chrono::steady_clock::time_point::min();

IMUManager::IMUManager(std::shared_ptr<DatabaseManager> databaseManager, std::string cofPath) {
    if (databaseManager == nullptr) {
        throw std::invalid_argument("databaseManager is nullptr");
    }

    m_databaseManager = std::move(databaseManager);

    m_magneticDeclination.LoadCOF(cofPath);

    m_imuRotationVector = {0, 0, 0, 0, 0};
    m_imuLinearAcceleration = {0, 0, 0};

    this->m_muEast = 0.0;
    this->m_muNorth = 0.0;
    this->m_muYaw = 0.0;
    this->m_EMWA_alpha = 0.6;
}

void IMUManager::InstallEkf(std::function<void(double, Eigen::Matrix<double, 2, 1> &)> ekfCallbackImuOnly,
                            std::function<void(double, Eigen::Matrix<double, 2, 1> &, Eigen::Matrix<double, 2, 1> &)> ekfCallbackWithGps) {

    if (!ekfCallbackImuOnly) {
        throw std::invalid_argument("ekfCallbackImuOnly is nullptr");
    }

    if (!ekfCallbackWithGps) {
        throw std::invalid_argument("ekfCallbackWithGps is nullptr");
    }

    m_ekfCallbackImuOnly = std::move(ekfCallbackImuOnly);
    m_ekfCallbackWithGps = std::move(ekfCallbackWithGps);
    m_ekfInstalled = true;
}

std::optional<GpsUpdate> IMUManager::GetLatestGps() const {
    return m_latestGps;
}

void IMUManager::UpdateLatestGps(const GpsUpdate &update) {
    if (update.valid == false) {
        return;
    }
    const unsigned int STALE_TIME_OUT_SECONDS = 5;
    double deltaTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - update.receiveTime).count();
    if (deltaTime > STALE_TIME_OUT_SECONDS) {
        return;
    }

    if (m_latestGps.has_value() && update.timestamp <= m_latestGps->timestamp) {
        return;
    }

    m_latestGps = update;
    m_gpsSentToEkf = false;
}

void IMUManager::SensorCallback(std::optional<Raw_RotationVectorWAcc> optRv, std::optional<Raw_Accelerometer> optLa, std::optional<Raw_RotationRate> optRr) {
    // if (ValidateImuEvent(optRv, optLa) == false) {
    //     return;
    // }

    StoreImuValue(optRv, optLa, optRr);

    if (ReadyForEkf()) {
        DispatchToEkf();
    }
}

bool IMUManager::ReadyForEkf() const {
    return m_ekfInstalled &&
           m_imuRotationVectorReady &&
           m_imuLinearAccelerationReady &&
           m_imuRotationRateReady;
}

void IMUManager::DispatchToEkf() {
    Raw_RotationVectorWAcc rotationVectorSnapshot = m_imuRotationVector;
    Raw_Accelerometer linearAccelerationSnapshot = m_imuLinearAcceleration;
    Raw_RotationRate rotationRateSnapshot = m_imuRotationRate;

    bool gpsSentToEkfSnapshot;
    std::optional<GpsUpdate> gpsUpdateSnapshot;
    {
        std::lock_guard gpsGuard(m_gpsMutex);
        gpsUpdateSnapshot = m_latestGps;
        gpsSentToEkfSnapshot = m_gpsSentToEkf;
    }

    if (gpsUpdateSnapshot.has_value() == false) {
        throw std::runtime_error("No GPS data ever recorded");
    }

    double dtSeconds = PrepareEkfTiming();

    if (dtSeconds <= 0.0) {
        ResetImuReadyFlags();
        return;
    }

    if (!gpsUpdateSnapshot->measurementYear.has_value()) {
        throw std::runtime_error("GPS measurement year is unavailable for magnetic declination");
    }

    Eigen::Matrix<double, 2, 1> zImu = BuildImuMeasurementVector(
        rotationVectorSnapshot,
        linearAccelerationSnapshot,
        rotationRateSnapshot,
        *gpsUpdateSnapshot,
        *gpsUpdateSnapshot->measurementYear,
        dtSeconds);
    
    if (gpsSentToEkfSnapshot == false) {
        Eigen::Matrix<double, 2, 1> zGps = BuildGpsMeasurementVector(gpsUpdateSnapshot.value());

        this->m_databaseManager->EnqueueGpsUpdate(gpsUpdateSnapshot.value());
        m_ekfCallbackWithGps(dtSeconds, zImu, zGps);

        m_gpsSentToEkf = true;
    } else {
        m_ekfCallbackImuOnly(dtSeconds, zImu);
    }

    ResetImuReadyFlags();
}

double IMUManager::PrepareEkfTiming() {
    uint64_t accHwTime = m_imuLinearAcceleration.timestamp;
    uint64_t rotHwTime = m_imuRotationVector.timestamp;
    uint64_t lastEKFhwTime = m_lastEKFMachineTime;

    // TODO: This logic needs to be backtested and changed if required.
    uint64_t oldestHwTime = std::min(accHwTime, rotHwTime);

    if (lastEKFhwTime == 0) {
        m_lastEKFMachineTime = oldestHwTime;
        return 0.0;
    }

    if (oldestHwTime <= lastEKFhwTime) {
        return 0.0;
    }

    m_lastEKFMachineTime = oldestHwTime;
    return static_cast<double>(oldestHwTime - lastEKFhwTime) / 1e6;
}

void IMUManager::ResetImuReadyFlags() {
    m_imuRotationVectorReady = false;
    m_imuLinearAccelerationReady = false;
}

bool IMUManager::ValidateImuEvent(const std::optional<Raw_RotationVectorWAcc> &optRv,
                                  const std::optional<Raw_Accelerometer> &optLa,
                                  const std::optional<Raw_RotationRate> &optRr) {
    if (optLa.has_value()) {
        return !(IsInvalidRange(optLa.value().x) ||
                IsInvalidRange(optLa.value().y) ||
                IsInvalidRange(optLa.value().z) ||
                IsInvalidRange(optLa.value().timestamp));
    }

    if (optRv.has_value()) {
        return !(IsInvalidRange(optRv.value().i) ||
                IsInvalidRange(optRv.value().j) ||
                IsInvalidRange(optRv.value().k) ||
                IsInvalidRange(optRv.value().real) ||
                IsInvalidRange(optRv.value().accuracy) ||
                IsInvalidRange(optRv.value().timestamp));
    }

        return false;
};

void IMUManager::StoreImuValue(const std::optional<Raw_RotationVectorWAcc> &optRv,
                               const std::optional<Raw_Accelerometer> &optLa,
                               const std::optional<Raw_RotationRate> &optRr) {
    if (optLa.has_value()) {
        m_imuLinearAccelerationReady = true;
        m_imuLinearAcceleration = optLa.value();
        m_databaseManager->EnqueueIMULinearAcceleration(m_imuLinearAcceleration);
    }

    if (optRv.has_value()) {
        m_imuRotationVectorReady = true;
        m_imuRotationVector = optRv.value();
        m_databaseManager->EnqueueIMURotationVector(m_imuRotationVector);
    }

    if (optRr.has_value()) {
        m_imuRotationRateReady = true;
        m_imuRotationRate = optRr.value();
    }
}

Eigen::Matrix<double, 2, 1> IMUManager::BuildGpsMeasurementVector(const GpsUpdate &gps) {
    Eigen::Matrix<double, 2, 1> gpsVector = {gps.longitude, gps.latitude};
    return gpsVector;
}

Eigen::Matrix<double, 2, 1> IMUManager::BuildImuMeasurementVector(const Raw_RotationVectorWAcc &rv,
                                               const Raw_Accelerometer &la,
                                               const Raw_RotationRate &rr,
                                               const GpsUpdate &gps, 
                                               int currentYear,
                                               double dt) {
    double latitude = gps.latitude;
    double longitude = gps.longitude;
    const double RADAR_HEIGHT_M = 4.0;

    double magneticDeclinationDeg = m_magneticDeclination.CalculateDeclination(longitude,
                                                                               latitude,
                                                                               RADAR_HEIGHT_M,
                                                                               currentYear);

    const IMUUtils::ENUAccel acc = IMUUtils::RotateLinearAccelToTrueENU(
        rv.real, rv.i, rv.j, rv.k, la.x, la.y, la.z, magneticDeclinationDeg
    );

    this->m_muEast = this->m_EMWA_alpha * acc.east + (1.0 - this->m_EMWA_alpha) * this->m_muEast;
    this->m_muNorth = this->m_EMWA_alpha * acc.north + (1.0 - this->m_EMWA_alpha) * this->m_muNorth;
    this->m_muYaw = this->m_EMWA_alpha * rr.d_yaw + (1.0 - this->m_EMWA_alpha) * this->m_muYaw;

    const double heading = IMUUtils::ComputeENUHeading(rv.i, rv.j, rv.k, rv.real, rr.d_roll, rr.d_pitch, this->m_muYaw, dt, magneticDeclinationDeg * 3.14159265359 / 180.0);
    const double accelerationForward = this->m_muEast * std::cos(heading) + this->m_muNorth * std::sin(heading);

    Eigen::Matrix<double, 2, 1> imuVector = {
        this->m_muYaw,
        accelerationForward
    };

    return imuVector;
}
