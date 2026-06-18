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

  m_kineticState = IMUUtils::KineticState(steadyMin, 0.0, 0.0, 0.0, 0.0); 
  m_imuRotationVector = {0, 0, 0, 0, 0};
  m_imuLinearAcceleration = {0, 0, 0};
}

void IMUManager::InstallEkf(std::function<void(double, Vector6d &)> ekfCallbackImuOnly,
                            std::function<void(double, Vector6d &, Vector6d &)> ekfCallbackWithGps) {

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

IMUManagerStats IMUManager::GetStats() const {
  return m_stats;
}

std::optional<GpsUpdate> IMUManager::GetLatestGps() const {
  return m_latestGps;
}

void IMUManager::UpdateLatestGps(const GpsUpdate &update) {
  if (update.valid == false) {
    m_stats.gpsRejected++;
    return;
  }

  const unsigned int STALE_TIME_OUT_SECONDS = 5;
  double deltaTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - update.receiveTime).count();
  if (deltaTime > STALE_TIME_OUT_SECONDS) {
    m_stats.gpsRejected++;
    return;
  }

  if (m_latestGps.has_value() && update.gpsTimestampMs <= m_latestGps->gpsTimestampMs) {
    m_stats.gpsRejected++;
    return;
  }

  m_latestGps = update;
  m_gpsSentToEkf = false;
  m_stats.gpsAccepted++;
}

void IMUManager::SensorCallback(std::optional<LinearAcceleration> optLa, std::optional<RotationVectorWAcc> optRv) {
  if (ValidateImuEvent(optLa, optRv) == false) {
    m_stats.imuRejected++;
    throw std::runtime_error("Sensor value out of range or Report type not supported");
  }

  StoreImuValue(optLa, optRv);
  m_stats.imuAccepted++;

  if (ReadyForEkf()) {
    DispatchToEkf();
  }
}

bool IMUManager::ReadyForEkf() const { 
  return m_ekfInstalled && m_imuRotationVectorReady && m_imuLinearAccelerationReady;
}

void IMUManager::DispatchToEkf() {
  RotationVectorWAcc rotationVectorSnapshot = m_imuRotationVector;
  LinearAcceleration linearAccelerationSnapshot = m_imuLinearAcceleration;

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

  int year = GetCurrentYear();
  Vector6d zImu = BuildImuMeasurementVector(rotationVectorSnapshot,
                                            linearAccelerationSnapshot,
                                            gpsUpdateSnapshot.value(),
                                            year);

  double dtSeconds = PrepareEkfTiming();

  if (gpsSentToEkfSnapshot == false) {
    Vector6d zGps = BuildGpsMeasurementVector(gpsUpdateSnapshot.value());
    m_ekfCallbackWithGps(dtSeconds, zImu, zGps);

    m_gpsSentToEkf = true;
  } else {
    m_ekfCallbackImuOnly(dtSeconds, zImu);
  }

  ResetImuReadyFlags();
}

int IMUManager::GetCurrentYear() const {
  auto ymdNow = std::chrono::system_clock::now();
  const std::chrono::year_month_day ymd{std::chrono::floor<std::chrono::days>(ymdNow)};
  return static_cast<int>(ymd.year());
}

double IMUManager::PrepareEkfTiming() {
  uint64_t accHwTime = m_imuLinearAcceleration.timestamp;
  uint64_t rotHwTime = m_imuRotationVector.timestamp;
  uint64_t lastEKFhwTime = m_lastEKFMachineTime;

  // TODO: This logic needs to be backtested and changed if required.
  uint64_t oldestHwTime = std::min(accHwTime, rotHwTime);

  double dtSeconds = static_cast<double>(oldestHwTime - lastEKFhwTime) / 1e6;
  m_lastEKFMachineTime = oldestHwTime;
  return dtSeconds;
}

void IMUManager::ResetImuReadyFlags() {
  m_imuRotationVectorReady = false;
  m_imuLinearAccelerationReady = false;
}

bool IMUManager::ValidateImuEvent(std::optional<LinearAcceleration> optLa, std::optional<RotationVectorWAcc> optRv) {
  if(optLa.has_value()) {
    return !(IsInvalidRange(optLa.value().x) ||
             IsInvalidRange(optLa.value().y) ||
             IsInvalidRange(optLa.value().z) ||
             IsInvalidRange(optLa.value().timestamp));
  }

  if(optRv.has_value()) {
    return !(IsInvalidRange(optRv.value().i) ||
             IsInvalidRange(optRv.value().j) ||
             IsInvalidRange(optRv.value().k) ||
             IsInvalidRange(optRv.value().real) ||
             IsInvalidRange(optRv.value().accuracy) ||
             IsInvalidRange(optRv.value().timestamp));
  }

  return false;
};

void IMUManager::StoreImuValue(std::optional<LinearAcceleration> optLa, std::optional<RotationVectorWAcc> optRv) {
  if(optLa.has_value()) {
    m_imuLinearAccelerationReady = true;
    m_imuLinearAcceleration = optLa.value();
    m_databaseManager->EnqueueIMULinearAcceleration(m_imuLinearAcceleration);
  }

  if(optRv.has_value()) {
    m_imuRotationVectorReady = true;
    m_imuRotationVector = optRv.value();
    m_databaseManager->EnqueueIMURotationVector(m_imuRotationVector);
  }
}

Vector6d IMUManager::BuildGpsMeasurementVector(const GpsUpdate &gps) {
  Vector6d gpsVector = {gps.longitude, gps.latitude, 0, 0, 0, 0};
  return gpsVector;
}

Vector6d IMUManager::BuildImuMeasurementVector(const RotationVectorWAcc &rv,
                                               const LinearAcceleration &la,
                                               const GpsUpdate &gps,
                                               int currentYear) {
  double latitude = gps.latitude;
  double longitude = gps.longitude;
  const double RADAR_HEIGHT_M = 10;

  double magneticHeading = IMUUtils::Calculate_Magnetic_Heading(rv.real,
                                                                rv.i,
                                                                rv.j,
                                                                rv.k);

  double magneticDeclination = m_magneticDeclination.CalculateDeclination(longitude,
                                                                          latitude,
                                                                          RADAR_HEIGHT_M,
                                                                          currentYear);

  double trueHeading = IMUUtils::MagneticToTrueHeading(magneticHeading, magneticDeclination);

  double trueHeadingRadians = IMUUtils::DegreesToRadians(trueHeading);

  double globalLinearAccelerationX = IMUUtils::InertialToGlobal_X(trueHeadingRadians,
                                                                  la.x,
                                                                  la.y);

  double globalLinearAccelerationY = IMUUtils::InertialToGlobal_Y(trueHeadingRadians,
                                                                  la.x,
                                                                  la.y);

  double globalGeoAccelerationX = IMUUtils::Convert_Global_X_to_DegPerS2(latitude, globalLinearAccelerationX);
  double globalGeoAccelerationY = IMUUtils::Convert_Global_Y_to_DegPerS2(globalLinearAccelerationY);

  IMUUtils::KineticState kineticState;
    if (m_kineticState.timestamp == steadyMin) {
      m_kineticState.timestamp = std::chrono::steady_clock::now();
    }
    kineticState = IMUUtils::CalculateKineticUpdate(m_kineticState,
                                                    globalGeoAccelerationX,
                                                    globalGeoAccelerationY);
    m_kineticState = kineticState;
  

  Vector6d imuVector = {0.0,
                        0.0,
                        kineticState.speedEastWest,
                        kineticState.speedNorthSouth,
                        kineticState.accelerationEastWest,
                        kineticState.accelerationNorthSouth};

  return imuVector;
}
