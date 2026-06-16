#include <cmath>
#include <cstdio>

#include "IMUManager.hpp"
#include "MagneticDeclination.hpp"
#include "utils.hpp"

constexpr auto steadyMin = std::chrono::steady_clock::time_point::min();
//-----------------------------------------------------------------------------------------------------------------------------------------
IMUManager::IMUManager(boost::shared_ptr<DatabaseManager> databaseManager, std::string cofPath) {
  if (databaseManager == nullptr) {
    throw std::invalid_argument("databaseManager is nullptr");
  }

  m_databaseManager = std::move(databaseManager);

  m_magneticDeclination.LoadCOF(cofPath);

  m_kineticState = IMUUtils::KineticState(steadyMin, 0.0, 0.0, 0.0, 0.0); 
  m_imuRotationVector = {0, 0, 0, 0, 0};
  m_imuLinearAcceleration = {0, 0, 0};
}

//-----------------------------------------------------------------------------------------------------------------------------------------
void IMUManager::InstallEkf(
    std::function<void(double, Vector6d &)> ekfCallbackImuOnly, std::function<void(double, Vector6d &, Vector6d &)> ekfCallbackWithGps
) {

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

IMUManagerStats IMUManager::GetStats() const { return m_stats; }

std::optional<GpsUpdate> IMUManager::GetLatestGps() const {
  return m_latestGps;
}
//-----------------------------------------------------------------------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------------------------------------------------------------------
void IMUManager::SensorCallback(void *cookie, sh2_SensorEvent *event) {
  try{
    const auto self = static_cast<IMUManager*>(cookie);
    if(self != nullptr) {
      self->OnSensorEvent(event);
    } 
    throw std::invalid_argument("Cookie (IMUManager Instance) was a nullptr");
  } catch (const std::exception &e) {
    std::cerr << "[ERROR] IMUManager::SensorCallback" << e.what() << std::endl;
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------
void IMUManager::OnSensorEvent(sh2_SensorEvent *event) {
  try {
    if (event == nullptr) {
      m_stats.imuRejected++;
      throw std::invalid_argument("Sh2_SensorEvent was a nullptr");
    }

    sh2_SensorValue_t value{};
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
      m_stats.imuRejected++;
      throw std::runtime_error("There was an error trying to decode Sh2_SensorEvent");
    };
    IngestSensorValue(value);
  } catch (const std::exception &e) {
    // TODO: Needs a way to log these errors, not to cerr in prod
    std::cerr << "[ERROR] IMUManager::OnSensorEvent " << e.what() << std::endl;
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------
void IMUManager::IngestSensorValue(const sh2_SensorValue &value) {
  if (ValidateImuEvent(value) == false) {
    m_stats.imuRejected++;
    throw std::runtime_error("Sensor value out of range or Report type not supported");
  }

  StoreImuValue(value);
  m_stats.imuAccepted++;

  if (ReadyForEkf()) {
    DispatchToEkf();
  }
}

bool IMUManager::ReadyForEkf() const { 
  return m_ekfInstalled && m_imuRotationVectorReady && m_imuLinearAccelerationReady;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int IMUManager::GetCurrentYear() const {
  auto ymdNow = std::chrono::system_clock::now();
  const std::chrono::year_month_day ymd{std::chrono::floor<std::chrono::days>(ymdNow)};
  return static_cast<int>(ymd.year());
}
//-----------------------------------------------------------------------------------------------------------------------------------------
double IMUManager::PrepareEkfTiming() {
  uint64_t accHwTime = m_lastAccelerationVectorMachineTime;
  uint64_t rotHwTime = m_lastRotationVectorMachineTime;
  uint64_t lastEKFhwTime = m_lastEKFMachineTime;
  uint64_t oldestHwTime = std::min(accHwTime, rotHwTime);

  double dtSeconds = static_cast<double>(oldestHwTime - lastEKFhwTime) / 1e6;
  m_lastEKFMachineTime = oldestHwTime;
  return dtSeconds;
}
//-----------------------------------------------------------------------------------------------------------------------------------------
void IMUManager::ResetImuReadyFlags() {
  m_imuRotationVectorReady = false;
  m_imuLinearAccelerationReady = false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------
void IMUManager::DispatchToEkf() {
  try {
    sh2_RotationVectorWAcc rotationVectorSnapshot = m_imuRotationVector;
    sh2_Accelerometer linearAccelerationSnapshot = m_imuLinearAcceleration;

    bool gpsSentToEkfSnapshot;
    std::optional<GpsUpdate> gpsUpdateSnapshot;
    {
      gpsUpdateSnapshot = m_latestGps;
      gpsSentToEkfSnapshot = m_gpsSentToEkf;
    }

    if (gpsUpdateSnapshot.has_value() == false) {
      throw std::runtime_error("No GPS data ever recorded");
    }

    int year = GetCurrentYear();
    Vector6d zImu = BuildImuMeasurementVector(rotationVectorSnapshot, linearAccelerationSnapshot, gpsUpdateSnapshot.value(), year);

    double dtSeconds = PrepareEkfTiming();

    if (gpsSentToEkfSnapshot == false) {
      Vector6d zGps = BuildGpsMeasurementVector(gpsUpdateSnapshot.value());
      m_ekfCallbackWithGps(dtSeconds, zImu, zGps);

      m_gpsSentToEkf = true;
    } else {
      m_ekfCallbackImuOnly(dtSeconds, zImu);
    }

    ResetImuReadyFlags();
  } catch (const std::exception &e) {
    std::cerr << "[ERROR] IMUManager::DispatchToEkf:" << e.what() << std::endl;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IMUManager::ValidateImuEvent(const sh2_SensorValue &sensorValue) {
  switch (sensorValue.sensorId) {

  case SH2_LINEAR_ACCELERATION:
    if (IsInvalidRange(sensorValue.un.linearAcceleration.x) || IsInvalidRange(sensorValue.un.linearAcceleration.y) ||
        IsInvalidRange(sensorValue.un.linearAcceleration.z)) {
      return false;
    }
    break;

  case SH2_ROTATION_VECTOR:
    if (IsInvalidRange(sensorValue.un.rotationVector.i) || IsInvalidRange(sensorValue.un.rotationVector.j) ||
        IsInvalidRange(sensorValue.un.rotationVector.k) || IsInvalidRange(sensorValue.un.rotationVector.real) ||
        IsInvalidRange(sensorValue.un.rotationVector.accuracy)) {
      return false;
    }
    break;

  default:
    return false;
  }

  return true;
};
//-----------------------------------------------------------------------------------------------------------------------------------------
void IMUManager::StoreImuValue(const sh2_SensorValue& sensorValue) {
  switch (sensorValue.sensorId) {
  case SH2_LINEAR_ACCELERATION: {
    m_imuLinearAccelerationReady = true;

    sh2_Accelerometer imuAcc = m_imuLinearAcceleration;
    imuAcc.x = sensorValue.un.linearAcceleration.x;
    imuAcc.y = sensorValue.un.linearAcceleration.y;
    imuAcc.z = sensorValue.un.linearAcceleration.z;
    m_imuLinearAcceleration = imuAcc;
    this->m_lastAccelerationVectorMachineTime = sensorValue.timestamp;
    m_databaseManager->EnqueueIMULinearAcceleration(sensorValue);
    break;
  }
  case SH2_ROTATION_VECTOR:
    m_imuRotationVectorReady = true;

    sh2_RotationVectorWAcc imuRot = m_imuRotationVector;
    imuRot.i = sensorValue.un.rotationVector.i;
    imuRot.j = sensorValue.un.rotationVector.j;
    imuRot.k = sensorValue.un.rotationVector.k;
    imuRot.real = sensorValue.un.rotationVector.real;
    imuRot.accuracy = sensorValue.un.rotationVector.accuracy;
    m_imuRotationVector = imuRot;
    m_lastRotationVectorMachineTime = sensorValue.timestamp;
    m_databaseManager->EnqueueIMURotationVector(sensorValue);
    break;
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------
Vector6d IMUManager::BuildGpsMeasurementVector(const GpsUpdate &gps) {
  Vector6d gpsVector = {gps.longitude, gps.latitude, 0, 0, 0, 0};
  return gpsVector;
}
//-----------------------------------------------------------------------------------------------------------------------------------------
Vector6d IMUManager::BuildImuMeasurementVector(
    const sh2_RotationVectorWAcc &rv, const sh2_Accelerometer &la, const GpsUpdate &gps, int currentYear
) {
  double latitude = gps.latitude;
  double longitude = gps.longitude;
  const double RADAR_HEIGHT_M = 10;

  double magneticHeading = IMUUtils::Calculate_Magnetic_Heading(rv.real, rv.i, rv.j, rv.k);
  double magneticDeclination = m_magneticDeclination.CalculateDeclination(longitude, latitude, RADAR_HEIGHT_M, currentYear);
  double trueHeading = IMUUtils::MagneticToTrueHeading(magneticHeading, magneticDeclination);

  double trueHeadingRadians = IMUUtils::DegreesToRadians(trueHeading);

  double globalLinearAccelerationX = IMUUtils::InertialToGlobal_X(trueHeadingRadians, la.x, la.y);

  double globalLinearAccelerationY = IMUUtils::InertialToGlobal_Y(trueHeadingRadians, la.x, la.y);

  double globalGeoAccelerationX = IMUUtils::Convert_Global_X_to_DegPerS2(latitude, globalLinearAccelerationX);
  double globalGeoAccelerationY = IMUUtils::Convert_Global_Y_to_DegPerS2(globalLinearAccelerationY);

  IMUUtils::KineticState kineticState;
    if (m_kineticState.timestamp == steadyMin) {
      m_kineticState.timestamp = std::chrono::steady_clock::now();
    }
    kineticState = IMUUtils::CalculateKineticUpdate(m_kineticState, globalGeoAccelerationX, globalGeoAccelerationY);
    m_kineticState = kineticState;
  

  Vector6d imuVector = {0.0,
                        0.0,
                        kineticState.speedEastWest,
                        kineticState.speedNorthSouth,
                        kineticState.accelerationEastWest,
                        kineticState.accelerationNorthSouth};

  return imuVector;
}
