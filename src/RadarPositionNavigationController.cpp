#include "RadarPositionNavigationController.hpp"
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <thread>

RadarPositionNavigationController::RadarPositionNavigationController(const _KalmanValues& config,
                                                                     std::shared_ptr<DatabaseManager> dbManager,
                                                                     std::unique_ptr<IMUSerialPortReader> imuSerialPortReader,
                                                                     std::unique_ptr<GpsManagerBase> gpsManager,
                                                                     std::unique_ptr<IMUManager> imuManager):
                                                                     m_config(config),
                                                                     m_running(false),
                                                                     m_isKFConfigured(false),
                                                                     m_latestX(Vector6d::Zero()),
                                                                     m_latestP(Matrix6d::Zero()),
                                                                     m_dbManager(dbManager),
                                                                     m_imuManager(std::move(imuManager)),
                                                                     m_gpsManager(std::move(gpsManager)),
                                                                     m_imuSerialPortReader(std::move(imuSerialPortReader)) {
  auto imuSerialCallback = [&imuManager = this->m_imuManager](std::optional<Raw_RotationVectorWAcc> optRv,
                                                              std::optional<Raw_Accelerometer> optLa){
    imuManager->SensorCallback(optRv, optLa);
  };
  this->m_imuSerialPortReader->InstallCallback(imuSerialCallback);

  auto gpsManagerCallback = [&imuManager = this->m_imuManager](const GpsUpdate& g) {
    imuManager->UpdateLatestGps(g);
  };
  this->m_gpsManager->InstallCallback(gpsManagerCallback);
}

RadarPositionNavigationController::~RadarPositionNavigationController() {
  this->TotalDestruction();
}

std::function<void(const GpsUpdate &)> RadarPositionNavigationController::GetGPSCallback() {
  return [this](const GpsUpdate &gpsUpdate) { this->_GPSCallback(gpsUpdate); };
}

void RadarPositionNavigationController::StartAndConfigureRadarPNT(double lat0, double lon0) {
  this->StopRadarPNT();

  if (!this->m_isKFConfigured) {
    this->ConfigureKalmanFilter(lat0,
                                lon0,
                                m_config.gpsChiSqLowerBound,
                                m_config.gpsChiSqUpperBound,
                                m_config.imuChiSqLowerBound,
                                m_config.imuChiSqUpperBound);

    this->m_isKFConfigured = true;
  }

  m_imuManager->InstallEkf([this](double dt, Vector6d &imuVec) { this->KFCallbackImuOnly(dt, imuVec); },
                           [this](double dt, Vector6d &imuVec, Vector6d &gpsVec) { this->KFCallbackWithGps(dt, imuVec, gpsVec); });

  this->StartIMUReader();
  this->m_running = true;
}

void RadarPositionNavigationController::StartIMUReader() {
    this->m_imuSerialPortReader->Start();
}

void RadarPositionNavigationController::StopRadarPNT() {
    this->m_imuSerialPortReader->Stop();
    this->m_isKFConfigured = false;
    this->m_running = false;
}

bool RadarPositionNavigationController::IsRunning() const {
  return m_running;
}

void RadarPositionNavigationController::ConfigureKalmanFilter(double lat0, double lon0, double gpsLowerPercentile,
                                                              double gpsUpperPercentile, double imuLowerPercentile,
                                                              double imuUpperPercentile) {
  Vector6d x0;
  x0 << lon0, lat0, 1e-15, 1e-15, 1e-16, 1e-16;

  Matrix6d P0 = Matrix6d::Zero();
  P0(0, 0) = 1e-10;
  P0(1, 1) = 1e-10;
  P0(2, 2) = 1e-8;
  P0(3, 3) = 1e-8;
  P0(4, 4) = 1e-10;
  P0(5, 5) = 1e-10;

  Matrix6d R0_GPS = Matrix6d::Zero();
  R0_GPS(0, 0) = 1e-10;
  R0_GPS(1, 1) = 1e-10;

  Matrix6d R0_IMU = Matrix6d::Zero();
  R0_IMU(2, 2) = 1e-8;
  R0_IMU(3, 3) = 1e-8;
  R0_IMU(4, 4) = 1e-10;
  R0_IMU(5, 5) = 1e-10;

  Matrix6d Q0 = Matrix6d::Zero();
  Q0(0, 0) = 1e-14;
  Q0(1, 1) = 1e-14;
  Q0(2, 2) = 1e-12;
  Q0(3, 3) = 1e-12;
  Q0(4, 4) = 1e-14;
  Q0(5, 5) = 1e-14;

  this->m_latestX = x0;
  this->m_latestP = P0;

  auto checkPercentileBounds = [](double percentile) { return percentile <= 0.0 || percentile >= 1.0; };

  // Calculate Chi SQ stats for df 2 and 4 at percentiles
  if (checkPercentileBounds(gpsLowerPercentile) ||
      checkPercentileBounds(gpsUpperPercentile) ||
      checkPercentileBounds(imuLowerPercentile) ||
      checkPercentileBounds(imuUpperPercentile)) {
    throw std::runtime_error("One or more Fuzzy fusion Chi SQ percentiles are <= 0 and/or >= 1");
  }

  if (gpsLowerPercentile >= gpsUpperPercentile) {
    throw std::runtime_error("GPS Chi SQ lower percentile is >= upper percentile");
  }

  if (imuLowerPercentile >= imuUpperPercentile) {
    throw std::runtime_error("IMU Chi SQ lower percentile is >= upper percentile");
  }

  double chiSquaredBetaLowerBound_GPS = boost::math::quantile(boost::math::chi_squared(2), gpsLowerPercentile);
  double chiSquaredBetaUpperBound_GPS = boost::math::quantile(boost::math::chi_squared(2), gpsUpperPercentile);

  double chiSquaredBetaLowerBound_IMU = boost::math::quantile(boost::math::chi_squared(4), imuLowerPercentile);
  double chiSquaredBetaUpperBound_IMU = boost::math::quantile(boost::math::chi_squared(4), imuUpperPercentile);

  this->m_kf = IMUGPSFusionKF_2D_ConstantAcceleration(x0,
                                                      P0,
                                                      R0_GPS,
                                                      R0_IMU,
                                                      Q0,
                                                      chiSquaredBetaLowerBound_GPS,
                                                      chiSquaredBetaLowerBound_IMU,
                                                      chiSquaredBetaUpperBound_GPS,
                                                      chiSquaredBetaUpperBound_IMU,
                                                      m_config.gpsN,
                                                      m_config.gpsL,
                                                      m_config.imuN,
                                                      m_config.imuL,
                                                      m_config.qN,
                                                      m_config.qL);
}

void RadarPositionNavigationController::KFCallbackImuOnly(double dt, Vector6d &imuVec) {
  bool needsReconfig = false;
  double reconfigLat = 0.0;
  double reconfigLon = 0.0;
  std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);
  {
    if (!this->m_isKFConfigured.load()) {
      return;
    }

    try {
      std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, imuVec);

      this->m_latestX = output.first;
      this->m_latestP = output.second;
    } catch (const std::exception &e) {
      // TODO: Log this
      std::cout << "[ERROR] " << e.what() << std::endl;
      if (this->m_isKFConfigured) {
        double lat = this->m_latestX(1, 0);
        double lon = this->m_latestX(0, 0);

        if (!std::isfinite(lat) || !std::isfinite(lon)) {
          this->m_isKFConfigured.store(false);
          return;
        }
        // note: original was lat0, lon0
        needsReconfig = true;
        reconfigLat = lat;
        reconfigLon = lon;
      }
    }
  }

  if (needsReconfig) {
    this->ConfigureKalmanFilter(reconfigLat,
                                reconfigLon,
                                m_config.gpsChiSqLowerBound,
                                m_config.gpsChiSqUpperBound,
                                m_config.imuChiSqLowerBound,
                                m_config.imuChiSqUpperBound);
  }
}

void RadarPositionNavigationController::KFCallbackWithGps(double dt, Vector6d &imuVec, Vector6d &gpsVec) {
  bool needsReconfig = false;
  double reconfigLat = 0.0;
  double reconfigLon = 0.0;
  {
    std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);

    if (!this->m_isKFConfigured.load()) {
      return;
    }

    try {
      std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, gpsVec, imuVec);

      this->m_latestX = output.first;
      this->m_latestP = output.second;
    } catch (const std::exception &e) {
      // TODO: Log this
      std::cout << "[ERROR] " << e.what() << std::endl;
      if (this->m_isKFConfigured) {
        double lat = this->m_latestX(1, 0);
        double lon = this->m_latestX(0, 0);

        if (!std::isfinite(lat) || !std::isfinite(lon)) {
          this->m_isKFConfigured.store(false);
          return;
        }
        // note: original was lat0, lon0
        needsReconfig = true;
        reconfigLat = lat;
        reconfigLon = lon;
      }
    }
  }
  if (needsReconfig) {
    this->ConfigureKalmanFilter(reconfigLat,
                                reconfigLon,
                                m_config.gpsChiSqLowerBound,
                                m_config.gpsChiSqUpperBound,
                                m_config.imuChiSqLowerBound,
                                m_config.imuChiSqUpperBound);
  }
}

void RadarPositionNavigationController::_GPSCallback(const GpsUpdate &gpsUpdate) {
  m_imuManager->UpdateLatestGps(gpsUpdate);
}

void RadarPositionNavigationController::TotalDestruction() {
  this->StopRadarPNT();

  std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);

  this->m_kf.Clean();
  this->m_isKFConfigured = false;

  this->m_latestX = Vector6d::Zero();
  this->m_latestP = Matrix6d::Zero();
}
