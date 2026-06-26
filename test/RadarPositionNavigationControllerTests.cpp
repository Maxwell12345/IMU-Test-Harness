#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <limits>
#include <thread>

#include "IMUManager.hpp"
#include "RadarPositionNavigationController.hpp"
#include "gps/GpsManagerBase.hpp"

namespace {
#ifdef _WIN32
    std::string path = R"(COM1)";
#else
    std::string path = "/dev/ttyUSB0";
#endif

  _ImuSerialPort imuSerialPortConfig = {
    path,
    460800
  };

  _KalmanValues kalmanValuesConfig = {
    20,
    5,
    0.20,
    0.95,
    100,
    10,
    0.20,
    0.95,
    100,
    10
  };
}

#define SET_UP()  std::shared_ptr<DatabaseManager> databaseManager = std::make_shared<DatabaseManager>("./IMUPROC_tests.db"); \
                  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(imuSerialPortConfig, std::make_unique<MockSerialPort>()); \
                  auto imuManager = std::make_unique<IMUManager>(databaseManager); \
                  auto gpsManager = std::make_unique<GpsManagerBase>(); \
                  RadarPositionNavigationController radarPositionNavigationController(kalmanValuesConfig, \
                                                                                      databaseManager, \
                                                                                      std::move(imuSerialPortReader), \
                                                                                      std::move(gpsManager), \
                                                                                      std::move(imuManager));

class MockSerialPort : public SerialPortBase {};

TEST(RadarPositionNavigationControllerTest, GetGPSCallbackUpdatesLatestGps) {
  SET_UP();

  GpsUpdate gps{};
  gps.receiveTime = std::chrono::steady_clock::now();
  gps.timestamp = 12345;
  gps.latitude = 47.319065;
  gps.longitude = 5.06832;
  gps.heading = 91.0;
  gps.fixQuality = 1;
  gps.numSatellites = 8;
  gps.hdop = 0.9;
  gps.gpsTimestampMs = std::numeric_limits<uint32_t>::max();
  gps.valid = true;

  auto callback = radarPositionNavigationController.GetGPSCallback();
  callback(gps);

  auto latest = radarPositionNavigationController.m_imuManager->GetLatestGps();

  ASSERT_TRUE(latest.has_value());
  ASSERT_NEAR(latest->latitude, 47.319065, 1e-12);
  ASSERT_NEAR(latest->longitude, 5.06832, 1e-12);
  ASSERT_TRUE(latest->heading.has_value());
  ASSERT_NEAR(latest->heading.value(), 91.0, 1e-12);
  ASSERT_EQ(latest->fixQuality, 1);
  ASSERT_EQ(latest->numSatellites, 8);
  ASSERT_NEAR(latest->hdop, 0.9, 1e-12);
  ASSERT_EQ(latest->gpsTimestampMs, std::numeric_limits<uint32_t>::max());
  ASSERT_TRUE(latest->valid);
}

TEST(RadarPositionNavigationControllerTest, StartAndConfigureRadarPNTConfiguresKFAndStartsReader) {
  SET_UP();

  radarPositionNavigationController.StartAndConfigureRadarPNT(47.319065, 5.06832);

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  ASSERT_TRUE(radarPositionNavigationController.IsRunning());

  ASSERT_NEAR(radarPositionNavigationController.m_latestX(0), 5.06832, 1e-12);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(1), 47.319065, 1e-12);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(2), 1e-15, 1e-20);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(3), 1e-15, 1e-20);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(4), 1e-16, 1e-21);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(5), 1e-16, 1e-21);

  radarPositionNavigationController.StopRadarPNT();

  ASSERT_FALSE(radarPositionNavigationController.IsRunning());
}

TEST(RadarPositionNavigationControllerTest, TotalDestructionStopsReaderCleansKFAndZerosLatestState) {
  SET_UP();

  radarPositionNavigationController.StartAndConfigureRadarPNT(47.319065, 5.06832);

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  ASSERT_TRUE(radarPositionNavigationController.m_isKFConfigured.load());
  ASSERT_FALSE(radarPositionNavigationController.m_latestX.isZero(0.0));

  radarPositionNavigationController.TotalDestruction();

  ASSERT_FALSE(radarPositionNavigationController.m_isKFConfigured.load());
  ASSERT_TRUE(radarPositionNavigationController.m_latestX.isZero(0.0));
  ASSERT_TRUE(radarPositionNavigationController.m_latestP.isZero(0.0));
}

TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterSetsInitialStateAndCovariance) {
  SET_UP();

  radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.20, 0.95);

  ASSERT_NEAR(radarPositionNavigationController.m_latestX(0), 5.06832, 1e-12);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(1), 47.319065, 1e-12);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(2), 1e-15, 1e-20);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(3), 1e-15, 1e-20);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(4), 1e-16, 1e-21);
  ASSERT_NEAR(radarPositionNavigationController.m_latestX(5), 1e-16, 1e-21);

  ASSERT_NEAR(radarPositionNavigationController.m_latestP(0, 0), 1e-10, 1e-20);
  ASSERT_NEAR(radarPositionNavigationController.m_latestP(1, 1), 1e-10, 1e-20);
  ASSERT_NEAR(radarPositionNavigationController.m_latestP(2, 2), 1e-8, 1e-18);
  ASSERT_NEAR(radarPositionNavigationController.m_latestP(3, 3), 1e-8, 1e-18);
  ASSERT_NEAR(radarPositionNavigationController.m_latestP(4, 4), 1e-10, 1e-20);
  ASSERT_NEAR(radarPositionNavigationController.m_latestP(5, 5), 1e-10, 1e-20);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i != j) {
        ASSERT_NEAR(radarPositionNavigationController.m_latestP(i, j), 0.0, 1e-30) << "Index i: " + std::to_string(i) + " and j: " +
        std::to_string(j);
      }
    }
  }
}

TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsInvalidPercentiles) {
  SET_UP();

  ASSERT_THROW(radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 0.0, 0.95, 0.20, 0.95), std::runtime_error);
  ASSERT_THROW(radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 1.0, 0.95, 0.20, 0.95), std::runtime_error);
  ASSERT_THROW(radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.0, 0.95), std::runtime_error);
  ASSERT_THROW(radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 1.0, 0.95), std::runtime_error);
}

TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsLowerPercentileGreaterThanUpperPercentile) {
  SET_UP();

  ASSERT_THROW(radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 0.95, 0.20, 0.20, 0.95), std::runtime_error);
  ASSERT_THROW(radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.95, 0.20), std::runtime_error);
}

TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyReturnsWithoutConfiguredKF) {
  SET_UP();

  Vector6d imuVec;
  imuVec << 0.0, 0.0, 1e-7, -1e-7, 1e-8, -1e-8;

  radarPositionNavigationController.KFCallbackImuOnly(0.01, imuVec);

  ASSERT_TRUE(radarPositionNavigationController.m_latestX.isZero(0.0));
  ASSERT_TRUE(radarPositionNavigationController.m_latestP.isZero(0.0));
}

TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsReturnsWithoutConfiguredKF) {
  SET_UP();

  Vector6d imuVec;
  imuVec << 0.0, 0.0, 1e-7, -1e-7, 1e-8, -1e-8;

  Vector6d gpsVec;
  gpsVec << 5.06832, 47.319065, 0.0, 0.0, 0.0, 0.0;

  radarPositionNavigationController.KFCallbackWithGps(0.01, imuVec, gpsVec);

  ASSERT_TRUE(radarPositionNavigationController.m_latestX.isZero(0.0));
  ASSERT_TRUE(radarPositionNavigationController.m_latestP.isZero(0.0));
}

TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyProducesNonFiniteStateBecauseKFUsesSingularR) {
  SET_UP();

  radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.20, 0.95);
  radarPositionNavigationController.m_isKFConfigured.store(true);

  Vector6d imuVec;
  imuVec << 0.0, 0.0, 1e-9, -1e-9, 1e-10, -1e-10;

  radarPositionNavigationController.KFCallbackImuOnly(0.01, imuVec);

  ASSERT_TRUE(radarPositionNavigationController.m_isKFConfigured.load());
  ASSERT_FALSE(std::isfinite(radarPositionNavigationController.m_latestX(0)));
}

TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsProducesNonFiniteStateBecauseKFUsesSingularR) {
  SET_UP();

  radarPositionNavigationController.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.20, 0.95);
  radarPositionNavigationController.m_isKFConfigured.store(true);

  Vector6d gpsVec;
  gpsVec << 5.0683200001, 47.3190650001, 0.0, 0.0, 0.0, 0.0;

  Vector6d imuVec;
  imuVec << 0.0, 0.0, 1e-9, -1e-9, 1e-10, -1e-10;

  radarPositionNavigationController.KFCallbackWithGps(0.01, imuVec, gpsVec);

  ASSERT_TRUE(radarPositionNavigationController.m_isKFConfigured.load());
  ASSERT_FALSE(std::isfinite(radarPositionNavigationController.m_latestX(0)));
}
