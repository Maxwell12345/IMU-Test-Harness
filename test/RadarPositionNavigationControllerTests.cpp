#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <limits>
#include <thread>

// #include "./third_party/sh2/sh2_hal.h"
#include <boost/make_shared.hpp>

#include "IMUManager.hpp"
#include "RadarPositionNavigationController.hpp"

namespace {
  std::atomic<int> sh2OpenCalls{0};
  std::atomic<int> sh2CloseCalls{0};
  std::atomic<int> sh2SetSensorCallbackCalls{0};
  std::atomic<int> sh2SetSensorConfigCalls{0};
  std::atomic<int> sh2ServiceCalls{0};
  std::atomic<int> bno085HalCreateCalls{0};
  std::atomic<int> forceOpenFailure{0};
} // namespace

namespace {
  std::shared_ptr<DatabaseManager> db = std::make_shared<DatabaseManager>("./IMUPROC_tests.db");

  #ifdef _WIN32
      std::string path = R"(COM1)";
  #else
      std::string path = "/dev/ttyUSB0";
  #endif

}

class MockSerialPort : public SerialPortBase {};


TEST(RadarPositionNavigationControllerTest, GetGPSCallbackUpdatesLatestGps) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

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

  // std::function<void(const GpsUpdate &)> callback = t.GetGPSCallback();
  auto callback = t.GetGPSCallback();
  callback(gps);

  // std::optional<GpsUpdate> latest = imuManager.GetLatestGps();
  auto latest = t.m_imuManager.GetLatestGps();

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

TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterSetsInitialStateAndCovariance) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

  t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.20, 0.95);

  ASSERT_NEAR(t.m_latestX(0), 5.06832, 1e-12);
  ASSERT_NEAR(t.m_latestX(1), 47.319065, 1e-12);
  ASSERT_NEAR(t.m_latestX(2), 1e-15, 1e-20);
  ASSERT_NEAR(t.m_latestX(3), 1e-15, 1e-20);
  ASSERT_NEAR(t.m_latestX(4), 1e-16, 1e-21);
  ASSERT_NEAR(t.m_latestX(5), 1e-16, 1e-21);

  ASSERT_NEAR(t.m_latestP(0, 0), 1e-10, 1e-20);
  ASSERT_NEAR(t.m_latestP(1, 1), 1e-10, 1e-20);
  ASSERT_NEAR(t.m_latestP(2, 2), 1e-8, 1e-18);
  ASSERT_NEAR(t.m_latestP(3, 3), 1e-8, 1e-18);
  ASSERT_NEAR(t.m_latestP(4, 4), 1e-10, 1e-20);
  ASSERT_NEAR(t.m_latestP(5, 5), 1e-10, 1e-20);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i != j) {
        ASSERT_NEAR(t.m_latestP(i, j), 0.0, 1e-30) << "Index i: " + std::to_string(i) + " and j: " +
        std::to_string(j);
      }
    }
  }
}

TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsInvalidPercentiles) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

  ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.0, 0.95, 0.20, 0.95), std::runtime_error);
  ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 1.0, 0.95, 0.20, 0.95), std::runtime_error);
  ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.0, 0.95), std::runtime_error);
  ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 1.0, 0.95), std::runtime_error);
}

TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsLowerPercentileGreaterThanUpperPercentile) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

  ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.95, 0.20, 0.20, 0.95), std::runtime_error);
  ASSERT_THROW(t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.95, 0.20), std::runtime_error);
}

TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyReturnsWithoutConfiguredKF) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

  Vector6d imuVec;
  imuVec << 0.0, 0.0, 1e-7, -1e-7, 1e-8, -1e-8;

  t.KFCallbackImuOnly(0.01, imuVec);

  ASSERT_TRUE(t.m_latestX.isZero(0.0));
  ASSERT_TRUE(t.m_latestP.isZero(0.0));
}

TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsReturnsWithoutConfiguredKF) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

  Vector6d imuVec;
  imuVec << 0.0, 0.0, 1e-7, -1e-7, 1e-8, -1e-8;

  Vector6d gpsVec;
  gpsVec << 5.06832, 47.319065, 0.0, 0.0, 0.0, 0.0;

  t.KFCallbackWithGps(0.01, imuVec, gpsVec);

  ASSERT_TRUE(t.m_latestX.isZero(0.0));
  ASSERT_TRUE(t.m_latestP.isZero(0.0));
}

TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyProducesNonFiniteStateBecauseKFUsesSingularR) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

  t.ConfigureKalmanFilter(47.319065, 5.06832, 0.20, 0.95, 0.20, 0.95);
  t.m_isKFConfigured.store(true);

  Vector6d imuVec;
  imuVec << 0.0, 0.0, 1e-9, -1e-9, 1e-10, -1e-10;

  t.KFCallbackImuOnly(0.01, imuVec);

  ASSERT_TRUE(t.m_isKFConfigured.load());
  ASSERT_FALSE(std::isfinite(t.m_latestX(0)));
}

TEST(RadarPositionNavigationControllerTest, YamlFileParsingForKalmanFilterValuesExpectingTryCatch) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

  EXPECT_THROW(t.ParseYamlForKalmanFilterValues("./dne.yml"), std::runtime_error);
}

TEST(RadarPositionNavigationControllerTest, YamlFileParsingForKalmanFilterValuesExpecting) {
  auto imuSerialPortReader = std::make_unique<IMUSerialPortReader>(path, 9600, std::make_unique<MockSerialPort>());                                                                                                                                                           
  RadarPositionNavigationController t(db, std::move(imuSerialPortReader));

  std::string filepath = "./compose.yml";
  EXPECT_NO_THROW(t.ParseYamlForKalmanFilterValues(filepath));

  // VALID RANGE: 0 <= x <= 1
  ASSERT_NEAR(t.m_gpsChiSqLowerBound, 0.20, 1e-6);
  ASSERT_NEAR(t.m_gpsChiSqUpperBound, 0.95, 1e-6);
  ASSERT_NEAR(t.m_imuChiSqLowerBound, 0.20, 1e-6);
  ASSERT_NEAR(t.m_imuChiSqUpperBound, 0.95, 1e-6);
}