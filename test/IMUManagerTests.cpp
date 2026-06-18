#include <cmath>
#include <limits>
#include <vector>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

#include "DatabaseManager.hpp"
#include "GpsUpdate.hpp"
#include "IMUGPSFusionKF.hpp"
#include "IMUManager.hpp"

namespace {
std::shared_ptr<DatabaseManager> db = std::make_shared<DatabaseManager>("./IMUPROC_tests.db");

IMUGPSFusionKF_2D_ConstantAcceleration ekf;

auto ekfNoGps = [](double dt, Vector6d &z_IMU) { return ekf.Step(dt, z_IMU); };
auto ekfWithGps = [](double dt, Vector6d &z_GPS, Vector6d &z_IMU) { return ekf.Step(dt, z_GPS, z_IMU); };
} // namespace

TEST(IMUManagerTest, GetStatsImuReject) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  std::optional<Raw_Accelerometer> optLa{Raw_Accelerometer{1, NAN, 5, 1000000}};

  std::optional<Raw_RotationVectorWAcc> optRv = {Raw_RotationVectorWAcc{1, 1, INFINITY, 1, 0.1, 1000000}};

  IMUManagerStats stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  EXPECT_THROW(imuManager.SensorCallback(optRv, std::nullopt), std::runtime_error);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 1);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  EXPECT_THROW(imuManager.SensorCallback(std::nullopt, optLa), std::runtime_error);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 2);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  EXPECT_THROW(imuManager.SensorCallback(std::nullopt, std::nullopt), std::runtime_error);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 3);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);
}

TEST(IMUManagerTest, GetStatsImuAccept) {
  IMUManager imuManager(db);
  // imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  std::optional<Raw_Accelerometer> optLa = {Raw_Accelerometer{1, 1, 5, 1000000}};

  std::optional<Raw_RotationVectorWAcc> optRv = {Raw_RotationVectorWAcc{0.7, 0.5, 0.1, 1, 0.1, 1000000}};

  IMUManagerStats stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  imuManager.SensorCallback(std::nullopt, optLa);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 1);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  imuManager.SensorCallback(optRv, std::nullopt);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 2);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);
}

TEST(IMUManagerTest, GetStatsGpsReject) {
  const std::chrono::seconds STALE_TIME_OUT_S(10);
  const uint64_t GPS_TIMESTAMP_MS = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(1000)).count();
  const uint64_t GPS_TIMESTAMP_MS_INVALID = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(900)).count();

  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  IMUManagerStats stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  GpsUpdate gpsUpdate;
  gpsUpdate.latitude = 10;
  gpsUpdate.longitude = 20;

  // When gps status is invalid
  gpsUpdate.valid = false;
  imuManager.UpdateLatestGps(gpsUpdate);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 1);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  // When stale gps
  gpsUpdate.receiveTime = std::chrono::steady_clock::now() - STALE_TIME_OUT_S;
  imuManager.UpdateLatestGps(gpsUpdate);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 2);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  // Update IMUManager with valid gps
  gpsUpdate.valid = true;
  gpsUpdate.receiveTime = std::chrono::steady_clock::now();
  gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS;
  imuManager.UpdateLatestGps(gpsUpdate);

  // Update IMUManager with gpsTimestampMs older than m_latestGps.gpsTimestampMs
  // Expect no update to m_latestGps
  gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS_INVALID;
  imuManager.UpdateLatestGps(gpsUpdate);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 1);
  EXPECT_EQ(stats.gpsRejected, 3);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);
}

TEST(IMUManagerTest, GetStatsGpsAccept) {
  const std::chrono::seconds STALE_TIME_OUT_S(10);
  const uint64_t GPS_TIMESTAMP_MS = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(1000)).count();
  const uint64_t GPS_TIMESTAMP_MS_INVALID = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(900)).count();

  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  IMUManagerStats stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  GpsUpdate gpsUpdate;
  gpsUpdate.latitude = 10;
  gpsUpdate.longitude = 20;

  // Update IMUManager with valid gps
  gpsUpdate.valid = true;
  gpsUpdate.receiveTime = std::chrono::steady_clock::now();
  gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS;
  imuManager.UpdateLatestGps(gpsUpdate);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 1);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);
}

TEST(IMUManagerTest, GetLatestGpsReturnsNullopt) {
  IMUManager imuManager(db);
  EXPECT_EQ(imuManager.GetLatestGps(), std::nullopt);
}

TEST(IMUManagerTest, UpdateLatestGpsReturnsValidGps) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  GpsUpdate gpsUpdate;
  gpsUpdate.latitude = 1;
  gpsUpdate.longitude = 1;
  gpsUpdate.valid = true;
  gpsUpdate.receiveTime = std::chrono::steady_clock::now();

  imuManager.UpdateLatestGps(gpsUpdate);

  std::optional<GpsUpdate> latestGpsOpt = imuManager.GetLatestGps();
  EXPECT_EQ(latestGpsOpt.has_value(), true);

  GpsUpdate latestGps = latestGpsOpt.value();
  EXPECT_EQ(latestGps.latitude, gpsUpdate.latitude);
  EXPECT_EQ(latestGps.longitude, gpsUpdate.longitude);
  EXPECT_EQ(latestGps.valid, gpsUpdate.valid);
  EXPECT_EQ(latestGps.receiveTime, gpsUpdate.receiveTime);
}

TEST(IMUManagerTest, UpdateLatestGpsReturnsInvalidGps) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  const std::chrono::seconds STALE_TIME_OUT_S(10);
  const uint64_t GPS_TIMESTAMP_MS = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(1000)).count();
  const uint64_t GPS_TIMESTAMP_MS_INVALID = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::milliseconds(900)).count();

  GpsUpdate gpsUpdate;
  gpsUpdate.latitude = 10;
  gpsUpdate.longitude = 20;
  std::optional<GpsUpdate> latestGpsOpt;

  // No GPS Update yet
  latestGpsOpt = imuManager.GetLatestGps();
  EXPECT_EQ(latestGpsOpt.has_value(), false);

  // When gps status is invalid
  gpsUpdate.valid = false;
  imuManager.UpdateLatestGps(gpsUpdate);
  latestGpsOpt = imuManager.GetLatestGps();
  EXPECT_EQ(latestGpsOpt.has_value(), false);

  // When stale gps
  gpsUpdate.receiveTime = std::chrono::steady_clock::now() - STALE_TIME_OUT_S;
  imuManager.UpdateLatestGps(gpsUpdate);
  latestGpsOpt = imuManager.GetLatestGps();
  EXPECT_EQ(latestGpsOpt.has_value(), false);

  // Update IMUManager with valid gps
  gpsUpdate.valid = true;
  gpsUpdate.receiveTime = std::chrono::steady_clock::now();
  gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS;
  imuManager.UpdateLatestGps(gpsUpdate);
  latestGpsOpt = imuManager.GetLatestGps();
  EXPECT_EQ(latestGpsOpt.has_value(), true);
  GpsUpdate latestGps = latestGpsOpt.value();
  EXPECT_EQ(latestGps.latitude, gpsUpdate.latitude);
  EXPECT_EQ(latestGps.longitude, gpsUpdate.longitude);
  EXPECT_EQ(latestGps.valid, gpsUpdate.valid);
  EXPECT_EQ(latestGps.receiveTime, gpsUpdate.receiveTime);
  EXPECT_EQ(latestGps.gpsTimestampMs, GPS_TIMESTAMP_MS);

  // Update IMUManager with gpsTimestampMs older than m_latestGps.gpsTimestampMs
  // Expect no update to m_latestGps
  gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS_INVALID;
  imuManager.UpdateLatestGps(gpsUpdate);
  latestGpsOpt = imuManager.GetLatestGps();
  EXPECT_EQ(latestGpsOpt.has_value(), true);
  latestGps = latestGpsOpt.value();
  EXPECT_EQ(latestGps.latitude, gpsUpdate.latitude);
  EXPECT_EQ(latestGps.longitude, gpsUpdate.longitude);
  EXPECT_EQ(latestGps.valid, gpsUpdate.valid);
  EXPECT_EQ(latestGps.receiveTime, gpsUpdate.receiveTime);
  EXPECT_EQ(latestGps.gpsTimestampMs, GPS_TIMESTAMP_MS);
}

TEST(IMUManagerTest, IsInvalidRangeReturnsFalse) {
  IMUManager imuManager(db);
  EXPECT_FALSE(imuManager.IsInvalidRange(0));
  EXPECT_FALSE(imuManager.IsInvalidRange(10));
  EXPECT_FALSE(imuManager.IsInvalidRange(-10));
}

TEST(IMUManagerTest, IsInvalidRangeReturnsTrue) {
  IMUManager imuManager(db);
  EXPECT_TRUE(imuManager.IsInvalidRange(NAN));
  EXPECT_TRUE(imuManager.IsInvalidRange(INFINITY));
  EXPECT_TRUE(imuManager.IsInvalidRange(std::numeric_limits<int>::max()));
  EXPECT_TRUE(imuManager.IsInvalidRange(std::numeric_limits<double>::max()));
  EXPECT_TRUE(imuManager.IsInvalidRange(std::numeric_limits<int>::min()));
  EXPECT_TRUE(imuManager.IsInvalidRange(std::numeric_limits<double>::min()));
}

TEST(IMUManagerTest, ValidateImuEventReturnsTrue) {
  IMUManager imuManager(db);
  Raw_Accelerometer la = {1, 2, 3, 1000000};

  EXPECT_TRUE(imuManager.ValidateImuEvent(std::nullopt, la));

  Raw_RotationVectorWAcc rv = {0.7, 0.5, 0.3, 0.1, 0.1, 1000000};

  EXPECT_TRUE(imuManager.ValidateImuEvent(rv, std::nullopt));
}

TEST(IMUManagerTest, ValidateImuEventReturnsFalse) {
  IMUManager imuManager(db);

  float maxLimitF = std::numeric_limits<float>::max();
  uint64_t maxLimitD = std::numeric_limits<uint64_t>::max();

  std::vector<std::optional<Raw_Accelerometer>> casesLA = {
      {Raw_Accelerometer{maxLimitF, 1, 1, 100000}},
      {Raw_Accelerometer{1, maxLimitF, 1, 100000}},
      {Raw_Accelerometer{1, 1, maxLimitF, 100000}},
      {Raw_Accelerometer{1, 1, 1, maxLimitD}},
  };

  for (auto &c : casesLA) {
    EXPECT_FALSE(imuManager.ValidateImuEvent(std::nullopt, c));
  }

  std::vector<std::optional<Raw_RotationVectorWAcc>> casesRV = {
      {Raw_RotationVectorWAcc{maxLimitF, 1, 1, 1, 1, 100000}}, {Raw_RotationVectorWAcc{1, maxLimitF, 1, 1, 1, 100000}},
      {Raw_RotationVectorWAcc{1, 1, maxLimitF, 1, 1, 100000}}, {Raw_RotationVectorWAcc{1, 1, 1, maxLimitF, 1, 100000}},
      {Raw_RotationVectorWAcc{1, 1, 1, 1, maxLimitF, 100000}}, {Raw_RotationVectorWAcc{1, 1, 1, 1, 1, maxLimitD}},
  };

  for (auto &c : casesRV) {
    EXPECT_FALSE(imuManager.ValidateImuEvent(c, std::nullopt));
  }

  EXPECT_FALSE(imuManager.ValidateImuEvent(std::nullopt, std::nullopt));
}

TEST(IMUManagerTest, StoreImuValueReturnsVoid) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  Raw_Accelerometer la = imuManager.m_imuLinearAcceleration;
  Raw_RotationVectorWAcc rv = imuManager.m_imuRotationVector;
  EXPECT_NEAR(rv.i, 0, 1e-12);
  EXPECT_NEAR(rv.j, 0, 1e-12);
  EXPECT_NEAR(rv.k, 0, 1e-12);
  EXPECT_NEAR(rv.real, 0, 1e-12);
  EXPECT_NEAR(rv.accuracy, 0, 1e-12);
  EXPECT_NEAR(rv.timestamp, 0, 1e-12);
  EXPECT_NEAR(la.x, 0, 1e-12);
  EXPECT_NEAR(la.y, 0, 1e-12);
  EXPECT_NEAR(la.z, 0, 1e-12);
  EXPECT_NEAR(la.timestamp, 0, 1e-12);

  std::optional<Raw_Accelerometer> optLa = {Raw_Accelerometer{1, 1, 5, 100000}};

  imuManager.StoreImuValue(std::nullopt, optLa);
  la = imuManager.m_imuLinearAcceleration;
  EXPECT_NEAR(la.x, 1, 1e-12);
  EXPECT_NEAR(la.y, 1, 1e-12);
  EXPECT_NEAR(la.z, 5, 1e-12);
  EXPECT_NEAR(la.timestamp, 100000, 1e-12);

  std::optional<Raw_RotationVectorWAcc> optRv = {Raw_RotationVectorWAcc{0.7, 0.5, 0.1, 1, 0.1, 100000}};

  imuManager.StoreImuValue(optRv, std::nullopt);
  rv = imuManager.m_imuRotationVector;
  EXPECT_NEAR(rv.i, 0.7, 1e-4);
  EXPECT_NEAR(rv.j, 0.5, 1e-4);
  EXPECT_NEAR(rv.k, 0.1, 1e-4);
  EXPECT_NEAR(rv.real, 1, 1e-4);
  EXPECT_NEAR(rv.accuracy, 0.1, 1e-4);
  EXPECT_NEAR(rv.timestamp, 100000, 1e-4);
}

TEST(IMUManagerTest, BuildGpsMeasurementVectorReturnsVector) {
  IMUManager imuManager(db);
  GpsUpdate gpsUpdate;
  gpsUpdate.latitude = 1;
  gpsUpdate.longitude = 1;

  Vector6d gpsVector = imuManager.BuildGpsMeasurementVector(gpsUpdate);
  Vector6d expected = {gpsUpdate.longitude, gpsUpdate.latitude, 0, 0, 0, 0};
  EXPECT_EQ(gpsVector, expected);
}

TEST(IMUManagerTest, BuildImuMeasurementVectorReturnsVector) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);
  double latitude = 80.0;
  double longitude = 0.0;

  Raw_RotationVectorWAcc rv = {0.032959, -0.061829, -0.706909, 0.703796, 0, 0};

  Raw_Accelerometer la = {20, 50, 0, 0};

  GpsUpdate gps;
  gps.latitude = 80.0;
  gps.longitude = 0;

  imuManager.m_kineticState = {std::chrono::steady_clock::now(), 0, 0, 0, 0};

  std::this_thread::sleep_for(std::chrono::seconds(1));
  Vector6d zImuT1Sec = imuManager.BuildImuMeasurementVector(rv, la, gps, 2025);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  Vector6d zImuT2Sec = imuManager.BuildImuMeasurementVector(rv, la, gps, 2025);

  EXPECT_NEAR(zImuT1Sec[0], 0, 1e-6);
  EXPECT_NEAR(zImuT1Sec[1], 0, 1e-16);
  EXPECT_NEAR(zImuT1Sec[2], -0.002575, 5e-4);
  EXPECT_NEAR(zImuT1Sec[3], 0.000187, 1e-4);
  EXPECT_NEAR(zImuT1Sec[4], -0.002575, 5e-4);
  EXPECT_NEAR(zImuT1Sec[5], 0.000187, 1e-4);

  EXPECT_NEAR(zImuT2Sec[0], 0, 1e-6);
  EXPECT_NEAR(zImuT2Sec[1], 0, 1e-16);
  EXPECT_NEAR(zImuT2Sec[2], -0.002575 * 2, 5e-4);
  EXPECT_NEAR(zImuT2Sec[3], 0.000187 * 2, 1e-4);
  EXPECT_NEAR(zImuT2Sec[4], -0.002575, 5e-4);
  EXPECT_NEAR(zImuT2Sec[5], 0.000187, 1e-4);
}

TEST(IMUManagerTest, ReadyForEkfReturnsFalse) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  imuManager.m_ekfInstalled = false;
  imuManager.m_imuRotationVectorReady = false;
  imuManager.m_imuLinearAccelerationReady = false;
  EXPECT_FALSE(imuManager.ReadyForEkf());

  imuManager.m_ekfInstalled = false;
  imuManager.m_imuRotationVectorReady = false;
  imuManager.m_imuLinearAccelerationReady = true;
  EXPECT_FALSE(imuManager.ReadyForEkf());

  imuManager.m_ekfInstalled = false;
  imuManager.m_imuRotationVectorReady = true;
  imuManager.m_imuLinearAccelerationReady = true;
  EXPECT_FALSE(imuManager.ReadyForEkf());
}

TEST(IMUManagerTest, ReadyForEkfReturnsTrue) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  imuManager.m_ekfInstalled = true;
  imuManager.m_imuRotationVectorReady = true;
  imuManager.m_imuLinearAccelerationReady = true;
  EXPECT_TRUE(imuManager.ReadyForEkf());
}

TEST(IMUManagerTest, GetCurrentYearReturnsYear) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  auto ymdNow = std::chrono::system_clock::now();
  const std::chrono::year_month_day ymd{std::chrono::floor<std::chrono::days>(ymdNow)};
  EXPECT_EQ(imuManager.GetCurrentYear(), static_cast<int>(ymd.year()));
}

TEST(IMUManagerTest, PrepareEkfTimingReturnsDtSeconds) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);
  imuManager.m_lastEKFMachineTime = 0;

  EXPECT_EQ(imuManager.PrepareEkfTiming(), 0);
  EXPECT_EQ(imuManager.m_lastEKFMachineTime, 0);

  imuManager.m_imuLinearAcceleration.timestamp = static_cast<uint64_t>(1e6);
  imuManager.m_imuRotationVector.timestamp = static_cast<uint64_t>(2e6);

  EXPECT_EQ(imuManager.PrepareEkfTiming(), 1);
  EXPECT_EQ(imuManager.m_lastEKFMachineTime, 1e6);
}

TEST(IMUManagerTest, ResetImuReadyFlagsExpectsFalse) {
  IMUManager imuManager(db);
  imuManager.m_imuRotationVectorReady = false;
  imuManager.m_imuLinearAccelerationReady = true;

  imuManager.ResetImuReadyFlags();
  EXPECT_FALSE(imuManager.m_imuRotationVectorReady);

  imuManager.m_imuRotationVectorReady = true;
  imuManager.m_imuLinearAccelerationReady = false;
  imuManager.ResetImuReadyFlags();
  EXPECT_FALSE(imuManager.m_imuLinearAccelerationReady);
}
