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
boost::shared_ptr<DatabaseManager> db = boost::make_shared<DatabaseManager>("./IMUPROC_tests.db");

IMUGPSFusionKF_2D_ConstantAcceleration ekf;

auto ekfNoGps = [](double dt, Vector6d &z_IMU) { return ekf.Step(dt, z_IMU); };
auto ekfWithGps = [](double dt, Vector6d &z_GPS, Vector6d &z_IMU) { return ekf.Step(dt, z_GPS, z_IMU); };
} // namespace
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, GetStatsImuReject) {
  IMUManager imuManager(db);
  sh2_setSensorCallback(IMUManager::SensorCallback, &imuManager);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  sh2_SensorEvent_t events[] = {
      {.timestamp_uS = 900000,
       .delay_uS = 0,
       .len = 12,
       .reportId = SH2_ACCELEROMETER,
       .report = {0xFF, 0xF0, 0xF0, 0x00, 
                  0x10, 0xF7, 0x00, 0x00, 
                  0x20, 0xFE, 0x00, 0x00}},
      {.timestamp_uS = 1000000,
       .delay_uS = 0,
       .len = 12,
       .reportId = SH2_ACCELEROMETER,
       .report = {0x00, 0x00, 0x00, 0x00, 
                  0x10, 0x27, 0x00, 0x00, 
                  0x20, 0x4E, 0x00, 0x00}}
  };

  IMUManagerStats stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  imuManager.SensorCallback(&imuManager, nullptr);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 1);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  imuManager.SensorCallback(&imuManager, &events[0]);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 2);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  imuManager.SensorCallback(&imuManager, &events[1]);
  stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 3);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);
}
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, GetStatsImuAccept) {
  IMUManager imuManager(db);
  sh2_setSensorCallback(IMUManager::SensorCallback, &imuManager);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);
  std::vector<sh2_SensorEvent_t> events = {
      {.timestamp_uS = 1010000,
       .delay_uS = 10000,
       .len = 12,
       .reportId = SH2_LINEAR_ACCELERATION,
       .report = {0x01, 0x00, 0x00, 0x00, 
                  0x34, 0x12, 0x00, 0x00, 
                  0x78, 0x56, 0x00, 0x00}},
      {.timestamp_uS = 1020000,
       .delay_uS = 10000,
       .len = 16,
       .reportId = SH2_ROTATION_VECTOR,
       .report = {0x02, 0x00, 0x00, 0x00, 
                  0x11, 0x22, 0x33, 0x44, 
                  0x55, 0x66, 0x77, 0x88, 
                  0x99, 0xAA, 0xBB, 0xCC}}
  };

  IMUManagerStats stats = imuManager.GetStats();
  EXPECT_EQ(stats.imuAccepted, 0);
  EXPECT_EQ(stats.imuRejected, 0);
  EXPECT_EQ(stats.gpsAccepted, 0);
  EXPECT_EQ(stats.gpsRejected, 0);
  EXPECT_EQ(stats.dbEnqueueFailures, 0);

  for (int i = 0; i < events.size(); ++i) {
    imuManager.SensorCallback(&imuManager, &events[i]);
    stats = imuManager.GetStats();
    EXPECT_EQ(stats.imuAccepted, i + 1);
    EXPECT_EQ(stats.imuRejected, 0);
    EXPECT_EQ(stats.gpsAccepted, 0);
    EXPECT_EQ(stats.gpsRejected, 0);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);
  }
}
//-----------------------------------------------------------------------------
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
// //-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, GetLatestGpsReturnsNullopt) {
  IMUManager imuManager(db);
  EXPECT_EQ(imuManager.GetLatestGps(), std::nullopt); }
//-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, IsInvalidRangeReturnsFalse) {
  IMUManager imuManager(db);
  EXPECT_EQ(imuManager.IsInvalidRange(0), false);
  EXPECT_EQ(imuManager.IsInvalidRange(10), false);
  EXPECT_EQ(imuManager.IsInvalidRange(-10), false);
  EXPECT_EQ(imuManager.IsInvalidRange(std::numeric_limits<int>::max()), false);
  EXPECT_EQ(imuManager.IsInvalidRange(std::numeric_limits<double>::max()), false);
  EXPECT_EQ(imuManager.IsInvalidRange(std::numeric_limits<int>::min()), false);
  EXPECT_EQ(imuManager.IsInvalidRange(std::numeric_limits<double>::min()), false);
}
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, IsInvalidRangeReturnsTrue) {
  IMUManager imuManager(db);
  EXPECT_EQ(imuManager.IsInvalidRange(NAN), true);
  EXPECT_EQ(imuManager.IsInvalidRange(INFINITY), true);
}
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, ValidateImuEventReturnsTrue) {
  IMUManager imuManager(db);
  sh2_SensorValue sensorValue;

  sensorValue.sensorId = SH2_LINEAR_ACCELERATION;
  sensorValue.timestamp = 1;
  sensorValue.un.linearAcceleration.x = 1;
  sensorValue.un.linearAcceleration.y = 1;
  sensorValue.un.linearAcceleration.z = 1;

  EXPECT_EQ(imuManager.ValidateImuEvent(sensorValue), true);

  sensorValue.sensorId = SH2_ROTATION_VECTOR;
  sensorValue.un.rotationVector.i = 1;
  sensorValue.un.rotationVector.j = 1;
  sensorValue.un.rotationVector.k = 1;
  sensorValue.un.rotationVector.real = 1;

  EXPECT_EQ(imuManager.ValidateImuEvent(sensorValue), true);
}
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, ValidateImuEventReturnsFalse) {
  IMUManager imuManager(db);
  sh2_SensorValue sensorValue;

  sensorValue.timestamp = 1;
  sensorValue.un.linearAcceleration.x = 1;
  sensorValue.un.linearAcceleration.y = 1;
  sensorValue.un.linearAcceleration.z = NAN;

  std::vector<sh2_SensorValue> casesLA = {
      {SH2_LINEAR_ACCELERATION, 0, 0, 0, 0, {.linearAcceleration = {1, 1, NAN}}},
      {SH2_LINEAR_ACCELERATION, 0, 0, 0, 0, {.linearAcceleration = {1, NAN, 1}}},
      {SH2_LINEAR_ACCELERATION, 0, 0, 0, 0, {.linearAcceleration = {NAN, 1, 1}}},
      {SH2_LINEAR_ACCELERATION, 0, 0, 0, 0, {.linearAcceleration = {NAN, NAN, NAN}}},
  };

  for (auto &c : casesLA) {
    EXPECT_EQ(imuManager.ValidateImuEvent(c), false);
  }

  std::vector<sh2_SensorValue> casesRV = {
      {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {1, 1, 1, 1, NAN}}},
      {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {1, 1, 1, NAN, 1}}},
      {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {1, 1, NAN, 1, 1}}},
      {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {1, NAN, 1, 1, 1}}},
      {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {NAN, 1, 1, 1, 1}}},
      {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {NAN, NAN, NAN, NAN, NAN}}},
  };

  for (auto &c : casesRV) {
    EXPECT_EQ(imuManager.ValidateImuEvent(c), false);
  }
}
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, StoreImuValueReturnsVoid) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  sh2_Accelerometer la = imuManager.m_imuLinearAcceleration;
  sh2_RotationVectorWAcc rv = imuManager.m_imuRotationVector;
  EXPECT_NEAR(rv.i, 0, 1e-12);
  EXPECT_NEAR(rv.j, 0, 1e-12);
  EXPECT_NEAR(rv.k, 0, 1e-12);
  EXPECT_NEAR(rv.real, 0, 1e-12);
  EXPECT_NEAR(rv.accuracy, 0, 1e-12);
  EXPECT_NEAR(la.x, 0, 1e-12);
  EXPECT_NEAR(la.y, 0, 1e-12);
  EXPECT_NEAR(la.z, 0, 1e-12);

  sh2_SensorValue sensorValue;

  sensorValue.sensorId = SH2_LINEAR_ACCELERATION;
  sensorValue.timestamp = 1;
  sensorValue.un.linearAcceleration.x = 2;
  sensorValue.un.linearAcceleration.y = 3;
  sensorValue.un.linearAcceleration.z = 4;
  imuManager.StoreImuValue(sensorValue);
  la = imuManager.m_imuLinearAcceleration;
  EXPECT_NEAR(la.x, 2, 1e-12);
  EXPECT_NEAR(la.y, 3, 1e-12);
  EXPECT_NEAR(la.z, 4, 1e-12);

  sensorValue.sensorId = SH2_ROTATION_VECTOR;
  sensorValue.un.rotationVector.i = 1;
  sensorValue.un.rotationVector.j = 2;
  sensorValue.un.rotationVector.k = 3;
  sensorValue.un.rotationVector.real = 4;
  sensorValue.un.rotationVector.accuracy = 5;
  imuManager.StoreImuValue(sensorValue);
  rv = imuManager.m_imuRotationVector;
  EXPECT_NEAR(rv.i, 1, 1e-12);
  EXPECT_NEAR(rv.j, 2, 1e-12);
  EXPECT_NEAR(rv.k, 3, 1e-12);
  EXPECT_NEAR(rv.real, 4, 1e-12);
  EXPECT_NEAR(rv.accuracy, 5, 1e-12);

  sensorValue.sensorId = SH2_GAME_ROTATION_VECTOR;
  sensorValue.un.gameRotationVector.i = 1;
  sensorValue.un.gameRotationVector.j = 2;
  sensorValue.un.gameRotationVector.k = 3;
  sensorValue.un.gameRotationVector.real = 4;
  imuManager.StoreImuValue(sensorValue);
  la = imuManager.m_imuLinearAcceleration;
  rv = imuManager.m_imuRotationVector;
  EXPECT_NEAR(la.x, 2, 1e-12);
  EXPECT_NEAR(la.y, 3, 1e-12);
  EXPECT_NEAR(la.z, 4, 1e-12);
  EXPECT_NEAR(rv.i, 1, 1e-12);
  EXPECT_NEAR(rv.j, 2, 1e-12);
  EXPECT_NEAR(rv.k, 3, 1e-12);
  EXPECT_NEAR(rv.real, 4, 1e-12);
  EXPECT_NEAR(rv.accuracy, 5, 1e-12);
}
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, BuildGpsMeasurementVectorReturnsVector) {
  IMUManager imuManager(db);
  GpsUpdate gpsUpdate;
  gpsUpdate.latitude = 1;
  gpsUpdate.longitude = 1;

  Vector6d gpsVector = imuManager.BuildGpsMeasurementVector(gpsUpdate);
  Vector6d expected = {gpsUpdate.longitude, gpsUpdate.latitude, 0, 0, 0, 0};
  EXPECT_EQ(gpsVector, expected);
}
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, BuildImuMeasurementVectorReturnsVector) {
  IMUManager imuManager(db);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);
  double latitude = 80.0;
  double longitude = 0.0;

  sh2_RotationVectorWAcc rv = {0.032959, -0.061829, -0.706909, 0.703796, 0};

  sh2_Accelerometer la = {20, 50, 0};

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
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, IngestSensorValueThrowsRuntimeError) {
  IMUManager imuManager(db);
  sh2_setSensorCallback(IMUManager::SensorCallback, &imuManager);

  sh2_SensorValue_t value{};
  EXPECT_THROW(imuManager.IngestSensorValue(value), std::runtime_error);
}
//-----------------------------------------------------------------------------

TEST(IMUManagerTest, ReadyForEkfReturnsFalse) {
  IMUManager imuManager(db);
  sh2_setSensorCallback(IMUManager::SensorCallback, &imuManager);
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
  sh2_setSensorCallback(IMUManager::SensorCallback, &imuManager);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  imuManager.m_ekfInstalled = true;
  imuManager.m_imuRotationVectorReady = true;
  imuManager.m_imuLinearAccelerationReady = true;
  EXPECT_TRUE(imuManager.ReadyForEkf());
}
//-----------------------------------------------------------------------------

TEST(IMUManagerTest, GetCurrentYearReturnsYear) {
  IMUManager imuManager(db);
  sh2_setSensorCallback(IMUManager::SensorCallback, &imuManager);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);

  auto ymdNow = std::chrono::system_clock::now();
  const std::chrono::year_month_day ymd{std::chrono::floor<std::chrono::days>(ymdNow)};
  EXPECT_EQ(imuManager.GetCurrentYear(), static_cast<int>(ymd.year()));
}
//-----------------------------------------------------------------------------

TEST(IMUManagerTest, PrepareEkfTimingReturnsDtSeconds) {
  IMUManager imuManager(db);
  sh2_setSensorCallback(IMUManager::SensorCallback, &imuManager);
  imuManager.InstallEkf(ekfNoGps, ekfWithGps);
  imuManager.m_lastAccelerationVectorMachineTime = 1e6;
  imuManager.m_lastRotationVectorMachineTime = 2e6;
  imuManager.m_lastEKFMachineTime = 0;

  EXPECT_EQ(imuManager.PrepareEkfTiming(), 1);
  EXPECT_EQ(imuManager.m_lastEKFMachineTime, 1e6);
}
//-----------------------------------------------------------------------------
TEST(IMUManagerTest, ResetImuReadyFlagsExpectsFalse) {
  IMUManager imuManager(db);
  sh2_setSensorCallback(IMUManager::SensorCallback, &imuManager);
  imuManager.m_imuRotationVectorReady = false;
  imuManager.m_imuLinearAccelerationReady = true;

  imuManager.ResetImuReadyFlags();
  EXPECT_FALSE(imuManager.m_imuRotationVectorReady);

  imuManager.m_imuRotationVectorReady = true;
  imuManager.m_imuLinearAccelerationReady = false;
  imuManager.ResetImuReadyFlags();
  EXPECT_FALSE(imuManager.m_imuLinearAccelerationReady);
}
