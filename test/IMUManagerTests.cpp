#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "GpsUpdate.hpp"
#include "IMUManager.hpp"
#include "IMUGPSFusionKF.hpp"

class DatabaseManager {

};

namespace {
    void SetupIMUManager() {
        boost::shared_ptr<DatabaseManager> db = boost::make_shared<DatabaseManager>();

        IMUGPSFusionKF_2D_ConstantAcceleration ekf;
        auto ekfNoGps = [&ekf](double dt, Vector6d& z_IMU) {
                            return ekf.Step(dt, z_IMU);
                        };
        auto ekfWithGps = [&ekf](double dt, Vector6d& z_GPS, Vector6d& z_IMU) {
                                return ekf.Step(dt, z_GPS, z_IMU);
                            };

        IMUManager::Initialize(db,
                            ekfNoGps,
                            ekfWithGps);
    }

    void CleanupIMUManager() {
        IMUManager::Deinitialize();
    }
}

TEST(IMUManagerTest, GetLatestGpsReturnsNullopt) {
    EXPECT_EQ(IMUManager::GetLatestGps(), std::nullopt);
}

TEST(IMUManagerTest, UpdateLatestGpsReturnsValidGps) {
    SetupIMUManager();

    GpsUpdate gpsUpdate;
    gpsUpdate.latitude = 1;
    gpsUpdate.longitude = 1;
    gpsUpdate.valid = true;
    gpsUpdate.receiveTime = std::chrono::steady_clock::now();

    IMUManager::UpdateLatestGps(gpsUpdate);
    
    std::optional<GpsUpdate> latestGpsOpt = IMUManager::GetLatestGps();
    EXPECT_EQ(latestGpsOpt.has_value(), true);

    GpsUpdate latestGps = latestGpsOpt.value();
    EXPECT_EQ(latestGps.latitude, gpsUpdate.latitude);
    EXPECT_EQ(latestGps.longitude, gpsUpdate.longitude);
    EXPECT_EQ(latestGps.valid, gpsUpdate.valid);
    EXPECT_EQ(latestGps.receiveTime, gpsUpdate.receiveTime);

    CleanupIMUManager();
}

TEST(IMUManagerTest, UpdateLatestGpsReturnsInvalidGps) {
    SetupIMUManager();

    const std::chrono::seconds STALE_TIME_OUT_S(10);
    const uint64_t GPS_TIMESTAMP_MS= std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::milliseconds(1000)).count();
    const uint64_t GPS_TIMESTAMP_MS_INVALID= std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::milliseconds(900)).count();

    GpsUpdate gpsUpdate;
    gpsUpdate.latitude = 10;
    gpsUpdate.longitude = 20;
    std::optional<GpsUpdate> latestGpsOpt;

    // No GPS Update yet
    latestGpsOpt = IMUManager::GetLatestGps();
    EXPECT_EQ(latestGpsOpt.has_value(), false);

    // When gps status is invalid
    gpsUpdate.valid = false;
    IMUManager::UpdateLatestGps(gpsUpdate);
    latestGpsOpt = IMUManager::GetLatestGps();
    EXPECT_EQ(latestGpsOpt.has_value(), false);

    // When stale gps
    gpsUpdate.receiveTime = std::chrono::steady_clock::now() - STALE_TIME_OUT_S;
    IMUManager::UpdateLatestGps(gpsUpdate);
    latestGpsOpt = IMUManager::GetLatestGps();
    EXPECT_EQ(latestGpsOpt.has_value(), false);

    // Update IMUManager with valid gps
    gpsUpdate.valid = true;
    gpsUpdate.receiveTime = std::chrono::steady_clock::now();
    gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS;
    IMUManager::UpdateLatestGps(gpsUpdate);
    latestGpsOpt = IMUManager::GetLatestGps();
    EXPECT_EQ(latestGpsOpt.has_value(), true);
    GpsUpdate latestGps = latestGpsOpt.value();
    EXPECT_EQ(latestGps.latitude, gpsUpdate.latitude);
    EXPECT_EQ(latestGps.longitude, gpsUpdate.longitude);
    EXPECT_EQ(latestGps.valid, gpsUpdate.valid);
    EXPECT_EQ(latestGps.receiveTime, gpsUpdate.receiveTime);
    EXPECT_EQ(latestGps.gpsTimestampMs, GPS_TIMESTAMP_MS);

    // Update IMUManager with gpsTimestampMs older than s_latestGps.gpsTimestampMs
    // Expect no update to s_latestGps
    gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS_INVALID;
    IMUManager::UpdateLatestGps(gpsUpdate);
    latestGpsOpt = IMUManager::GetLatestGps();
    EXPECT_EQ(latestGpsOpt.has_value(), true);
    latestGps = latestGpsOpt.value();
    EXPECT_EQ(latestGps.latitude, gpsUpdate.latitude);
    EXPECT_EQ(latestGps.longitude, gpsUpdate.longitude);
    EXPECT_EQ(latestGps.valid, gpsUpdate.valid);
    EXPECT_EQ(latestGps.receiveTime, gpsUpdate.receiveTime);
    EXPECT_EQ(latestGps.gpsTimestampMs, GPS_TIMESTAMP_MS);

    CleanupIMUManager();
}

TEST(IMUManagerTest, IsInvalidRangeReturnsFalse) {
    EXPECT_EQ(IMUManager::IsInvalidRange(0), false);
    EXPECT_EQ(IMUManager::IsInvalidRange(10), false);
    EXPECT_EQ(IMUManager::IsInvalidRange(-10), false);
    EXPECT_EQ(IMUManager::IsInvalidRange(std::numeric_limits<int>::max()), false);
    EXPECT_EQ(IMUManager::IsInvalidRange(std::numeric_limits<double>::max()), false);
    EXPECT_EQ(IMUManager::IsInvalidRange(std::numeric_limits<int>::min()), false);
    EXPECT_EQ(IMUManager::IsInvalidRange(std::numeric_limits<double>::min()), false);
}

TEST(IMUManagerTest, IsInvalidRangeReturnsTrue) {
    EXPECT_EQ(IMUManager::IsInvalidRange(NAN), true);
    EXPECT_EQ(IMUManager::IsInvalidRange(INFINITY), true);
}

TEST(IMUManagerTest, ValidateImuEventReturnsTrue) {
    sh2_SensorValue sensorValue;

    sensorValue.sensorId = SH2_LINEAR_ACCELERATION;
    sensorValue.timestamp = 1;
    sensorValue.un.linearAcceleration.x = 1;
    sensorValue.un.linearAcceleration.y = 1;
    sensorValue.un.linearAcceleration.z = 1;

    EXPECT_EQ(IMUManager::ValidateImuEvent(sensorValue), true);

    sensorValue.sensorId = SH2_ROTATION_VECTOR;
    sensorValue.un.rotationVector.i = 1;
    sensorValue.un.rotationVector.j = 1;
    sensorValue.un.rotationVector.k = 1;
    sensorValue.un.rotationVector.real = 1;

    EXPECT_EQ(IMUManager::ValidateImuEvent(sensorValue), true);
}

TEST(IMUManagerTest, ValidateImuEventReturnsFalse) {
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

    for(auto& c : casesLA) {
        EXPECT_EQ(IMUManager::ValidateImuEvent(c), false);
    }

    std::vector<sh2_SensorValue> casesRV = {
        {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {1, 1, 1, 1, NAN}}},
        {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {1, 1, 1, NAN, 1}}},
        {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {1, 1, NAN, 1, 1}}},
        {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {1, NAN, 1, 1, 1}}},
        {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {NAN, 1, 1, 1, 1}}},
        {SH2_ROTATION_VECTOR, 0, 0, 0, 0, {.rotationVector = {NAN, NAN, NAN, NAN, NAN}}},
    };

    for(auto& c : casesRV) {
        EXPECT_EQ(IMUManager::ValidateImuEvent(c), false);
    }
}

TEST(IMUManagerTest, BuildGpsMeasurementVectorReturnsVector) {
    GpsUpdate gpsUpdate;
    gpsUpdate.latitude = 1;
    gpsUpdate.longitude = 1;

    Vector6d gpsVector = IMUManager::BuildGpsMeasurementVector(gpsUpdate);
    Vector6d expected = {gpsUpdate.longitude,
                         gpsUpdate.latitude,
                         0, 0, 0, 0};
    EXPECT_EQ(gpsVector, expected);
}

TEST(IMUManagerTest, BuildImuMeasurementVectorReturnsVector) {
    SetupIMUManager();
    double latitude = 80.0;
    double longitude = 0.0;

    sh2_RotationVectorWAcc rv = {
        0.032959,
        -0.061829,
        -0.706909,
        0.703796,
        0
    };

    sh2_Accelerometer la = {
        20,
        50,
        0
    };

    GpsUpdate gps;
    gps.latitude = 80.0;
    gps.longitude = 0;

    IMUManager::s_kineticState = {
        std::chrono::steady_clock::now(),
        0, 0, 0, 0
    };

    std::this_thread::sleep_for(std::chrono::seconds(1));
    Vector6d zImuT1Sec = IMUManager::BuildImuMeasurementVector(rv, la, gps, 2025);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    Vector6d zImuT2Sec = IMUManager::BuildImuMeasurementVector(rv, la, gps, 2025);

    EXPECT_NEAR(zImuT1Sec[0], 0, 1e-6);
    EXPECT_NEAR(zImuT1Sec[1], 0, 1e-16);
    EXPECT_NEAR(zImuT1Sec[2], -0.002575, 1e-4);
    EXPECT_NEAR(zImuT1Sec[3], 0.000187, 1e-4);
    EXPECT_NEAR(zImuT1Sec[4], -0.002575, 1e-4);
    EXPECT_NEAR(zImuT1Sec[5], 0.000187, 1e-4);

    EXPECT_NEAR(zImuT2Sec[0], 0, 1e-6);
    EXPECT_NEAR(zImuT2Sec[1], 0, 1e-16);
    EXPECT_NEAR(zImuT2Sec[2], -0.002575 * 2, 1e-4);
    EXPECT_NEAR(zImuT2Sec[3], 0.000187 * 2, 1e-4);
    EXPECT_NEAR(zImuT2Sec[4], -0.002575, 1e-4);
    EXPECT_NEAR(zImuT2Sec[5], 0.000187, 1e-4);

    CleanupIMUManager();
}