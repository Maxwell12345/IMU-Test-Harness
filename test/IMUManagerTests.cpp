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
    boost::shared_ptr<DatabaseManager> db = boost::make_shared<DatabaseManager>();

    IMUGPSFusionKF_2D_ConstantAcceleration ekf;

    auto ekfNoGps = [](double dt, Vector6d& z_IMU) {
                        return ekf.Step(dt, z_IMU);
                    };
    auto ekfWithGps = [](double dt, Vector6d& z_GPS, Vector6d& z_IMU) {
                        return ekf.Step(dt, z_GPS, z_IMU);
                    };
}


TEST(IMUManagerTest, GetStatsImuReject) {
    IMUManager imuManager(db);
    IMUManager::InstallEkf(ekfNoGps,
                           ekfWithGps);

    sh2_SensorEvent_t events[] = {
        {
            .timestamp_uS = 900000,
            .delay_uS = 0,
            .len = 12,
            .reportId = SH2_ACCELEROMETER,
            .report = {
                0xFF, 0xF0, 0xF0, 0x00,
                0x10, 0xF7, 0x00, 0x00,
                0x20, 0xFE, 0x00, 0x00
            }
        },
        {
            .timestamp_uS = 1000000,
            .delay_uS = 0,
            .len = 12,
            .reportId = SH2_ACCELEROMETER,
            .report = {
                0x00, 0x00, 0x00, 0x00,
                0x10, 0x27, 0x00, 0x00,
                0x20, 0x4E, 0x00, 0x00
            }
        }
    };

    GpsUpdate gpsUpdate;
    gpsUpdate.latitude = 1;
    gpsUpdate.longitude = 1;
    gpsUpdate.valid = true;
    gpsUpdate.receiveTime = std::chrono::steady_clock::now();

    IMUManagerStats stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 0);
    EXPECT_EQ(stats.gpsAccepted, 0);
    EXPECT_EQ(stats.gpsRejected, 0);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);

    IMUManager::SensorCallback(nullptr, nullptr);
    stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 1);
    EXPECT_EQ(stats.gpsAccepted, 0);
    EXPECT_EQ(stats.gpsRejected, 0);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);

    IMUManager::SensorCallback(nullptr, &events[0]);
    stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 2);
    EXPECT_EQ(stats.gpsAccepted, 0);
    EXPECT_EQ(stats.gpsRejected, 0);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);

    IMUManager::SensorCallback(nullptr, &events[1]);
    stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 3);
    EXPECT_EQ(stats.gpsAccepted, 0);
    EXPECT_EQ(stats.gpsRejected, 0);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);
}

TEST(IMUManagerTest, GetStatsImuAccept) {
    IMUManager imuManager(db);
    IMUManager::InstallEkf(ekfNoGps,
                           ekfWithGps);

    std::vector<sh2_SensorEvent_t> events = {
        {
            .timestamp_uS = 1010000,
            .delay_uS = 10000,
            .len = 12,
            .reportId = SH2_LINEAR_ACCELERATION,
            .report = {
                0x01, 0x00, 0x00, 0x00,
                0x34, 0x12, 0x00, 0x00,
                0x78, 0x56, 0x00, 0x00
            }
        },
        {
            .timestamp_uS = 1020000,
            .delay_uS = 10000,
            .len = 16,
            .reportId = SH2_ROTATION_VECTOR,
            .report = {
                0x02, 0x00, 0x00, 0x00,
                0x11, 0x22, 0x33, 0x44,
                0x55, 0x66, 0x77, 0x88,
                0x99, 0xAA, 0xBB, 0xCC
            }
        }
    };

    IMUManagerStats stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 0);
    EXPECT_EQ(stats.gpsAccepted, 0);
    EXPECT_EQ(stats.gpsRejected, 0);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);

    for(int i = 0; i < events.size(); ++i) {
        IMUManager::SensorCallback(nullptr, &events[i]);
        stats = IMUManager::GetStats();
        EXPECT_EQ(stats.imuAccepted, i + 1);
        EXPECT_EQ(stats.imuRejected, 0);
        EXPECT_EQ(stats.gpsAccepted, 0);
        EXPECT_EQ(stats.gpsRejected, 0);
        EXPECT_EQ(stats.dbEnqueueFailures, 0);
    }
}

TEST(IMUManagerTest, GetStatsGpsReject) {
    const std::chrono::seconds STALE_TIME_OUT_S(10);
    const uint64_t GPS_TIMESTAMP_MS= std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::milliseconds(1000)).count();
    const uint64_t GPS_TIMESTAMP_MS_INVALID= std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::milliseconds(900)).count();

    IMUManager imuManager(db);
    IMUManager::InstallEkf(ekfNoGps,
                           ekfWithGps);

    IMUManagerStats stats = IMUManager::GetStats();
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
    IMUManager::UpdateLatestGps(gpsUpdate);
    stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 0);
    EXPECT_EQ(stats.gpsAccepted, 0);
    EXPECT_EQ(stats.gpsRejected, 1);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);

    // When stale gps
    gpsUpdate.receiveTime = std::chrono::steady_clock::now() - STALE_TIME_OUT_S;
    IMUManager::UpdateLatestGps(gpsUpdate);
    stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 0);
    EXPECT_EQ(stats.gpsAccepted, 0);
    EXPECT_EQ(stats.gpsRejected, 2);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);

    // Update IMUManager with valid gps
    gpsUpdate.valid = true;
    gpsUpdate.receiveTime = std::chrono::steady_clock::now();
    gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS;
    IMUManager::UpdateLatestGps(gpsUpdate);

    // Update IMUManager with gpsTimestampMs older than m_sLatestGps.gpsTimestampMs
    // Expect no update to m_sLatestGps
    gpsUpdate.gpsTimestampMs = GPS_TIMESTAMP_MS_INVALID;
    IMUManager::UpdateLatestGps(gpsUpdate);
    stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 0);
    EXPECT_EQ(stats.gpsAccepted, 1);
    EXPECT_EQ(stats.gpsRejected, 3);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);
}

TEST(IMUManagerTest, GetStatsGpsAccept) {
    const std::chrono::seconds STALE_TIME_OUT_S(10);
    const uint64_t GPS_TIMESTAMP_MS= std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::milliseconds(1000)).count();
    const uint64_t GPS_TIMESTAMP_MS_INVALID= std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::milliseconds(900)).count();

    IMUManager imuManager(db);
    IMUManager::InstallEkf(ekfNoGps,
                           ekfWithGps);

    IMUManagerStats stats = IMUManager::GetStats();
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
    IMUManager::UpdateLatestGps(gpsUpdate);
    stats = IMUManager::GetStats();
    EXPECT_EQ(stats.imuAccepted, 0);
    EXPECT_EQ(stats.imuRejected, 0);
    EXPECT_EQ(stats.gpsAccepted, 1);
    EXPECT_EQ(stats.gpsRejected, 0);
    EXPECT_EQ(stats.dbEnqueueFailures, 0);
}

TEST(IMUManagerTest, GetLatestGpsReturnsNullopt) {
    EXPECT_EQ(IMUManager::GetLatestGps(), std::nullopt);
}

TEST(IMUManagerTest, UpdateLatestGpsReturnsValidGps) {
    IMUManager imuManager(db);
    IMUManager::InstallEkf(ekfNoGps,
                           ekfWithGps);

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
}

TEST(IMUManagerTest, UpdateLatestGpsReturnsInvalidGps) {
    IMUManager imuManager(db);
    IMUManager::InstallEkf(ekfNoGps,
                           ekfWithGps);

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

    // Update IMUManager with gpsTimestampMs older than m_sLatestGps.gpsTimestampMs
    // Expect no update to m_sLatestGps
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

TEST(IMUManagerTest, StoreImuValueReturnsVoid) {
    IMUManager imuManager(db);
    IMUManager::InstallEkf(ekfNoGps,
                           ekfWithGps);

    sh2_Accelerometer la = IMUManager::m_sImuLinearAcceleration;
    sh2_RotationVectorWAcc rv = IMUManager::m_sImuRotationVector;
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
    IMUManager::StoreImuValue(sensorValue);
    la = IMUManager::m_sImuLinearAcceleration;
    EXPECT_NEAR(la.x, 2, 1e-12);
    EXPECT_NEAR(la.y, 3, 1e-12);
    EXPECT_NEAR(la.z, 4, 1e-12);

    sensorValue.sensorId = SH2_ROTATION_VECTOR;
    sensorValue.un.rotationVector.i = 1;
    sensorValue.un.rotationVector.j = 2;
    sensorValue.un.rotationVector.k = 3;
    sensorValue.un.rotationVector.real = 4;
    sensorValue.un.rotationVector.accuracy = 5;
    IMUManager::StoreImuValue(sensorValue);
    rv = IMUManager::m_sImuRotationVector;
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
    IMUManager::StoreImuValue(sensorValue);
    la = IMUManager::m_sImuLinearAcceleration;
    rv = IMUManager::m_sImuRotationVector;
    EXPECT_NEAR(la.x, 2, 1e-12);
    EXPECT_NEAR(la.y, 3, 1e-12);
    EXPECT_NEAR(la.z, 4, 1e-12);
    EXPECT_NEAR(rv.i, 1, 1e-12);
    EXPECT_NEAR(rv.j, 2, 1e-12);
    EXPECT_NEAR(rv.k, 3, 1e-12);
    EXPECT_NEAR(rv.real, 4, 1e-12);
    EXPECT_NEAR(rv.accuracy, 5, 1e-12);
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
    IMUManager imuManager(db);
    IMUManager::InstallEkf(ekfNoGps,
                           ekfWithGps);
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

    IMUManager::m_sKineticState = {
        std::chrono::steady_clock::now(),
        0, 0, 0, 0
    };

    std::this_thread::sleep_for(std::chrono::seconds(1));
    Vector6d zImuT1Sec = IMUManager::BuildImuMeasurementVector(rv, la, gps, 2025);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    Vector6d zImuT2Sec = IMUManager::BuildImuMeasurementVector(rv, la, gps, 2025);

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
