#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "IMUManager.hpp"
#include "GpsUpdate.hpp"

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

TEST(IMUManagerTest, GetLatestGpsReturnsNullopt) {
    EXPECT_EQ(IMUManager::GetLatestGps(), std::nullopt);
}

TEST(IMUManagerTest, ValidateImuEventLinearAccelerationReturnsTrue) {
    sh2_SensorValue sensorValue;

    sensorValue.sensorId = SH2_LINEAR_ACCELERATION;
    sensorValue.timestamp = 1;
    sensorValue.un.linearAcceleration.x = 1;
    sensorValue.un.linearAcceleration.y = 1;
    sensorValue.un.linearAcceleration.z = 1;
    sensorValue.status = 1;

    EXPECT_EQ(IMUManager::ValidateImuEvent(sensorValue), true);

    sensorValue.sensorId = SH2_MAGNETIC_FIELD_CALIBRATED;
    sensorValue.un.magneticField.x = 1;
    sensorValue.un.magneticField.y = 1;
    sensorValue.un.magneticField.z = 1;

    EXPECT_EQ(IMUManager::ValidateImuEvent(sensorValue), true);

    sensorValue.sensorId = SH2_ROTATION_VECTOR;
    sensorValue.un.rotationVector.i = 1;
    sensorValue.un.rotationVector.j = 1;
    sensorValue.un.rotationVector.k = 1;
    sensorValue.un.rotationVector.real = 1;

    EXPECT_EQ(IMUManager::ValidateImuEvent(sensorValue), true);
}