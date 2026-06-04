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