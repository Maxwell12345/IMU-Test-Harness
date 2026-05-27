/******************************************************************************
 * Filename:     imu_utils_tests.cpp
 *
 * Author:       Tran Sgt Brandon, Gomez LCpl Greg
 * Organization: Marine Corps Software Factory
 * Created On:   5/21/2026 2:45 PM
 * Description:  This test files validates (1) DegreesToRadians, (2) InertialToGlobal_X and (3) InertialToGlobal_Y functionality.
 *
 ******************************************************************************/

#include "utils.hpp"
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>

using namespace IMUUtils;

TEST(IMUUtils, Convert_Degrees_To_Radians)
{

    struct Case
    {
        double degrees_input;
        double degrees_expected;
    };

    const std::vector<Case> cases = {
        // These tests are pass correct
        {-45.0, M_PI * 7 / 4},
        {0.0, M_PI * 0},
        {45.0, M_PI / 4},
        {90.0, M_PI / 2},
        {135.0, M_PI * 3 / 4},
        {180.0, M_PI},
        {225.0, M_PI * 5 / 4},
        {270.0, M_PI * 3 / 2},
        {315.0, M_PI * 7 / 4},
        {360.0, 0.0},
        {405.0, M_PI / 4}};

    for (const auto &c : cases)
    {
        EXPECT_NEAR(IMUUtils::DegreesToRadians(c.degrees_input), c.degrees_expected, 1e-9);
    };
};
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
TEST(IMUUtils, Convert_Local_XY_Acceleration_To_Global_X_Acceleration)
{
    struct Case
    {
        double theta_input;
        double x_accel_input;
        double y_accel_input;
        double x_accel_output;
    };

    const std::vector<Case> cases = {
        // Translations about 0.0
        {IMUUtils::DegreesToRadians(0.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 0.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, 0.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), 0.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, 0.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, 1.0, -1.0},
        // Translations about 45.0
        {IMUUtils::DegreesToRadians(45.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), 0.0, 1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 0.0, -1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), -1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, 1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), -1.0, -1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), -1.0, 1.0, 0.0},
        // Translations about 90.0
        {IMUUtils::DegreesToRadians(90.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 0.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 0.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, 1.0, 1.0},
        // Translations about 135.0
        {IMUUtils::DegreesToRadians(135.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), 0.0, 1.0, 1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, 0.0, -1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 0.0, -1.0, -1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), -1.0, 0.0, 1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), -1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), 1.0, -1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), -1.0, 1.0, std::sqrt(2.0)},
        // Translations about 180.0
        {IMUUtils::DegreesToRadians(180.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 0.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, 0.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), 0.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, 0.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, 1.0, 1.0},
        // Translations about 225.0
        {IMUUtils::DegreesToRadians(225.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), 0.0, 1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 0.0, -1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), -1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, 1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), -1.0, -1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), -1.0, 1.0, 0.0},
        // Translations about 270.0
        {IMUUtils::DegreesToRadians(270.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 0.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 0.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, 1.0, -1.0},
        // Translations about 315.0
        {IMUUtils::DegreesToRadians(315.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), 0.0, 1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 0.0, -1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), -1.0, 0.0, -1 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), -1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), 1.0, -1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), -1.0, 1.0, -std::sqrt(2.0)},

        // Translations beyond 0-360 bounds
        {IMUUtils::DegreesToRadians(405.0), 1.0, 1.0, std::sqrt(2.0)}};

    for (const auto &c : cases)
    {
        EXPECT_NEAR(InertialToGlobal_X(c.theta_input, c.x_accel_input, c.y_accel_input), c.x_accel_output, 1e-9);
    };
};
// ----------------------------------------------------------------------------
TEST(IMUUtils, Convert_Local_xy_Acceleration_To_Global_Y_Acceleration)
{
    struct Case
    {
        double theta_input;
        double x_accel_input;
        double y_accel_input;
        double y_accel_output;
    };

    const std::vector<Case> cases = {
        // Translations about 0.0
        {IMUUtils::DegreesToRadians(0.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 0.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 0.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), 1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(0.0), -1.0, 1.0, 1.0},
        // Translations about 45.0
        {IMUUtils::DegreesToRadians(45.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), 0.0, 1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 0.0, -1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), -1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), 1.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), -1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(45.0), 1.0, -1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(45.0), -1.0, 1.0, std::sqrt(2.0)},
        // Translations about 90.0
        {IMUUtils::DegreesToRadians(90.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 0.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, 0.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), 0.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, 0.0, 1.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(90.0), 1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(90.0), -1.0, 1.0, 1.0},
        // Translations about 135.0
        {IMUUtils::DegreesToRadians(135.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), 0.0, 1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 0.0, -1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), -1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, 1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), -1.0, -1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(135.0), 1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(135.0), -1.0, 1.0, 0.0},
        // Translations about 180.0
        {IMUUtils::DegreesToRadians(180.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 0.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 0.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, 1.0, -1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), 1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(180.0), -1.0, 1.0, -1.0},
        // Translations about 225.0
        {IMUUtils::DegreesToRadians(225.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), 0.0, 1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 0.0, -1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), -1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), 1.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), -1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(225.0), 1.0, -1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(225.0), -1.0, 1.0, -std::sqrt(2.0)},
        // Translations about 270.0
        {IMUUtils::DegreesToRadians(270.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 0.0, 1.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, 0.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), 0.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, 0.0, -1.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, 1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, -1.0, -1.0},
        {IMUUtils::DegreesToRadians(270.0), 1.0, -1.0, 1.0},
        {IMUUtils::DegreesToRadians(270.0), -1.0, 1.0, -1.0},
        // Translations about 315.0
        {IMUUtils::DegreesToRadians(315.0), 0.0, 0.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), 0.0, 1.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, 0.0, 1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 0.0, -1.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), -1.0, 0.0, -1.0 / std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, 1.0, std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), -1.0, -1.0, -std::sqrt(2.0)},
        {IMUUtils::DegreesToRadians(315.0), 1.0, -1.0, 0.0},
        {IMUUtils::DegreesToRadians(315.0), -1.0, 1.0, 0.0},

        // Translations beyond 0-360 bounds
        {IMUUtils::DegreesToRadians(405.0), 1.0, 1.0, 0.0}}; // cases

    for (const auto &c : cases)
    {
        EXPECT_NEAR(InertialToGlobal_Y(c.theta_input, c.x_accel_input, c.y_accel_input), c.y_accel_output, 1e-9);
    };
};

TEST(IMUUtils, MagneticToTrueHeadingExpectedValues) {
    struct TruthAngles {
        double lat;
        double lon;
        double declinationAngle;
        double magneticHeading;
        double trueNorthHeading;
    };

    std::vector<TruthAngles> expected = {
        {76, -112, 6.63, 52, 58.63},
        {76, 112, -7.05, 52, 44.95},
        {-76, 112, -129.6, 52, 282.4},
        {-76, -112, 55.4, 52, 107.4},

        {62, -40, -18.65, 271, 252.35},
        {62, 40, 16.4, 271, 287.4},
        {-62, 40, -51.08, 271, 219.92},
        {-62, -40, 0.14, 271, 271.14},

        {82, -170, -6.53, 1, 354.47},
        {82, 170, -16.3, 1, 344.7},
        {-82, 170, 146.58, 320, 106.58},
        {-82, -170, 122.19, 320, 82.19},

        {-65, -10, -13.89, 200, 186.11},
        {-55, -60, 6.09, 200, 206.09}, 
        {-45, 170, 25.48, 200, 225.48},
        {-55, 160, 35.79, 200, 235.79}, 
    };

    for (const auto& truthAngle : expected) {
        EXPECT_NEAR(MagneticToTrueHeading(truthAngle.magneticHeading, truthAngle.declinationAngle), truthAngle.trueNorthHeading, 1e-9);
    }

    // Expect method to throw exception for invalid input

    EXPECT_THROW(MagneticToTrueHeading(-10, 2), std::runtime_error);
    EXPECT_THROW(MagneticToTrueHeading(10, -200), std::runtime_error);
    EXPECT_THROW(MagneticToTrueHeading(360, 100), std::runtime_error);
    EXPECT_THROW(MagneticToTrueHeading(10, 200), std::runtime_error);
    EXPECT_THROW(MagneticToTrueHeading(0.0, 180), std::runtime_error);
    EXPECT_NO_THROW(MagneticToTrueHeading(0.0, -180));
}
