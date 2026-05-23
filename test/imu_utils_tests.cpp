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
