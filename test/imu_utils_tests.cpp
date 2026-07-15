/******************************************************************************
 * Filename:     imu_utils_tests.cpp
 *
 * Author:       Tran Sgt Brandon, Gomez LCpl Greg
 * Organization: Marine Corps Software Factory
 * Created On:   5/21/2026 2:45 PM
 * Description:  This test files validates (1) DegreesToRadians, (2)
 * InertialToGlobal_X and (3) InertialToGlobal_Y functionality.
 *
 ******************************************************************************/

#include "utils.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <numbers>

using namespace IMUUtils;

#ifndef M_PI
#define M_PI std::numbers::pi
#endif

TEST(IMUUtils, Convert_Degrees_To_Radians) {

  struct Case {
    double degrees_input;
    double degrees_expected;
  };

  const std::vector<Case> cases = {// These tests are pass correct
                                    {-45.0, std::numbers::pi * 7 / 4},
                                    {0.0, std::numbers::pi * 0},
                                    {45.0, std::numbers::pi / 4},
                                    {90.0, std::numbers::pi / 2},
                                    {135.0, std::numbers::pi * 3 / 4},
                                    {180.0, std::numbers::pi},
                                    {225.0, std::numbers::pi * 5 / 4},
                                    {270.0, std::numbers::pi * 3 / 2},
                                    {315.0, std::numbers::pi * 7 / 4},
                                    {360.0, 0.0},
                                    {405.0, std::numbers::pi / 4}
  };

  for (const auto &c : cases) {
    EXPECT_NEAR(IMUUtils::DegreesToRadians(c.degrees_input), c.degrees_expected, 1e-9);
  };
};
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
// -- -- -- -- -- -- -- -- -- -- -- -- -
TEST(IMUUtils, Convert_Local_XY_Acceleration_To_Global_X_Acceleration) {
  struct Case {
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
      {IMUUtils::DegreesToRadians(405.0), 1.0, 1.0, std::sqrt(2.0)}
  };

  for (const auto &c : cases) {
    EXPECT_NEAR(InertialToGlobal_X(c.theta_input, c.x_accel_input, c.y_accel_input), c.x_accel_output, 1e-9);
  };
};
// ----------------------------------------------------------------------------
TEST(IMUUtils, Convert_Local_xy_Acceleration_To_Global_Y_Acceleration) {
  struct Case {
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
      {IMUUtils::DegreesToRadians(405.0), 1.0, 1.0, 0.0}
  }; // cases

  for (const auto &c : cases) {
    EXPECT_NEAR(InertialToGlobal_Y(c.theta_input, c.x_accel_input, c.y_accel_input), c.y_accel_output, 1e-9);
  };
};
// ----------------------------------------------------------------------------
TEST(IMUUtils, Convert_Global_X_Accel_Into_Degrees_Longitude) {
  struct Case {
    double degree_latitude_input;
    double accel_x_input;
    double accel_deg_longitude_output;
  };

  const std::vector<Case> cases = {
      // Near equator at +1 m/s2
      {0.0, 1.0, 1.0 / (111111.11 * std::cos(0.0 * std::numbers::pi / 180.0))},
      // 45 deg latitude at +1 m/s2
      {45.0, 1.0, 1.0 / (111111.11 * std::cos(45.0 * std::numbers::pi / 180.0))},
      // 60 deg latitude at +1 m/s2
      {60.0, 1.0, 1.0 / (111111.11 * std::cos(60.0 * std::numbers::pi / 180.0))},
      // Near equator at -1 m/s2
      {0.0, -1.0, -1.0 / (111111.11 * std::cos(0.0 * std::numbers::pi / 180.0))},
      // 45 deg latitude at -1 m/s2
      {45.0, -1.0, -1.0 / (111111.11 * std::cos(45.0 * std::numbers::pi / 180.0))},
      // 60 deg latitude at -1 m/s2
      {60.0, -1.0, -1.0 / (111111.11 * std::cos(60.0 * std::numbers::pi / 180.0))}
  };

  for (const auto &c : cases) {
    EXPECT_NEAR(
        IMUUtils::Convert_Global_X_to_DegPerS2(c.degree_latitude_input, c.accel_x_input),
        c.accel_deg_longitude_output, 1e-9
    );
  };
};

TEST(IMUUtils, Calculating_Magnetic_Heading_From_Rotation_Vectors) {
  struct Case {
    double w_input;
    double i_input;
    double j_input;
    double k_input;
    double magnetic_heading_output;
  };

  const std::vector<Case> cases {
    {0.703796, 0.032959, -0.061829, -0.706909, 269.593608},
    {0.821960, 0.038635, -0.052124, -0.565857, 290.761891},
    {0.919800, 0.020264, -0.003479, -0.391846, 313.861807},
    {0.971741, 0.043945, 0.005249, -0.231873, 333.231346},
    {0.988098, 0.045898, -0.030945, -0.143311, 343.358572},
    {0.994141, 0.066956, 0.005554, -0.084900, 350.322444},
    {0.993530, 0.028931, 0.059387, 0.092651, 10.879328},
    {0.984985, 0.027466, 0.052551, 0.161926, 18.864364},
    {0.967468, 0.047852, 0.013611, 0.247986, 28.760084},
    {0.925964, -0.011536, 0.037781, 0.375549, 44.168813},
    {0.888550, -0.016907, 0.033936, 0.457214, 54.459598},
    {0.873535, -0.032959, -0.015015, 0.485413, 58.108868},
    {0.855530, -0.011414, 0.014709, 0.517395, 62.322400},
    {0.759521, -0.021912, 0.012573, 0.650024, 81.095378},
    {0.710815, -0.068420, -0.024597, 0.699646, 88.864519},
    {0.561279, -0.008545, 0.029053, 0.827026, 111.720331},
    {0.416138, -0.013733, 0.022156, 0.908936, 130.837101},
    {0.293030, 0.000122, 0.048401, 0.954895, 145.956349},
    {0.179443, 0.000977, 0.070435, 0.981262, 159.367645},
    {0.003174, 0.005005, -0.064453, -0.997925, 180.399891},
    {0.126343, 0.019043, -0.089600, -0.987732, 194.657630},
    {0.312500, 0.018921, -0.066467, -0.947388, 216.488994},
    {0.347595, 0.014954, -0.076599, -0.934387, 220.698976},
    {0.411987, 0.023132, -0.064087, -0.908630, 228.739217},
    {0.435303, 0.006226, -0.084595, -0.896301, 231.522990},
    {0.578125, 0.008057, -0.035156, -0.815186, 250.632424},
    {0.679871, 0.050903, -0.031006, -0.730957, 265.954413},
    {0.736328, 0.056946, -0.088562, -0.668396, 275.217673},
    {0.030884, 0.213745, -0.180786, -0.959534, 188.563874},
    {0.750427, -0.429993, 0.121277, 0.487061, 51.633530},
    {0.960205, -0.225525, 0.037781, -0.160461, 341.022779},
    {0.860779, -0.093201, 0.083801, -0.493286, 299.998807},
    {0.836914, -0.107727, 0.034851, -0.535461, 295.140112},
    {0.852295, -0.101379, 0.033203, -0.512024, 298.294129},
  };

  for (const auto &c : cases) {
    EXPECT_NEAR(IMUUtils::Calculate_Magnetic_Heading(c.w_input, c.i_input, c.j_input, c.k_input), c.magnetic_heading_output, 1e-3);
  };
};
// ----------------------------------------------------------------------------
TEST(IMUUtils, Calculating_Magnetic_Heading_From_Rotation_Vectors_Throws_Exception) {
  EXPECT_THROW(IMUUtils::Calculate_Magnetic_Heading(10.0, 1.0, 1.0, 1.0), std::out_of_range);
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

TEST(IMUUtils, KineticUpdates) {
    const auto initialTimePoint = std::chrono::steady_clock::time_point{};
    const auto initialState = IMUUtils::KineticState(initialTimePoint, 0.0, 0.0, 0.0, 0.0);

    const auto secondTimePoint = initialTimePoint + std::chrono::milliseconds(1);
    IMUUtils::KineticState secondState = IMUUtils::CalculateKineticUpdate(initialState, 1.0, 1.0, secondTimePoint);
    EXPECT_EQ(secondState.timestamp, secondTimePoint);
    EXPECT_NEAR(secondState.accelerationEastWest, 1.0, 1e-12);
    EXPECT_NEAR(secondState.accelerationNorthSouth, 1.0, 1e-12);
    EXPECT_NEAR(secondState.speedEastWest, .001, 1e-12);
    EXPECT_NEAR(secondState.speedNorthSouth, .001, 1e-12);

    const auto thirdTimePoint = secondTimePoint + std::chrono::milliseconds(1);
    IMUUtils::KineticState thirdState = IMUUtils::CalculateKineticUpdate(secondState, 2.0, -1.0, thirdTimePoint);
    EXPECT_EQ(thirdState.timestamp, thirdTimePoint);
    EXPECT_NEAR(thirdState.accelerationEastWest, 2.0, 1e-12);
    EXPECT_NEAR(thirdState.accelerationNorthSouth, -1.0, 1e-12);
    EXPECT_NEAR(thirdState.speedEastWest, .003, 1e-12);
    EXPECT_NEAR(thirdState.speedNorthSouth, 0.0, 1e-12);
}

TEST(IMUUtils, WGS84_to_UTMTests) {
    {
        auto point1 = IMUUtils::WGS84_to_UTM(30.272884413486207,-97.74489895728875);
        auto point2 = IMUUtils::WGS84_to_UTM(30.272885413486208,-97.74489995728875);
        double mercator_distance = std::hypot(point2[0] - point1[0],point2[1] - point1[1]);
        EXPECT_NEAR(mercator_distance, 0.14675938895142, 0.0000001);
    }

    {
        auto point1 = IMUUtils::WGS84_to_UTM(20.736223782144965,121.86724791604726);
        auto point2 = IMUUtils::WGS84_to_UTM(20.736123782144965,121.86734791604727);
        double mercator_distance = std::hypot(point2[0] - point1[0],point2[1] - point1[1]);
        EXPECT_NEAR(mercator_distance, 15.196908912050178, 0.00001);
    }

    {
        auto point1 = IMUUtils::WGS84_to_UTM(-39.223277923963025,143.17886116659233);
        auto point2 = IMUUtils::WGS84_to_UTM(-39.213277923963027,143.16886116659234);
        double mercator_distance = std::hypot(point2[0] - point1[0],point2[1] - point1[1]);
        EXPECT_NEAR(mercator_distance, 1406.5801614702168, 1.0);
    }

    {
        auto point1 = IMUUtils::WGS84_to_UTM(63.133702046071825,-165.3609490928344);
        auto point2 = IMUUtils::WGS84_to_UTM(63.033702046071824,-165.26094909283441);
        double mercator_distance = std::hypot(point2[0] - point1[0],point2[1] - point1[1]);
        EXPECT_NEAR(mercator_distance, 12233.212012160137, 10.0);
    }

    {
        auto point1 = IMUUtils::WGS84_to_UTM(-62.684373621915974,-64.52170570858675);
        auto point2 = IMUUtils::WGS84_to_UTM(-61.684373621915974,-65.52170570858675);
        double mercator_distance = std::hypot(point2[0] - point1[0],point2[1] - point1[1]);
        EXPECT_NEAR(mercator_distance, 122981.73833530223, 10.0);
    }

    {
        auto point1 = IMUUtils::WGS84_to_UTM(29.940582911429292,48.77482933117357);
        auto point2 = IMUUtils::WGS84_to_UTM(34.940582911429289,53.77482933117357);
        double mercator_distance = std::hypot(point2[0] - point1[0], point2[1] - point1[1]);
        EXPECT_NEAR(mercator_distance, 726630.5340157725, 10.0);
    }
}

TEST(IMUUtils, UTM_to_WGS84Tests) {
    {
        auto point = UTM_to_WGS84(355263.84370631055, 2769368.6305411784, 51., 0.0);
        EXPECT_NEAR(point[0], 25.033000, 0.00001);
        EXPECT_NEAR(point[1], 121.565400, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(324182.2094294755, 5591607.607408224, 36., 0.0);
        EXPECT_NEAR(point[0], 50.450100, 0.00001);
        EXPECT_NEAR(point[1], 30.523400, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(484902.6214104738, 3619781.607950021, 11., 0.0);
        EXPECT_NEAR(point[0], 32.715700, 0.00001);
        EXPECT_NEAR(point[1], -117.161100, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(600105.5994494675, 3627015.7043538853, 17., 0.0);
        EXPECT_NEAR(point[0], 32.776500, 0.00001);
        EXPECT_NEAR(point[1], -79.931100, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(465609.1687300466, 9317795.753332414, 31., 0.0);
        EXPECT_NEAR(point[0], 83.900000, 0.00001);
        EXPECT_NEAR(point[1], 0.100000, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(443247.87180748064, 1128161.3728647754, 31., 10000000.0);
        EXPECT_NEAR(point[0], -79.900000, 0.00001);
        EXPECT_NEAR(point[1], 0.100000, 0.00001);
    }
}

TEST(IMUUtils, UTM_to_WGS84_to_UTM_and_vice_versa_tests) {
    {
        auto utm = IMUUtils::WGS84_to_UTM(0.000001, 0.000001);
        auto wgs84 = IMUUtils::UTM_to_WGS84(utm[0], utm[1], utm[2], utm[3]);
        EXPECT_NEAR(wgs84[0], 0.000001, 0.00001);
        EXPECT_NEAR(wgs84[1], 0.000001, 0.00001);
    }

    {
        auto utm = IMUUtils::WGS84_to_UTM(83.999000, 5.999000);
        auto wgs84 = IMUUtils::UTM_to_WGS84(utm[0], utm[1], utm[2], utm[3]);
        EXPECT_NEAR(wgs84[0], 83.999000, 0.00001);
        EXPECT_NEAR(wgs84[1], 5.999000, 0.00001);
    }

    {
        auto utm = IMUUtils::WGS84_to_UTM(-79.999000, -177.000100);
        auto wgs84 = IMUUtils::UTM_to_WGS84(utm[0], utm[1], utm[2], utm[3]);
        EXPECT_NEAR(wgs84[0], -79.999000, 0.00001);
        EXPECT_NEAR(wgs84[1], -177.000100, 0.00001);
    }

    {
        auto utm = IMUUtils::WGS84_to_UTM(0.000001, 179.999000);
        auto wgs84 = IMUUtils::UTM_to_WGS84(utm[0], utm[1], utm[2], utm[3]);
        EXPECT_NEAR(wgs84[0], 0.000001, 0.00001);
        EXPECT_NEAR(wgs84[1], 179.999000, 0.00001);
    }

    {
        auto utm = IMUUtils::WGS84_to_UTM(-0.000001, -179.999000);
        auto wgs84 = IMUUtils::UTM_to_WGS84(utm[0], utm[1], utm[2], utm[3]);
        EXPECT_NEAR(wgs84[0], -0.000001, 0.00001);
        EXPECT_NEAR(wgs84[1], -179.999000, 0.00001);
    }

    {
        auto wgs84 = IMUUtils::UTM_to_WGS84(166021.44317933184, 0.0, 31., 0.0);
        auto utm = IMUUtils::WGS84_to_UTM(wgs84[0], wgs84[1]);
        EXPECT_NEAR(utm[0], 166021.44317933184, 0.01);
        EXPECT_NEAR(utm[1], 0.0, 0.01);
        EXPECT_NEAR(utm[2], 31., 0.0);
        EXPECT_NEAR(utm[3], 0.0, 0.0);
    }

    {
        auto wgs84 = IMUUtils::UTM_to_WGS84(833978.5568206682, 0.0, 30., 0.0);
        auto utm = IMUUtils::WGS84_to_UTM(wgs84[0], wgs84[1]);
        EXPECT_NEAR(utm[0], 833978.5568206682, 0.01);
        EXPECT_NEAR(utm[1], 0.0, 0.01);
        EXPECT_NEAR(utm[2], 30., 0.0);
        EXPECT_NEAR(utm[3], 0.0, 0.0);
    }

    {
        auto wgs84 = IMUUtils::UTM_to_WGS84(500000.0, 9328093.83056051, 31., 0.0);
        auto utm = IMUUtils::WGS84_to_UTM(wgs84[0], wgs84[1]);
        EXPECT_NEAR(utm[0], 500000.0, 0.01);
        EXPECT_NEAR(utm[1], 9328093.83056051, 0.01);
        EXPECT_NEAR(utm[2], 31., 0.0);
        EXPECT_NEAR(utm[3], 0.0, 0.0);
    }

    {
        auto wgs84 = IMUUtils::UTM_to_WGS84(500000.0, 1116915.0440516975, 31., 10000000.0);
        auto utm = IMUUtils::WGS84_to_UTM(wgs84[0], wgs84[1]);
        EXPECT_NEAR(utm[0], 500000.0, 0.01);
        EXPECT_NEAR(utm[1], 1116915.0440516975, 0.01);
        EXPECT_NEAR(utm[2], 31., 0.0);
        EXPECT_NEAR(utm[3], 10000000.0, 0.0);
    }

    {
        auto wgs84 = IMUUtils::UTM_to_WGS84(500000.0, 9999999.0, 60., 10000000.0);
        auto utm = IMUUtils::WGS84_to_UTM(wgs84[0], wgs84[1]);
        EXPECT_NEAR(utm[0], 500000.0, 0.01);
        EXPECT_NEAR(utm[1], 9999999.0, 0.01);
        EXPECT_NEAR(utm[2], 60., 0.0);
        EXPECT_NEAR(utm[3], 10000000.0, 0.0);
    }
}

TEST(IMUUtils, UTM_to_WGS84OutsideNormalBoundsTests) {
    {
        auto point = UTM_to_WGS84(1079218.653553612, 3333984.371390104, 14., 0.0);

        EXPECT_NEAR(point[0], 30.000000, 0.00001);
        EXPECT_NEAR(point[1], -93.000000, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(1051706.264848742, 5006840.966858340, 14., 0.0);

        EXPECT_NEAR(point[0], 45.000000, 0.00001);
        EXPECT_NEAR(point[1], -92.000000, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(-79218.653553612, 3333984.371390104, 14., 0.0);

        EXPECT_NEAR(point[0], 30.000000, 0.00001);
        EXPECT_NEAR(point[1], -105.000000, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(-51706.264848742, 5006840.966858340, 14., 0.0);

        EXPECT_NEAR(point[0], 45.000000, 0.00001);
        EXPECT_NEAR(point[1], -106.000000, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(500000.000000000, 9551375.062623605, 31., 0.0);

        EXPECT_NEAR(point[0], 86.000000, 0.00001);
        EXPECT_NEAR(point[1], 3.000000, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(500000.000000000, 9663020.129747849, 32., 0.0);

        EXPECT_NEAR(point[0], 87.000000, 0.00001);
        EXPECT_NEAR(point[1], 9.000000, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(500000.000000000, 895171.028425648, 31., 10000000.0);

        EXPECT_NEAR(point[0], -82.000000, 0.00001);
        EXPECT_NEAR(point[1], 3.000000, 0.00001);
    }

    {
        auto point = UTM_to_WGS84(500000.000000000, 783540.980941838, 32., 10000000.0);

        EXPECT_NEAR(point[0], -83.000000, 0.00001);
        EXPECT_NEAR(point[1], 9.000000, 0.00001);
    }
}
