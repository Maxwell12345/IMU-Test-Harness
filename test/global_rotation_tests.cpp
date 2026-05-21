//
// Created by user on 5/20/26.
//


#include <gtest/gtest.h>
#include <cmath>
#include <format>

namespace {
    struct RelationMapping {
        double bx_Multiplier;
        double by_Multiplier;

        [[nodiscard]] std::string ToString() const {
            char sign = by_Multiplier >= 0.0 ? '+' : '-';
            return std::format("{:.2f} * b_x {} {:.2f} * b_y",
                               bx_Multiplier, sign, std::abs(by_Multiplier));
        }
    };

    double CalculateGlobalX(double theta_t, double y_r, double x_r) {
        // x_g = y_r * sin(theta_t) + x_r * cos(theta_t)
        double x_g = y_r * std::sin(theta_t) + x_r * std::cos(theta_t);
        return x_g;
    }

    double CalculateGlobalY(double theta_t, double y_r, double x_r) {
        // y_g = y_r * cos(theta_t) - x_r * sin(theta_t)
        double y_g = y_r * std::cos(theta_t) - x_r * std::sin(theta_t);
        return y_g;
    }

    double DegreesToRadians(double degrees) {
        return degrees * std::numbers::pi / 180;
    }

    void RunOrientationTest(double degreesTrue, RelationMapping& globalXMap, RelationMapping& globalYMap) {
        SCOPED_TRACE(std::format("heading={:.1f}°  global_X_map = {}  |  global_Y_map = {}",
                                 degreesTrue, globalXMap.ToString(), globalYMap.ToString()));
        double theta = DegreesToRadians(degreesTrue);

        auto mapFunc = [](RelationMapping& m, double boatX, double boatY) {
            return m.bx_Multiplier * boatX + m.by_Multiplier * boatY;
        };

        double x_r, y_r;

        // boat stays still
        {
            x_r = 0.0; y_r = 0.0;
            SCOPED_TRACE(std::format("boat stays still: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }

        // boat moves forward
        {
            x_r = 0.0; y_r = 1.0;
            SCOPED_TRACE(std::format("boat moves forward: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }

        // boat moves diagonally forward right
        {
            x_r = 1.0; y_r = 1.0;
            SCOPED_TRACE(std::format("boat moves diagonally forward right: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }

        // boat moves right
        {
            x_r = 1.0; y_r = 0.0;
            SCOPED_TRACE(std::format("boat moves right: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }

        // boat moves diagonally backward right
        {
            x_r = 1.0; y_r = -1.0;
            SCOPED_TRACE(std::format("boat moves diagonally backward right: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }

        // boat moves backward
        {
            x_r = 0.0; y_r = -1.0;
            SCOPED_TRACE(std::format("boat moves backward: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }

        // boat moves diagonally backward left
        {
            x_r = -1.0; y_r = -1.0;
            SCOPED_TRACE(std::format("boat moves diagonally backward left: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }

        // boat moves left
        {
            x_r = -1.0; y_r = 0.0;
            SCOPED_TRACE(std::format("boat moves left: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }

        // boat moves diagonally forward left
        {
            x_r = -1.0; y_r = 1.0;
            SCOPED_TRACE(std::format("boat moves diagonally forward left: boat_x={}, boat_y={}", x_r, y_r));
            EXPECT_NEAR(CalculateGlobalX(theta, y_r, x_r), mapFunc(globalXMap, x_r, y_r), 1e-9) << "global_x";
            EXPECT_NEAR(CalculateGlobalY(theta, y_r, x_r), mapFunc(globalYMap, x_r, y_r), 1e-9) << "global_y";
        }
    }
}

TEST(Inertial_To_Global_Rotation, Oriented_North) {
    double degrees = 0.0;
    RelationMapping xMap {1.0, 0.0};
    RelationMapping yMap {0.0, 1.0};
    RunOrientationTest(degrees, xMap, yMap);
}

TEST(Inertial_To_Global_Rotation, Oriented_East) {
    double degrees = 90.0;
    RelationMapping xMap {0.0, 1.0};
    RelationMapping yMap {-1.0, 0.0};
    RunOrientationTest(degrees, xMap, yMap);
}

TEST(Inertial_To_Global_Rotation, Oriented_South) {
    double degrees = 180.0;
    RelationMapping xMap {-1.0, 0.0};
    RelationMapping yMap {0.0, -1.0};
    RunOrientationTest(degrees, xMap, yMap);
}

TEST(Inertial_To_Global_Rotation, Oriented_West) {
    double degrees = 270.0;
    RelationMapping xMap {0.0, -1.0};
    RelationMapping yMap {1.0, 0.0};
    RunOrientationTest(degrees, xMap, yMap);
}

TEST(Inertial_To_Global_Rotation, Oriented_Northeast) {
    double degrees = 45.0;
    double theta = DegreesToRadians(degrees);
    RelationMapping xMap {std::cos(theta), std::sin(theta)};
    RelationMapping yMap {-1.0 * std::sin(theta), std::cos(theta)};
    RunOrientationTest(degrees, xMap, yMap);
}

TEST(Inertial_To_Global_Rotation, Oriented_Southeast) {
    double degrees = 135.0;
    double theta = DegreesToRadians(degrees);
    RelationMapping xMap {std::cos(theta), std::sin(theta)};
    RelationMapping yMap {std::cos(theta), -1.0 * std::sin(theta)};
    RunOrientationTest(degrees, xMap, yMap);
}

TEST(Inertial_To_Global_Rotation, Oriented_Southwest) {
    double degrees = 225.0;
    double theta = DegreesToRadians(degrees);
    RelationMapping xMap {std::cos(theta), std::sin(theta)};
    RelationMapping yMap {-1.0 * std::sin(theta), std::cos(theta)};
    RunOrientationTest(degrees, xMap, yMap);
}

TEST(Inertial_To_Global_Rotation, Oriented_Northwest) {
    double degrees = 315.0;
    double theta = DegreesToRadians(degrees);
    RelationMapping xMap {std::cos(theta), std::sin(theta)};
    RelationMapping yMap {-1.0 * std::sin(theta), std::cos(theta)};
    RunOrientationTest(degrees, xMap, yMap);
}
