#include <gtest/gtest.h>
#include "IMUGPSFusionKF.hpp"

TEST(IMUGPSFusionKF_2D_ConstantAcceleration, BuildFkMatchesConstantAccelerationModel) {
    using Mat6 = Eigen::Matrix<double, 6, 6>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;

    Vec6 x0 = Vec6::Zero();
    Mat6 I6 = Mat6::Identity();

    IMUGPSFusionKF_2D_ConstantAcceleration kf(
        x0, I6, I6, I6, I6, 0.0, 0.0, 1e6, 1e6, 5, 5, 5, 5, 5, 5
    );

    const double dt = 0.1;
    const Mat6 F = kf.BuildFk(dt);

    Mat6 expected;
    const double h = 0.5 * dt * dt;
    expected <<
        1, 0, dt, 0, h, 0,
        0, 1, 0, dt, 0, h,
        0, 0, 1, 0, dt, 0,
        0, 0, 0, 1, 0, dt,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    EXPECT_TRUE(F.isApprox(expected, 1e-12));
}