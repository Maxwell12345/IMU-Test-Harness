#include <gtest/gtest.h>
// #include "IMUGPSFusionKF.hpp"

// TEST(IMUGPSFusionKF_2D_ConstantAcceleration, BuildFkMatchesConstantAccelerationModel) {

//     Eigen::Matrix<double, 6, 1> x0 = Eigen::Matrix<double, 6, 1>::Zero();
//     Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();
//     Eigen::Matrix<double, 2, 2> I2 = Eigen::Matrix<double, 2, 2>::Identity();

//     IMUGPSFusionKF_2D_ConstantAcceleration kf(
//         x0, I6, I2, I2, I6, 0.0, 0.0, 1e6, 1e6, 5, 5, 5, 5, 5, 5
//     );

//     const double dt = 0.1;
//     const Eigen::Matrix<double, 6, 6> F = kf.BuildFk(dt);

//     Eigen::Matrix<double, 6, 6> expected;
//     const double h = 0.5 * dt * dt;
//     expected <<
//         1, 0, dt, 0, h, 0,
//         0, 1, 0, dt, 0, h,
//         0, 0, 1, 0, dt, 0,
//         0, 0, 0, 1, 0, dt,
//         0, 0, 0, 0, 1, 0,
//         0, 0, 0, 0, 0, 1;

//     EXPECT_TRUE(F.isApprox(expected, 1e-12));
// }