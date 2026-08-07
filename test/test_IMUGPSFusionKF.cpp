/******************************************************************************
 * File:             test_IMUGPSFusionKF.cpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Verifies ENU motion-model state transitions for positive
 *                   and negative vehicle yaw rates.
 *
 ******************************************************************************/

#include <cmath>

#include <gtest/gtest.h>

#include "IMUGPSFusionKF.hpp"
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

TEST(IMUGPSFusionKFTest, NegativeYawRateUsesTurningModel) {
    Eigen::Matrix<double, N_STATE, 1> state = Eigen::Matrix<double, N_STATE, 1>::Zero();
    state(3) = 10.0;

    Eigen::Matrix<double, 2, 1> imuControl;
    imuControl << 0.0, -0.5;

    const auto updatedState = IMUGPSFusionKF::f(1.0, state, imuControl);
    const double expectedEasting = (10.0 / -0.5) * std::sin(-0.5);
    const double expectedNorthing = (10.0 / -0.5) * (-std::cos(-0.5) + 1.0);

    EXPECT_NEAR(updatedState(0), expectedEasting, 1e-12);
    EXPECT_NEAR(updatedState(1), expectedNorthing, 1e-12);
    EXPECT_NEAR(updatedState(2), -0.5, 1e-12);
    EXPECT_NEAR(updatedState(3), 10.0, 1e-12);
}

TEST(IMUGPSFusionKFTest, ImuControlUpdatesAccelerationAndYawRateStateElements) {
    const Eigen::Matrix<double, N_STATE, 1> initialState =
        Eigen::Matrix<double, N_STATE, 1>::Zero();
    const Eigen::Matrix<double, N_STATE, N_STATE> initialCovariance =
        Eigen::Matrix<double, N_STATE, N_STATE>::Identity();
    const Eigen::Matrix<double, M_Z, M_Z> measurementCovariance =
        0.01 * Eigen::Matrix<double, M_Z, M_Z>::Identity();
    IMUGPSFusionKF filter(initialState, initialCovariance, measurementCovariance);

    Eigen::Matrix<double, 2, 1> imuControl;
    imuControl << 2.0, 0.5;

    const auto [updatedState, updatedCovariance] = filter.Step(0.01, imuControl);

    EXPECT_LT(std::abs(updatedState(4) - 0.5), std::abs(updatedState(4) - 2.0));
    EXPECT_LT(std::abs(updatedState(5) - 2.0), std::abs(updatedState(5) - 0.5));
    EXPECT_TRUE(updatedState.allFinite());
    EXPECT_TRUE(updatedCovariance.allFinite());
}
