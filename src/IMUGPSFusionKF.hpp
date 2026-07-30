#ifndef IMU_GPS_FUSION_KF_HPP
#define IMU_GPS_FUSION_KF_HPP

#include <iostream>
#include <deque>
#include "IMUCovarianceEstimator.hpp"

#define N_STATE 6
#define M_Z 4

class IMUGPSFusionKF {
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUGPSFusionKF() = default;

    IMUGPSFusionKF(
        Eigen::Matrix<double, N_STATE, 1> x0, // initial state vector
        Eigen::Matrix<double, N_STATE, N_STATE> P0, // initial error covariance matrix
        Eigen::Matrix<double, M_Z, M_Z> R0 // initial measurement noise covariance
    );

    std::pair<Eigen::Matrix<double, N_STATE, 1>, Eigen::Matrix<double, N_STATE, N_STATE>> Step(double dt, Eigen::Matrix<double, 2, 1>& z_IMU);

    std::pair<Eigen::Matrix<double, N_STATE, 1>, Eigen::Matrix<double, N_STATE, N_STATE>> Step(double dt, Eigen::Matrix<double, 2, 1>& z_GPS, Eigen::Matrix<double, 2, 1>& z_IMU);

    void UpdatePosition(double E, double N, double oldLat, double oldLon, double newLat, double newLon);

    static inline Eigen::Matrix<double, N_STATE, N_STATE> BuildFk(double dt, Eigen::Matrix<double, N_STATE, 1> x, Eigen::Matrix<double, 2, 1> u);

    static inline Eigen::Matrix<double, N_STATE, 1> f(double dt, Eigen::Matrix<double, N_STATE, 1> x, Eigen::Matrix<double, 2, 1> u);

    static inline Eigen::Matrix<double, N_STATE, N_STATE> Update_Q(double dt, Eigen::Matrix<double, N_STATE, 1> x);

    Eigen::Matrix<double, M_Z, M_Z> m_R;
    Eigen::Matrix<double, N_STATE, N_STATE> m_Q;

private:

private:
    // Fusion filter members
    Eigen::Matrix<double, N_STATE, 1> m_x; 
    Eigen::Matrix<double, N_STATE, N_STATE> m_P; 
    Eigen::Matrix<double, M_Z, N_STATE> m_H_GPS_IMU;
    Eigen::Matrix<double, M_Z, N_STATE> m_H_IMU;

    Eigen::Matrix<double, N_STATE, 1> m_lastGPSx; 

    double m_lastGPSDt;

    IMUCovarianceEstimator m_covarianceEstimator;
};

#endif