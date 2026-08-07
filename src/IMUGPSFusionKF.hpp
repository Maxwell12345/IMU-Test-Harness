/******************************************************************************
 * File:             IMUGPSFusionKF.hpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Declares the ENU extended Kalman filter used to fuse GPS
 *                   position with vehicle acceleration and yaw-rate controls.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_IMUGPSFUSIONKF_HPP
#define INU_DISPLAY_IMUGPSFUSIONKF_HPP

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

    /**
     *
     * @brief   Advances the ENU state using forward acceleration and yaw rate when no new GPS position is available.
     *
     * @param [in] dt          Elapsed measurement time in seconds derived from IMU payload timestamps.
     * @param [in] imuControl  Two-element control containing forward acceleration in meters per second squared and yaw rate
     *                         in radians per second.
     *
     * @return  Pair containing the updated six-element state and six-by-six covariance matrix.
     *
     * @throws  std::bad_alloc  If the adaptive covariance estimator cannot extend its sample window.
     */
    std::pair<Eigen::Matrix<double, N_STATE, 1>, Eigen::Matrix<double, N_STATE, N_STATE>> Step(
        double dt,
        Eigen::Matrix<double, 2, 1>& imuControl);

    /**
     *
     * @brief   Advances the ENU state using a new GPS easting/northing position together with forward acceleration and yaw
     *          rate from the IMU payloads.
     *
     * @param [in] dt              Elapsed measurement time in seconds derived from IMU payload timestamps.
     * @param [in] gpsMeasurement  Two-element GPS position containing easting and northing in meters.
     * @param [in] imuControl      Two-element control containing forward acceleration in meters per second squared and yaw
     *                             rate in radians per second.
     *
     * @return  Pair containing the updated six-element state and six-by-six covariance matrix.
     *
     * @throws  std::bad_alloc  If the adaptive covariance estimator cannot extend its sample window.
     */
    std::pair<Eigen::Matrix<double, N_STATE, 1>, Eigen::Matrix<double, N_STATE, N_STATE>> Step(
        double dt,
        Eigen::Matrix<double, 2, 1>& gpsMeasurement,
        Eigen::Matrix<double, 2, 1>& imuControl);

    /**
     *
     * @brief   Re-expresses the current position and heading after the controller changes the local ENU tangent origin.
     *
     * @param [in] easting       State easting in meters relative to the new origin.
     * @param [in] northing      State northing in meters relative to the new origin.
     * @param [in] oldLatitude   Previous origin latitude in decimal degrees.
     * @param [in] oldLongitude  Previous origin longitude in decimal degrees.
     * @param [in] newLatitude   New origin latitude in decimal degrees.
     * @param [in] newLongitude  New origin longitude in decimal degrees.
     *
     * @return
     */
    void UpdatePosition(double easting,
                        double northing,
                        double oldLatitude,
                        double oldLongitude,
                        double newLatitude,
                        double newLongitude);

    /**
     *
     * @brief   Builds the nonlinear ENU motion model Jacobian for the supplied state, control, and measurement interval.
     *
     * @param [in] dt          Elapsed measurement time in seconds.
     * @param [in] state       State ordered as easting, northing, heading, speed, yaw rate, and acceleration.
     * @param [in] imuControl  Forward acceleration and yaw rate control.
     *
     * @return  Six-by-six state transition Jacobian.
     */
    static Eigen::Matrix<double, N_STATE, N_STATE> BuildFk(double dt,
                                                            Eigen::Matrix<double, N_STATE, 1> state,
                                                            Eigen::Matrix<double, 2, 1> imuControl);

    /**
     *
     * @brief   Applies the nonlinear constant-turn-rate motion model to predict the next ENU state.
     *
     * @param [in] dt          Elapsed measurement time in seconds.
     * @param [in] state       State ordered as easting, northing, heading, speed, yaw rate, and acceleration.
     * @param [in] imuControl  Forward acceleration and yaw rate control.
     *
     * @return  Predicted six-element state.
     */
    static Eigen::Matrix<double, N_STATE, 1> f(double dt,
                                                Eigen::Matrix<double, N_STATE, 1> state,
                                                Eigen::Matrix<double, 2, 1> imuControl);

    /**
     *
     * @brief   Builds state-dependent process-noise covariance for longitudinal jerk and yaw acceleration over dt.
     *
     * @param [in] dt     Elapsed measurement time in seconds.
     * @param [in] state  State ordered as easting, northing, heading, speed, yaw rate, and acceleration.
     *
     * @return  Six-by-six process-noise covariance matrix.
     */
    static Eigen::Matrix<double, N_STATE, N_STATE> Update_Q(double dt,
                                                             Eigen::Matrix<double, N_STATE, 1> state);

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

#endif // INU_DISPLAY_IMUGPSFUSIONKF_HPP
