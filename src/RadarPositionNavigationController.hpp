#ifndef RADAR_POSITION_NAVIGATION_CONTROLLER_HPP
#define RADAR_POSITION_NAVIGATION_CONTROLLER_HPP

#include <atomic>
#include <iostream>
#include <Eigen/Dense>
#include <functional>
#include <mutex>
#include <gtest/gtest_prod.h>

#include "IMUManager.hpp"
#include "GpsUpdate.hpp"
#include "IMUGPSFusionKF.hpp"
#include "bno085_hal.hpp"  // brings in sh2.h and sh2_err.h under extern "C"

extern "C" {
#include "sh2_SensorValue.h"
}

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class RadarPositionNavigationController {
public:
    RadarPositionNavigationController();

    ~RadarPositionNavigationController();

    /**
     * @brief Provides a callback for a GPS service to receive asyncronous data.
     * 
     * @return
     * 
     * @remarks Intended use is in an asynchronous GPS service.
     * 
     * @exception
     */
    std::function<void(const GpsUpdate&)> GetGPSCallback();

    /**
     * @brief Starts the IMU sh2 listener and begins self tracking process. We assume all order 
     *        position derivatives are 0.
     * 
     * @param [in] lat0 the initial starting latitude position of the radar.
     * @param [in] lon0 the initial starting longitude position of the radar.
     * 
     * @return
     * 
     * @remarks
     * 
     * @exception
     */
    void StartAndConfigureRadarPNT(double lat0, double lon0);

    /**
     * @brief Stops the IMU sh2 listener and does NOT self tracking process. 
     * 
     * @return
     * 
     * @remarks This logically kills the KF posteriori update mechanism. The state 
     *          of the KF will remain available for predictions.
     * 
     * @exception std::runtime_error requires positive non zero df values and percentiles.
     * @exception std::exception KF initialization error.
     */
    void StopRadarPNT();

    /**
     * @brief Stops the IMU sh2 listener and DOES self tracking process. 
     * 
     * @return
     * 
     * @remarks This logically kills all tracking and destroys KF memory entirely.
     * 
     * @exception
     */
    void TotalDestruction();

private:
    /**
     * @brief Starts the sh2 listener service. 
     * 
     * @return
     * 
     * @remarks This service runs an indefinite while loop dependent on m_sh2ServiceIsRunning.
     * 
     * @exception
     */
    void StartIMUReader();

    /**
     * @brief Sets initial conditions in KF.
     * 
     * @param [in] lat0 the initial starting latitude position of the radar.
     * @param [in] lon0 the initial starting longitude position of the radar.
     * @param [in] gpsLowerPercentile Fuzzy algo GPS Chi SQ lower bound (0.0, 1.0).
     * @param [in] gpsUpperPercentile Fuzzy algo GPS Chi SQ upper bound (0.0, 1.0).
     * @param [in] imuLowerPercentile Fuzzy algo IMU Chi SQ lower bound (0.0, 1.0).
     * @param [in] imuUpperPercentile Fuzzy algo IMU Chi SQ upper bound (0.0, 1.0).
     * 
     * @return
     * 
     * @remarks Requires chi sq percentiles for fuzzy fusion algorithm.
     * 
     * @exception std::runtime_error requires positive non zero df values and percentiles.
     * @exception std::exception KF initialization error.
     */
    void ConfigureKalmanFilter(double lat0, double lon0, double gpsLowerPercentile, double gpsUpperPercentile, double imuLowerPercentile, double imuUpperPercentile);

    /**
     * @brief Execute KF with IMU measurement only.
     * 
     * @param [in] dt the time delta from the last step to the current time (measurement time).
     * @param [in] imuVec the IMU measurement vector. This is a column vector of 6 items [null, null, vlon, vlat, alon, alat]^T
     * 
     * @return
     * 
     * @remarks Run single shot fusion EKF algorithm with null GPS for deadreckoning without GPS.
     * 
     * @exception std::runtime_error requires positive non zero df values and percentiles.
     * @exception std::exception KF initialization or Step error.
     */
    void KFCallbackImuOnly(double dt, Vector6d& imuVec);

    /**
     * @brief Execute KF with IMU and GPS measurement.
     * 
     * @param [in] dt the time delta from the last step to the current time (measurement time).
     * @param [in] imuVec the IMU measurement vector. This is a column vector of 6 items [null, null, vlon, vlat, alon, alat]^T
     * @param [in] gpsVec the IMU measurement vector. This is a column vector of 6 items [lon, lat, null, null, null, null]^T
     * 
     * @return
     * 
     * @remarks Run single shot fusion EKF algorithm with GPS measurement to adjust and fuse position kinematics.
     * 
     * @exception std::runtime_error requires positive non zero df values and percentiles.
     * @exception std::exception KF initialization or Step error.
     */
    void KFCallbackWithGps(double dt, Vector6d& imuVec, Vector6d& gpsVec);

    /**
     * @brief GPS callback service in order to set IMUManager GPS state.
     * 
     * @return
     * 
     * @remarks 
     * 
     * @exception
     */
    void _GPSCallback (const GpsUpdate& gpsUpdate);

private:
    std::atomic<bool> m_sh2ServiceIsRunning;
    std::atomic<bool> m_isKFConfigured;
    std::thread m_serviceThread;
    sh2_Hal_t m_hal{};
    std::atomic<bool> m_sh2IsOpen{false};

    std::mutex m_sKFUpdateMutex;
    IMUGPSFusionKF_2D_ConstantAcceleration m_kf;
    Vector6d m_latestX;
    Matrix6d m_latestP;

    FRIEND_TEST(RadarPositionNavigationControllerTest, GetGPSCallbackUpdatesLatestGps);
    FRIEND_TEST(RadarPositionNavigationControllerTest, StartAndConfigureRadarPNTConfiguresKFAndStartsReader);
    FRIEND_TEST(RadarPositionNavigationControllerTest, StartAndConfigureRadarPNTDoesNotStartReaderWhenOpenFails);
    FRIEND_TEST(RadarPositionNavigationControllerTest, StopRadarPNTStopsThreadAndClosesSh2Once);
    FRIEND_TEST(RadarPositionNavigationControllerTest, TotalDestructionStopsReaderCleansKFAndZerosLatestState);
    FRIEND_TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterSetsInitialStateAndCovariance);
    FRIEND_TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsInvalidPercentiles);
    FRIEND_TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsLowerPercentileGreaterThanUpperPercentile);
    FRIEND_TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyReturnsWithoutConfiguredKF);
    FRIEND_TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsReturnsWithoutConfiguredKF);
    FRIEND_TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyProducesNonFiniteStateBecauseKFUsesSingularR);
    FRIEND_TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsProducesNonFiniteStateBecauseKFUsesSingularR);
};

#endif // RADAR_POSITION_NAVIGATION_CONTROLLER_HPP