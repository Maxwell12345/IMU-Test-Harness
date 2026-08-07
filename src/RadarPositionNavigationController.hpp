/******************************************************************************
 * File:             RadarPositionNavigationController.hpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Declares the navigation controller that coordinates IMU
 *                   processing, ENU GPS fusion, and Kalman-filter state access.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_RADARPOSITIONNAVIGATIONCONTROLLER_HPP
#define INU_DISPLAY_RADARPOSITIONNAVIGATIONCONTROLLER_HPP

#include <Eigen/Dense>
#include <atomic>
#include <functional>
#include <gtest/gtest_prod.h>
#include <iostream>
#include <mutex>
#include <optional>
#include <utility>

#include "GpsUpdate.hpp"
#include "IMUGPSFusionKF.hpp"
#include "IMUManager.hpp"
#include "IMUSerialPortReader.hpp"
#include "SerialPortBase.hpp"
#include "YamlConfigService.hpp"
#include "YamlConfig.hpp"

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class RadarPositionNavigationController {
public:
    RadarPositionNavigationController(const _KalmanValues& config,
                                        std::shared_ptr<DatabaseManager> databaseManager,
                                        std::unique_ptr<IMUSerialPortReader> imuSerialPortReader,
                                        std::unique_ptr<IMUManager> m_imuManager);

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
    std::function<void(const GpsUpdate &)> GetGPSCallback();

    /**
     * @brief Begins self tracking process. We assume all order
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
     * @brief Stops self tracking process.
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
     * @brief Stops self tracking process.
     *
     * @return
     *
     * @remarks This logically kills all tracking and destroys KF memory entirely.
     *
     * @exception
     */
    void TotalDestruction();

    /**
     * @brief Returns PNT running status
     * 
     * @return true if running, else false
     */
    bool IsRunning() const;

    /**
     *
     * @brief   Returns a thread-safe snapshot of the most recently accepted Kalman-filter state in the active ENU frame. The
     *          six elements contain easting, northing, heading, speed, yaw rate, and longitudinal acceleration in that order.
     *
     * @return  Copy of the latest six-element Kalman-filter state vector. Easting and northing are measured in meters, heading
     *          in radians, speed in meters per second, yaw rate in radians per second, and acceleration in meters per second
     *          squared.
     *
     * @remarks Acquires the controller's Kalman-filter update mutex so the returned vector cannot contain a partially written
     *          filter update.
     *
     * @throws  std::system_error  If the Kalman-filter update mutex cannot be locked.
     */
    Vector6d GetKFState() const;

    /**
     *
     * @brief   Returns a thread-safe snapshot of the covariance associated with the state returned by GetKFState().
     *
     * @return  Copy of the latest six-by-six Kalman-filter covariance matrix in state order: easting, northing, heading,
     *          speed, yaw rate, and longitudinal acceleration.
     *
     * @remarks Acquires the controller's Kalman-filter update mutex so the returned matrix cannot contain a partially written
     *          filter update.
     *
     * @throws  std::system_error  If the Kalman-filter update mutex cannot be locked.
     */
    Matrix6d GetKFCovariance() const;

    /**
     *
     * @brief   Converts the latest Kalman-filter easting and northing from the active local ENU frame into WGS84 longitude
     *          and latitude while holding the filter-state mutex.
     *
     * @return  Pair containing WGS84 longitude in the first element and WGS84 latitude in the second element, both in
     *          decimal degrees.
     *
     * @remarks The conversion uses the same active ENU origin as the filter state returned by GetKFState().
     *
     * @throws  std::runtime_error  If the controller has not established an ENU origin.
     * @throws  std::system_error   If the Kalman-filter update mutex cannot be locked.
     */
    std::pair<double, double> GetKFWGS84Position() const;

private:
    /**
     * @brief Starts the IMU serial reader service if one was constructed.
     *    Called by StartAndConfigureRadarPNT() to begin feeding data
     *    from the serial port into IMUManager + the Kalman filter.
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
    void ConfigureKalmanFilter(double lat0, double lon0);

    /**
     * @brief Execute KF with IMU measurement only.
     *
     * @param [in] dt the time delta from the last step to the current time (measurement time).
     * @param [in] imuVec the IMU measurement vector. This is a column vector of 6 items [null, null, vlon, vlat, alon,
     * alat]^T
     *
     * @return
     *
     * @remarks Run single shot fusion EKF algorithm with null GPS for deadreckoning without GPS.
     *
     * @exception std::runtime_error requires positive non zero df values and percentiles.
     * @exception std::exception KF initialization or Step error.
     */
    void KFCallbackImuOnly(double dt, Eigen::Matrix<double, 2, 1> &imuVec);

    /**
     * @brief Execute KF with IMU and GPS measurement.
     *
     * @param [in] dt the time delta from the last step to the current time (measurement time).
     * @param [in] imuVec the IMU measurement vector. This is a column vector of 6 items [null, null, vlon, vlat, alon,alat]^T
     * 
     * @param [in] gpsVec the IMU measurement vector. This is a column vector of 6 items [lon, lat, null, null, null,null]^T
     * 
     *
     * @return
     *
     * @remarks Run single shot fusion EKF algorithm with GPS measurement to adjust and fuse position kinematics.
     *
     * @exception std::runtime_error requires positive non zero df values and percentiles.
     * @exception std::exception KF initialization or Step error.
     */
    void KFCallbackWithGps(double dt, Eigen::Matrix<double, 2, 1> &imuVec, Eigen::Matrix<double, 2, 1> &gpsVec);

    /**
     * @brief GPS callback service in order to set IMUManager GPS state.
     *
     * @return
     *
     * @remarks
     *
     * @exception
     */
    void _GPSCallback(const GpsUpdate &gpsUpdate);

    /**
     *
     * @brief   Converts a geodetic GPS position to easting and northing in the active local ENU frame. When the GPS
     *          position is more than 100 meters from the active origin, establishes a new origin at the GPS position and
     *          returns coordinates in that new frame.
     *
     * @param [out] easting    GPS easting in meters relative to the active origin on return.
     * @param [out] northing   GPS northing in meters relative to the active origin on return.
     * @param [in]  latitude   GPS latitude in decimal degrees.
     * @param [in]  longitude  GPS longitude in decimal degrees.
     *
     * @return  True when the active ENU origin changed; otherwise false.
     */
    bool ConvertGPSToENU(double& easting, double& northing, double latitude, double longitude);

    /**
     *
     * @brief   Checks a predicted Kalman-filter position against the active ENU origin and recenters both the supplied state
     *          and the filter's internal position when the state is more than 100 meters from that origin.
     *
     * @param [in,out] state  Six-element Kalman-filter state to inspect. When recentering occurs, easting and northing are
     *                        rewritten in the new frame and heading is adjusted for the tangent-frame change.
     *
     * @return  True when the active ENU origin changed; otherwise false.
     *
     * @throws  std::runtime_error  If no geodetic origin is available for the coordinate conversion.
     */
    bool ValidateAndUpdateENUOrigin(Eigen::Matrix<double, 6, 1> &state);

    /**
     *
     * @brief   Converts the easting and northing in a Kalman-filter state into WGS84 latitude and longitude using the active
     *          ENU origin.
     *
     * @param [out] latitude   Converted WGS84 latitude in decimal degrees.
     * @param [out] longitude  Converted WGS84 longitude in decimal degrees.
     * @param [in]  state      Six-element state whose first two elements are easting and northing in meters.
     *
     * @return
     */
    void ConvertKFStateToWGS84(double& latitude,
                               double& longitude,
                               Eigen::Matrix<double, 6, 1> &state);

    /**
     *
     * @brief   Re-expresses the latest Kalman-filter position and the filter's internal state in the current ENU frame after
     *          a GPS measurement establishes a new origin.
     *
     * @param [in] oldLatOrigin  Latitude of the previous ENU origin in decimal degrees.
     * @param [in] oldLonOrigin  Longitude of the previous ENU origin in decimal degrees.
     *
     * @return
     */
    void RestKFOrigin(double oldLatOrigin, double oldLonOrigin);

private:
    Vector6d m_latestX;
    Matrix6d m_latestP;
    mutable std::mutex m_kFUpdateMutex;
    IMUGPSFusionKF m_kf;
    
    const _KalmanValues& m_config;
    
    std::atomic<bool> m_running;
    std::atomic<bool> m_isKFConfigured;

    std::atomic<bool> m_hasOrigin;
    std::pair<double,double> m_originLatLon;
    std::pair<double,double> m_originEN;    

    std::unique_ptr<IMUManager> m_imuManager;
    std::unique_ptr<IMUSerialPortReader> m_imuSerialPortReader;

    std::shared_ptr<DatabaseManager> m_databaseManager;

    FRIEND_TEST(RadarPositionNavigationControllerTest, GetGPSCallbackUpdatesLatestGps);
    FRIEND_TEST(RadarPositionNavigationControllerTest, StartAndConfigureRadarPNTConfiguresKFAndStartsReader);
    FRIEND_TEST(RadarPositionNavigationControllerTest, StopRadarPNTStopsThreadAndClosesSerial);
    FRIEND_TEST(RadarPositionNavigationControllerTest, TotalDestructionStopsReaderCleansKFAndZerosLatestState);
    FRIEND_TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterSetsInitialStateAndCovariance);
    FRIEND_TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsInvalidPercentiles);
    FRIEND_TEST(RadarPositionNavigationControllerTest, ConfigureKalmanFilterRejectsLowerPercentileGreaterThanUpperPercentile);
    FRIEND_TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyReturnsWithoutConfiguredKF);
    FRIEND_TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsReturnsWithoutConfiguredKF);
    FRIEND_TEST(RadarPositionNavigationControllerTest, KFCallbackImuOnlyProducesNonFiniteStateBecauseKFUsesSingularR);
    FRIEND_TEST(RadarPositionNavigationControllerTest, KFCallbackWithGpsProducesNonFiniteStateBecauseKFUsesSingularR);
    FRIEND_TEST(RadarPositionNavigationControllerTest, YamlFileParsingForKalmanFilterValuesExpectingTryCatch);
    FRIEND_TEST(RadarPositionNavigationControllerTest, YamlFileParsingForKalmanFilterValuesExpecting);
    FRIEND_TEST(RadarPositionNavigationControllerTest, GpsOriginResetPreservesPhysicalFilterPosition);
    FRIEND_TEST(RadarPositionNavigationControllerTest, FilterOriginResetRecentersReturnedState);
    FRIEND_TEST(RadarPositionNavigationControllerTest, UnknownPayloadIntervalDoesNotAdvanceFilter);
};

#endif // INU_DISPLAY_RADARPOSITIONNAVIGATIONCONTROLLER_HPP
