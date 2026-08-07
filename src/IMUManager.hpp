/******************************************************************************
 * File:             IMUManager.hpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Declares the manager that synchronizes GPS and recorded or
 *                   live IMU payloads before dispatching filter measurements.
 *
 ******************************************************************************/

#ifndef INU_DISPLAY_IMUMANAGER_HPP
#define INU_DISPLAY_IMUMANAGER_HPP
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <optional>

#include <Eigen/Dense>
#include <gtest/gtest_prod.h>

#include "DatabaseManager.hpp"
#include "GpsUpdate.hpp"
#include "IMUManagerStats.hpp"
#include "MagneticDeclination.hpp"
#include "imu_data.hpp"
#include "utils.hpp"

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class IMUManager
{
public:
    /**
     * @brief Constructor
     *
     * @param [in] databaseManager Shared pointer to the Database Manager used to enqueue IMU and EKF output records for
     * persistence.
     * @param [in] ekfCallbackImuOnly Callback to the EKF Step(dt, z_IMU) method for IMU-only updates (no fresh GPS
     * available).
     * @param [in] ekfCallbackWithGps Callback to the EKF Step(dt, z_GPS, z_IMU) method for fused GPS+IMU updates
     *
     * @throws std::invalid_argument when databaseManager is nullptr
     */
    IMUManager(std::shared_ptr<DatabaseManager> databaseManager, std::string cofPath = "WMM.COF");

    /**
     * @brief Installs ekf. If none is installed, calls to ekf will not be made.
     *
     * @param ekfCallbackImuOnly ekf call without gps data
     * @param ekfCallbackWithGps ekf call with gps
     *
     * @throws std::invalid_argument when ekfCallbackImuOnly is nullptr
     * @throws std::invalid_argument when ekfCallbackWithGps is nullptr
     *
     * @return
     */
    void InstallEkf(std::function<void(double, Eigen::Matrix<double, 2, 1> &)> ekfCallbackImuOnly,
                    std::function<void(double, Eigen::Matrix<double, 2, 1> &, Eigen::Matrix<double, 2, 1> &)> ekfCallbackWithGps);

    /**
     * @brief Returns the runtime statistics of this class.
     *
     * @remarks The statistic includes:
     *      - number of imu measurements accepted and rejected
     *      - number of gps measurements accepted and rejected
     *      - number of failed ekfCallbacks
     *      - number of db enqueue failures
     *
     * @return IMUManagerStats include
     */
    IMUManagerStats GetStats() const;

    /**
     * @brief Returns the current GPS status if available
     *
     * @return std::optional<GpsUpdate> if gps data is available, else nullopt
     */
    std::optional<GpsUpdate> GetLatestGps() const;

    /**
     * @brief Method used to update IMUManager gps data
     *
     * @remarks Gps will not update if:
     *      - Incoming data is older than current GPS data
     *      - Older than 5 seconds from steady_clock::now()
     *      - Marked as update.valid == false
     *
     * @param [in] update gps data to be updated to
     *
     * @return
     */
    void UpdateLatestGps(const GpsUpdate &update);

  /**
   * @brief Call back upon host receives event from IMU sensor.
   *
   * @param [in] optLa Optional Linear Acceleration IMU measurement
   * @param [in] optRv Optional Rotation Vector with Accuracy IMU measurement
   *
   * @throws runtime_error if sensor report invalid measurements
   *
   * @return
   */
  void SensorCallback(std::optional<Raw_RotationVectorWAcc> optRv,
                      std::optional<Raw_Accelerometer> optLa,
                      std::optional<Raw_RotationRate> optRr);

private:
    /**
     * @brief checks whether:
     *    - Ekf callbacks are installed
     *    - m_imuRotationVector is updated with new unused data
     *    - m_imuLinearAcceleration is updated with new unused data
     *
     *  @returns true if all of the above is true
     */
    bool ReadyForEkf() const;

    /**
     * @brief helper function that invokes EKF callback. The function takes a snapshot of
     *    m_imuRotationVector, m_imuLienarAcceleration, m_latesetGps then use it to calculate
     *    true heading, 6d-Vector and Matrix used to feed into the EKF.
     *
     * @throws runtime_error if there has never been a gps update (same as std::nullopt)
     *
     * @return
     */
    void DispatchToEkf();

    /**
     * @brief Calculates elapsed measurement time from consecutive IMU payload timestamps.
     *
     * @remarks The first synchronized payload bundle establishes the timestamp baseline and returns zero because no prior
     *          measurement interval exists. Non-monotonic payload time also returns zero without moving the baseline.
     *
     * @return Elapsed payload time in seconds, or zero when an interval cannot be derived from two monotonic bundles.
     */
    double PrepareEkfTiming();

    /**
     * @brief Resets both m_imuRotationVectorReady and m_imuLinearAccelerationReady to false
     *
     * @return
     */
    void ResetImuReadyFlags();

    /**
     * @brief Checks if a number is out of numerical bounds
     *
     * @param [in] x number to be checked
     *
     * @return true if number is out of bounds else false
     */
    template <typename T>
    static bool IsInvalidRange(T x)
    {
        T maxLimit = std::numeric_limits<T>::max();
        T minLimit = std::numeric_limits<T>::min();
        return (x <= minLimit) || (x >= maxLimit) || std::isnan(x) || !std::isfinite(x);
    }

  /**
   * @brief Validates incoming IMU events for troublesome values
   *
   * @param [in] optLa optional imu sensor linear acceleration
   * @param [in] optRv optional imu sensor rotation vector
   *
   * @return True if the sensor event contains usable IMU data
   */
  static bool ValidateImuEvent(const std::optional<Raw_RotationVectorWAcc> &optRv,
                               const std::optional<Raw_Accelerometer> &optLa,
                               const std::optional<Raw_RotationRate> &optRr);

  /**
   * @brief Storing IMU Value to its respective member variable
   *
   * @param [in] optLa optional imu sensor linear acceleration
   * @param [in] optRv optional imu sensor rotation vector
   *
   * @return
   */
  void StoreImuValue(const std::optional<Raw_RotationVectorWAcc> &optRv,
                     const std::optional<Raw_Accelerometer> &optLa,
                     const std::optional<Raw_RotationRate> &optRr);

    /**
     * @brief Build an Eigen vector representation of GpsUpdate data
     *
     * @param [in] gps gps data struct
     *
     * @return Vector6d EKF-ready GPS measurement vector [x, y, 0, 0, 0, 0]^T in local coordinates.
     */
    Eigen::Matrix<double, 2, 1> BuildGpsMeasurementVector(const GpsUpdate &gps);

    /**
     *
     * @brief   Rotates body-frame linear acceleration into true ENU coordinates, smooths acceleration and yaw rate, and
     *          projects ENU acceleration onto the payload-derived vehicle heading for the Kalman-filter control input.
     *
     * @param [in] rv           Rotation-vector quaternion used to transform body acceleration and predict heading.
     * @param [in] la           Body-frame linear acceleration measurement in meters per second squared.
     * @param [in] rr           Euler rotation-rate measurement in radians per second.
     * @param [in] gps          GPS position used to calculate magnetic declination for the measurement location.
     * @param [in] currentYear  UTC measurement year parsed from the GPS RMC payload.
     * @param [in] dt           Elapsed measurement time in seconds derived from consecutive IMU payload timestamps.
     *
     * @return  Two-element filter control vector containing forward acceleration in meters per second squared followed by
     *          smoothed yaw rate in radians per second.
     *
     * @throws  std::runtime_error  If RotateLinearAccelToTrueENU() or ComputeENUHeading() rejects an invalid quaternion.
     */
    Eigen::Matrix<double, 2, 1> BuildImuMeasurementVector(const Raw_RotationVectorWAcc &rv,
                                       const Raw_Accelerometer &la,
                                       const Raw_RotationRate &rr,
                                       const GpsUpdate &gps,
                                       int currentYear,
                                       double dt);

    bool m_imuRotationVectorReady = false;            // True when class is updated with new RotationVector measurement and not used yet in EKF
    bool m_imuRotationRateReady = false;            // True when class is updated with new RotationVector measurement and not used yet in EKF
    bool m_imuLinearAccelerationReady = false;        // True when class is updated with new LinearAcceleration measurement and not used yet in EKF
    Raw_RotationVectorWAcc m_imuRotationVector = {};  // Internal RotationVector measurement state
    Raw_RotationRate m_imuRotationRate = {};          // Internal RotationRate measurement state
    Raw_Accelerometer m_imuLinearAcceleration = {};   // Internal LinearAcceleration measurement state

    uint64_t m_lastEKFMachineTime = 0;    // Machine time of the oldest time used in the EKF innovation in micro seconds

    bool m_gpsSentToEkf = false;          // Flag indicating latestGps is sent to ekf
    bool m_ekfInstalled = false;          // True if installed Ekf, else no ekf installed, no call to ekf will be made
    std::optional<GpsUpdate> m_latestGps; // Internal GpsUpdate data state

    mutable std::mutex m_gpsMutex;         // Mutex used when m_latestGps is read/written

    MagneticDeclination m_magneticDeclination;          // MagneticDeclination member used to calculate declination angle in BuildImuMeasurementVector()
    std::shared_ptr<DatabaseManager> m_databaseManager; // shared ptr to DatabaseManager used to store incoming data persistently

    std::function<void(double, Eigen::Matrix<double, 2, 1> &)> m_ekfCallbackImuOnly;             // EKF callback without new GPS data
    std::function<void(double, Eigen::Matrix<double, 2, 1> &, Eigen::Matrix<double, 2, 1> &)> m_ekfCallbackWithGps; // EKF callback with new unused GPS data

    double m_muEast;
    double m_muNorth;
    double m_muYaw;
    double m_EMWA_alpha;

    FRIEND_TEST(IMUManagerTest, IsInvalidRangeReturnsTrue);
    FRIEND_TEST(IMUManagerTest, IsInvalidRangeReturnsFalse);
    FRIEND_TEST(IMUManagerTest, GetLatestGpsReturnsNullopt);
    FRIEND_TEST(IMUManagerTest, ValidateImuEventReturnsTrue);
    FRIEND_TEST(IMUManagerTest, ValidateImuEventReturnsFalse);
    FRIEND_TEST(IMUManagerTest, StoreImuValueReturnsVoid);
    FRIEND_TEST(IMUManagerTest, BuildGpsMeasurementVectorReturnsVector);
    FRIEND_TEST(IMUManagerTest, BuildImuMeasurementVectorReturnsVector);
    FRIEND_TEST(IMUManagerTest, IngestSensorValueThrowsRuntimeError);
    FRIEND_TEST(IMUManagerTest, ReadyForEkfReturnsFalse);
    FRIEND_TEST(IMUManagerTest, ReadyForEkfReturnsTrue);
    FRIEND_TEST(IMUManagerTest, PrepareEkfTimingReturnsDtSeconds);
    FRIEND_TEST(IMUManagerTest, PrepareEkfTimingRejectsNonMonotonicPayloadTime);
    FRIEND_TEST(IMUManagerTest, ResetImuReadyFlagsExpectsFalse);
};

#endif // INU_DISPLAY_IMUMANAGER_HPP
