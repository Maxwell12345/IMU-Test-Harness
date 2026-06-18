#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <optional>

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <gtest/gtest_prod.h>

#include "DatabaseManager.hpp"
#include "GpsUpdate.hpp"
#include "IMUManagerStats.hpp"
#include "MagneticDeclination.hpp"
#include "utils.hpp"
#include "SerialDataModel.hpp"

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class IMUManager {
public:
  /**
   * @brief Constructor
   *
   * @param [in] databaseManager Shared pointer to the Database Manager used to enqueue IMU and EKF output records for persistence.
   * @param [in] ekfCallbackImuOnly Callback to the EKF Step(dt, z_IMU) method for IMU-only updates (no fresh GPS available).
   * @param [in] ekfCallbackWithGps Callback to the EKF Step(dt, z_GPS, z_IMU) method for fused GPS+IMU updates
   *
   * @throws std::invalid_argument when databaseManager is nullptr
   */
  IMUManager(std::shared_ptr<DatabaseManager> databaseManager, std::string cofPath = "./WMM.COF");

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
  void InstallEkf(std::function<void(double, Vector6d &)> ekfCallbackImuOnly,
                  std::function<void(double, Vector6d &, Vector6d &)> ekfCallbackWithGps);

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
  void SensorCallback(std::optional<LinearAcceleration> optLa, std::optional<RotationVectorWAcc> optRv);

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
   * @brief uses system_clock::now() to get current calendar year
   *
   * @return current year in YYYY format
   */
  int GetCurrentYear() const;

  /**
   * @brief Calculates dt from last EKF invocation
   *
   * @remarks Assume m_imuRotationVector and m_imuLinearAcceleration are newly updated and unused
   *    and m_lastEKFMachineTime exists:
   *    dt = firstNewlyArrivedImuMeausurementTimestamp - m_lastEKFMachineTime
   *
   * @returns dt in seconds
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
  template <typename T> static bool IsInvalidRange(T x) {
    T maxLimit = std::numeric_limits<T>::max();
    T minLimit = std::numeric_limits<T>::min();
    return (x <= minLimit) ||
           (x >= maxLimit) ||
            std::isnan(x)  ||
           !std::isfinite(x);
  }

  /**
   * @brief Validates incoming IMU events for troublesome values
   *
   * @param [in] sensorValue imu sensor value containing sensor type and measurement data
   *
   * @return True if the sensor event contains usable IMU data
   */
  static bool ValidateImuEvent(std::optional<LinearAcceleration> optLa, std::optional<RotationVectorWAcc> optRv);

  /**
   * @brief Storing IMU Value to its respective static member variable
   *
   * @param sensorValue reference to the decoded sensor value
   *
   * @return
   */
  void StoreImuValue(std::optional<LinearAcceleration> optLa, std::optional<RotationVectorWAcc> optRv);

  /**
   * @brief Build an Eigen vector representation of GpsUpdate data
   *
   * @param [in] gps gps data struct
   *
   * @return Vector6d EKF-ready GPS measurement vector [x, y, 0, 0, 0, 0]^T in local coordinates.
   */
  Vector6d BuildGpsMeasurementVector(const GpsUpdate &gps);

  /**
   * @brief Build an Eigen vector representation of SensorValue data
   *
   * @remarks Converts local IMU measurents and gps data to velocity and acceleration
   *      in Global frame of reference in Geodetic units. Applies Magnetic Declination
   *      to rotation vector using GPS coordinate.
   *
   * @param [in] rv the rotation vector snapshot from IMU (i, j, k, w)
   * @param [in] la the linear acceleration measurement snapshot from IMU (m/s^2)
   * @param [in] gps the latest gps snapshot
   * @param [in] currentYear the current year YYYY
   *
   * @return Vector6d EKF-ready IMU measurement vector [0, 0, vx, vy, ax, ay]^T in the navigation frame.
   */
  Vector6d BuildImuMeasurementVector(const RotationVectorWAcc &rv,
                                     const LinearAcceleration &la,
                                     const GpsUpdate &gps,
                                     int currentYear);

  bool m_imuRotationVectorReady = false;            // True when class is updated with new RotationVector measurement and not used yet in EKF
  bool m_imuLinearAccelerationReady = false;        // True when class is updated with new LinearAcceleration measurement and not used yet in EKF
  RotationVectorWAcc m_imuRotationVector = {};      // Internal RotationVector measurement state
  LinearAcceleration m_imuLinearAcceleration = {};  // Internal LinearAcceleration measurement state

  uint64_t m_lastEKFMachineTime = 0;                // Machine time of the oldest time used in the EKF innovation in micro seconds

  bool m_gpsSentToEkf = false;          // Flag indicating latestGps is sent to ekf
  bool m_ekfInstalled = false;          // True if installed Ekf, else no ekf installed, no call to ekf will be made
  std::optional<GpsUpdate> m_latestGps; // Internal GpsUpdate data state

  // TODO: atomic, also refactor KineticState struct
  std::mutex m_kineticStateMutex;        // Mutex used when m_kineticState is read/written
  mutable std::mutex m_gpsMutex;         // Mutex used when m_latestGps is read/written
  IMUUtils::KineticState m_kineticState; // Internal KineticState data state

  IMUManagerStats m_stats; // Internal IMUManagerStats data state, holds accepted and rejected incoming IMU and Gps data

  MagneticDeclination m_magneticDeclination;            // MagneticDeclination member used to calculate declination angle in BuildImuMeasurementVector()
  std::shared_ptr<DatabaseManager> m_databaseManager; // shared ptr to DatabaseManager used to store incoming data persistently

  std::function<void(double, Vector6d &)> m_ekfCallbackImuOnly;             // EKF callback without new GPS data
  std::function<void(double, Vector6d &, Vector6d &)> m_ekfCallbackWithGps; // EKF callback with new unused GPS data

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
  FRIEND_TEST(IMUManagerTest, GetCurrentYearReturnsYear);
  FRIEND_TEST(IMUManagerTest, PrepareEkfTimingReturnsDtSeconds);
  FRIEND_TEST(IMUManagerTest, ResetImuReadyFlagsExpectsFalse);
};

#endif