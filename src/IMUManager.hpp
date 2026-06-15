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

extern "C" {
#include "sh2_err.h"
#include <sh2.h>
#include <sh2_SensorValue.h>
}

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
   */
  explicit IMUManager(boost::shared_ptr<DatabaseManager> databaseManager = nullptr, std::string cofPath = "./WMM.COF");

  /**
   * @brief Destructor
   *
   * @remarks reset all static members to initial values
   */
  ~IMUManager();

  /**
   * @ brief Delete Pattern
   *
   * @remarks safety mechanism to garuntee only one instance of IMUManager exists at a time
   */
  IMUManager(const IMUManager &) = delete;
  IMUManager &operator=(const IMUManager &) = delete;

  /**
   * @brief Installs ekf. If none is installed, calls to ekf will not be made.
   *
   * @param ekfCallbackImuOnly ekf call without gps data
   * @param ekfCallbackWithGps ekf call with gps
   *
   * @return
   */
  void InstallEkf(
      std::function<void(double, Vector6d &)> ekfCallbackImuOnly, std::function<void(double, Vector6d &, Vector6d &)> ekfCallbackWithGps
  );

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
   * @remark Method does not throw exception because it is a callback from C,
   *         but will cerr if:
   *      - m_sInstance is nullptr
   *      - event is nullptr
   *      - failure to decode event
   *      - out of bound IMU measurements
   *      - unsupported IMU report type
   *      - no gps ever recorded
   *         Exits if no GPS ever recoded
   *
   *
   * @param [in] cookie Opaque user-data pointer passed by the SH2 driver
   * @param [in] event The pointer to an undecoded report coming from the IMU
   *
   * @throws SEE ABOVE, internally handled
   * @throws runtime_error if m_sInstance does not exist
   * @throws runtime_error if event is nullptr
   * @throws runtime_error if decode event failure
   * @throws runtime_error if sensorId is not supported
   * @throws runtime_error if sensor report invalid measurements
   */
  static void SensorCallback(void *cookie, sh2_SensorEvent *event);

  /**
   *
   *
   */
  void IngestSensorValue(const sh2_SensorValue &value);

private:
  /**
   *
   *
   */
  bool ReadyForEkf() const;

  /**
   *
   *
   */
  void DispatchToEkf();

  /**
   * @brief Decodes sh2 sensor event
   *
   * @param cookie A value that will be passed to the sensor callback function.
   * @param event
   */
  void OnSensorEvent(sh2_SensorEvent *event);

  /**
   *
   *
   */
  sh2_RotationVectorWAcc loadRotationVectorSnapshot();

  /**
   *
   *
   */
  sh2_Accelerometer loadLinearAccelerationSnapshot();
  /**
   *
   *
   */
  int getCurrentYear() const;

  /**
   *
   *
   */
  double prepareEkfTiming();

  /**
   *
   *
   */
  void resetImuReadyFlags();

  /**
   * @brief Checks if a number is out of numerical bounds
   *
   * @param [in] x number to be checked
   *
   * @return true if number is out of bounds else false
   */
  template <typename T> static bool IsInvalidRange(T x) { return std::isnan(x) || std::isinf(x); }

  /**
   * @brief Validates incoming IMU events for troublesome values
   *
   * @param [in] sensorValue imu sensor value containing sensor type and measurement data
   *
   * @return True if the sensor event contains usable IMU data
   */
  static bool ValidateImuEvent(const sh2_SensorValue &sensorValue);

  /**
   * @brief Storing IMU Value to its respective static member variable
   *
   * @remarks IMU sensor ids maps to the following static member variables:
   *      SH2_LINEAR_ACCELERATION -> s_ImuAccelerometer
   *      SH2_MAGNETIC_FIELD_CALIBRATED -> s_ImuMagnetometer
   *
   * @param sensorValue reference to the decoded sensor value
   *
   * @return
   */
  void StoreImuValue(const sh2_SensorValue &sensorValue);

  /**
   * @brief Build an Eigen vector representation of GpsUpdate data
   *
   * @param [in] gps gps data struct
   *
   * @return Vector6d EKF-ready GPS measurement vector [x, y, 0, 0, 0, 0]^T in local coordinates.
   */
  static Vector6d BuildGpsMeasurementVector(const GpsUpdate &gps);

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
  Vector6d BuildImuMeasurementVector(const sh2_RotationVectorWAcc &rv, const sh2_Accelerometer &la, const GpsUpdate &gps, int currentYear);

  bool m_imuRotationVectorReady = false;     // True when class is updated with new RotationVector measurement and not used yet in EKF
  bool m_imuLinearAccelerationReady = false; // True when class is updated with new LinearAcceleration measurement and not used yet in EKF
  sh2_RotationVectorWAcc m_imuRotationVector = {}; // Internal RotationVector measurement state
  sh2_Accelerometer m_imuLinearAcceleration = {};  // Internal LinearAcceleration measurement state

  uint64_t m_lastRotationVectorMachineTime = 0;     // Machine time of rotation vector from IMU in micro seconds
  uint64_t m_lastAccelerationVectorMachineTime = 0; // Machine time of the acceleration vector from IMU in micro seconds
  uint64_t m_lastEKFMachineTime = 0;                // Machine time of the oldest time used in the EKF innovation in micro seconds

  bool m_gpsSentToEkf = false;          // Flag indicating latestGps is sent to ekf
  bool m_ekfInstalled = false;          // True if installed Ekf, else no ekf installed, no call to ekf will be made
  std::optional<GpsUpdate> m_latestGps; // Internal GpsUpdate data state

  // TODO: atomic, also refactor KineticState struct
  std::mutex m_kineticStateMutex;        // Mutex used when m_kineticState is read/written
  mutable std::mutex m_gpsMutex;         // Mutex used when m_latestGps is read/written
  IMUUtils::KineticState m_kineticState; // Internal KineticState data state

  IMUManagerStats m_stats; // Internal IMUManagerStats data state, holds accepted and rejected incoming IMU and Gps data

  MagneticDeclination
      m_magneticDeclination; // MagneticDeclination member used to calculate declination angle in BuildImuMeasurementVector()
  boost::shared_ptr<DatabaseManager>
      m_databaseManager; // UNIMPLEMENTED shared ptr to DatabaseManager used to store incoming data persistently

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
};

#endif