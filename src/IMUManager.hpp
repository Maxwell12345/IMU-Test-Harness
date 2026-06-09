#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP
#pragma once

#include <chrono>
#include <optional>
#include <functional>

#include <Eigen/Dense> 
#include <gtest/gtest_prod.h> 
#include <boost/shared_ptr.hpp>

#include "utils.hpp"
#include "GpsUpdate.hpp"
#include "IMUManagerStats.hpp"
#include "MagneticDeclination.hpp"

extern "C" {
#include <sh2.h>
#include <sh2_SensorValue.h>
#include "sh2_err.h"
}

class DatabaseManager;

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class IMUManager {
private:
    /**
     * @brief Constructor
     *
     * @param [in] databaseManager Shared pointer to the Database Manager used to enqueue IMU and EKF output records for persistence.
     * @param [in] ekfCallbackImuOnly Callback to the EKF Step(dt, z_IMU) method for IMU-only updates (no fresh GPS available).
     * @param [in] ekfCallbackWithGps Callback to the EKF Step(dt, z_GPS, z_IMU) method for fused GPS+IMU updates
     */
    IMUManager(boost::shared_ptr<DatabaseManager> databaseManager,
               std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> ekfCallbackImuOnly,
               std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> ekfCallbackWithGps);

    /**
     * @brief Destructor
     *
     * @remarks reset all static members to initial values
     */
    ~IMUManager();

public:

    /**
     * @brief Singleton Pattern Initializer
     *
     * @remarks MUST BE CALLED, only ONCE. If not, code will get segfault, though attempts at catching has been made
     *
     * @param [in] databaseManager Shared pointer to the Database Manager used to enqueue IMU and EKF output records for persistence.
     * @param [in] ekfCallbackImuOnly Callback to the EKF Step(dt, z_IMU) method for IMU-only updates (no fresh GPS available).
     * @param [in] ekfCallbackWithGps Callback to the EKF Step(dt, z_GPS, z_IMU) method for fused GPS+IMU updates
     *
     * @return
     *
     * @throws runtime_error when Singleton instance already exists
     */
    static void Initialize(boost::shared_ptr<DatabaseManager> databaseManager,
                           std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> ekfCallbackImuOnly,
                           std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> ekfCallbackWithGps);

    /**
     * @brief Clean up singleton instance
     */
    static void Deinitialize();

    /**
     * @brief Reference to singleton instance of IMUManager
     *
     * @return IMUManager reference
     *
     * @throws runtime_error when Singleton instance is not initialized (nullptr)
     */
    static IMUManager& Instance();

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
    static std::optional<GpsUpdate> GetLatestGps();

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
    static void UpdateLatestGps(const GpsUpdate& update);

    /**
     * @brief Call back upon host receives event from IMU sensor.
     * 
     * @remark Method does not throw exception because it is a callback from C,
     *         but will cerr if:
     *      - s_instance is nullptr
     *      - event is nullptr
     *      - failure to decode event
     *      - out of bound IMU measurements
     *      - unsupported IMU report type
     *      - no gps ever recorded
     *         Exits if no GPS ever recoded
     *
     *
     * @param [in] cookie (unused) Opaque user-data pointer passed by the SH2 driver
     * @param [in] event The pointer to an undecoded report coming from the IMU
     *
     * @throws SEE ABOVE, internally handled
     * @throws runtime_error if s_instance does not exist
     * @throws runtime_error if event is nullptr
     * @throws runtime_error if decode event failure
     * @throws runtime_error if sensorId is not supported
     * @throws runtime_error if sensor report invalid measurements
     */
    static void SensorCallback(void* cookie, sh2_SensorEvent* event);

    /**
     * @brief This is just a method similar to SensorCallback(void* cookie, sh2_SensorEvent* event),
     *        but dev can modify it to play around
     * 
     * @remark This method is for testing. It behaves almost the same way as SensorCallback(void* cookie, sh2_SensorEvent* event).
     *
     * @param [in] val a referece to a sh2_SensorValue populated with IMU measurements already. (Events decoded before calling this method)
     */
    static void SensorCallback(const sh2_SensorValue& val);

private:

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
        return std::isnan(x) || std::isinf(x);
    }

    /**
     * @brief Validates incoming IMU events for troublesome values
     *
     * @param [in] sensorValue imu sensor value containing sensor type and measurement data
     *
     * @return True if the sensor event contains usable IMU data
     */
    static bool ValidateImuEvent(const sh2_SensorValue& sensorValue);

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
    static void StoreImuValue(const sh2_SensorValue& sensorValue);

    /**
     * @brief Build an Eigen vector representation of GpsUpdate data
     * 
     * @param [in] gps gps data struct
     *
     * @return Vector6d EKF-ready GPS measurement vector [x, y, 0, 0, 0, 0]^T in local coordinates.
     */
    static Vector6d BuildGpsMeasurementVector(const GpsUpdate& gps);

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
    static Vector6d BuildImuMeasurementVector(const sh2_RotationVectorWAcc& rv, const sh2_Accelerometer& la, const GpsUpdate& gps, int currentYear);
    
    static IMUManager* s_instance;                      // Singleton IMUManager instance. Only one per run time

    static std::mutex s_lastImuEkfInvocationMutex;      // Mutex used when s_lastImuTimestamp is read/writen
    static std::mutex s_imuValueMutex;                  // Mutex used when s_ImuMagneticField/s_ImuLinearAcceleration modified / read
    static bool s_imuRotationVectorReady;               // True when class is updated with new RotationVector measurement and not used yet in EKF
    static bool s_imuLinearAccelerationReady;           // True when class is updated with new LinearAcceleration measurement and not used yet in EKF
    static sh2_RotationVectorWAcc s_imuRotationVector;  // Internal RotationVector measurement state
    static sh2_Accelerometer s_imuLinearAcceleration;   // Internal LinearAcceleration measurement state
    static std::chrono::steady_clock::time_point s_lastImuEkfInvocation;    // Last timepoint when EKF was invoked, used to calculate dt for EKF as well
    
    static std::mutex s_gpsMutex;                       // Mutex used when s_latestGps is read/written
    static bool s_gpsSentToEkf;                         // Flag indicating latestGps is sent to ekf
    static std::optional<GpsUpdate> s_latestGps;        // Internal GpsUpdate data state

    static std::mutex s_kineticStateMutex;              // Mutex used when s_kineticState is read/written
    static IMUUtils::KineticState s_kineticState;       // Internal KineticState data state
    
    mutable std::mutex m_statsMutex;                    // Mutex used when m_statsMutex is read/written
    IMUManagerStats m_stats;                            // Internal IMUManagerStats data state, holds accepted and rejected incoming IMU and Gps data

    MagneticDeclination m_magneticDeclination;          // MagneticDeclination member used to calculate declination angle in BuildImuMeasurementVector()
    boost::shared_ptr<DatabaseManager> m_databaseManager;   // UNIMPLEMENTED shared ptr to DatabaseManager used to store incoming data persistently

    std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> m_ekfCallbackImuOnly;               // EKF callback without new GPS data
    std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> m_ekfCallbackWithGps;    // EKF callback with new unused GPS data

    FRIEND_TEST(IMUManagerTest, IsInvalidRangeReturnsTrue);
    FRIEND_TEST(IMUManagerTest, IsInvalidRangeReturnsFalse);
    FRIEND_TEST(IMUManagerTest, GetLatestGpsReturnsNullopt);
    FRIEND_TEST(IMUManagerTest, ValidateImuEventReturnsTrue);
    FRIEND_TEST(IMUManagerTest, ValidateImuEventReturnsFalse);
    FRIEND_TEST(IMUManagerTest, BuildGpsMeasurementVectorReturnsVector);
    FRIEND_TEST(IMUManagerTest, BuildImuMeasurementVectorReturnsVector);
};

#endif