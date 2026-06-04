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

    ~IMUManager() = default;

public:

    /**
     * @brief Singleton Pattern Initializer
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
     *
     * @param [in] cookie Opaque user-data pointer passed by the SH2 driver
     * @param [in] event The pointer to an undecoded report coming from the IMU
     */
    static void SensorCallback(void* cookie, sh2_SensorEvent* event);

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
     * @param [in] sensorValue imu sensor value containing sensor type and measurement data
     *
     * @return Vector6d EKF-ready IMU measurement vector [0, 0, vx, vy, ax, ay]^T in the navigation frame.
     */
    static Vector6d BuildImuMeasurementVector(const sh2_RotationVector& rv, const sh2_Accelerometer& la, const GpsUpdate& gps);
    
    static IMUManager* s_instance;              // Singleton IMUManager instance. Only one per run time

    static std::mutex s_lastImuEkfInvocationMutex;   // Mutex used when s_lastImuTimestamp is read/writen
    static std::mutex s_imuValueMutex;                  // Mutex used when s_ImuMagneticField/s_ImuLinearAcceleration modified / read
    static bool s_imuRotationVectorReady;               
    static bool s_imuLinearAccelerationReady;         
    static sh2_RotationVector s_imuRotationVector;    
    static sh2_Accelerometer s_imuLinearAcceleration;
    static std::chrono::steady_clock::time_point s_lastImuEkfIvocation;
    
    static std::mutex s_gpsMutex;               // Mutex used when s_latestGps is read/written
    static bool s_gpsSentToEkf;                 // Flag indicating latestGps is sent to ekf
    static std::optional<GpsUpdate> s_latestGps;

    static std::mutex s_kineticStateMutex;      // Mutex used when s_kineticState is read/written
    static IMUUtils::KineticState s_kineticState;
    
    mutable std::mutex m_statsMutex;

    IMUManagerStats m_stats;
    MagneticDeclination m_magneticDeclination;
    boost::shared_ptr<DatabaseManager> m_databaseManager;
    std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> m_ekfCallbackImuOnly;
    std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> m_ekfCallbackWithGps;

private:
    FRIEND_TEST(IMUManagerTest, IsInvalidRangeReturnsTrue);
    FRIEND_TEST(IMUManagerTest, IsInvalidRangeReturnsFalse);
    FRIEND_TEST(IMUManagerTest, GetLatestGpsReturnsNullopt);
    FRIEND_TEST(IMUManagerTest, BuildGpsMeasurementVectorReturnsVector);
    FRIEND_TEST(IMUManagerTest, ValidateImuEventLinearAccelerationReturnsTrue);
};

#endif