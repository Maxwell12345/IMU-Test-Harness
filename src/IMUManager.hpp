#pragma once

#include <chrono>
#include <optional>
#include <functional>

#include <Eigen/Dense> 
#include <boost/shared_ptr.hpp>

#include "utils.hpp"
#include "GpsUpdate.hpp"
#include "IMUManagerStats.hpp"

extern "C" {
#include <sh2.h>
#include <sh2_SensorValue.h>
#include "sh2_err.h"
}

class DatabaseManager;

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class IMUManager {
public:

    IMUManager(
        boost::shared_ptr<DatabaseManager> databaseManager,
        std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> ekfCallbackImuOnly,
        std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> ekfCallbackWithGps
    );

    ~IMUManager();

    IMUManagerStats GetStats() const;
    static std::optional<GpsUpdate> GetLatestGps();

    /**
     * @brief Method used to update IMUManager gps data
     *
     * @param update gps data to be updated to
     * 
     * @return
     *
     * @remarks Will not update if:
     *      - Incoming data is older than current GPS data
     *      - Older than 5 seconds from steady_clock::now()
     *      - Marked as update.valid == false
     */
    static void UpdateLatestGps(const GpsUpdate& update);

    static void SensorCallback(void* cookie, sh2_SensorEvent_t* event);

private:

    /**
     * @brief Checks if a number is out of numerical bounds
     *
     * @param x number to be checked
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
     * @param sensorValue imu sensor value containing sensor type and measurement data
     *
     * @return True if the sensor event contains usable IMU data
     */
    static bool ValidateImuEvent(const sh2_SensorValue_t& sensorValue);

    /**
     * @brief Build an Eigen vector representation of GpsUpdate data
     * 
     * @param gps gps data struct
     *
     * @return Vector6d EKF-ready GPS measurement vector [x, y, 0, 0, 0, 0]^T in local coordinates.
     */
    static Vector6d BuildGpsMeasurementVector(const GpsUpdate& gps);

    /**
     * @brief Build an Eigen vector representation of SensorValue data
     * 
     * @param sensorValue imu sensor value containing sensor type and measurement data
     *
     * @return Vector6d EKF-ready IMU measurement vector [0, 0, vx, vy, ax, ay]^T in the navigation frame.
     */
    static Vector6d BuildImuMeasurementVector(const sh2_SensorValue_t& sensorValue);
 
    static bool s_gpsSentToEkf;
    static std::mutex s_gpsMutex;
    static IMUManager* s_instance;
    static std::optional<GpsUpdate> s_latestGps;
    static IMUUtils::KineticState s_kineticState;
    static std::chrono::steady_clock::time_point s_lastImuTimestamp;
    
    IMUManagerStats m_stats;
    mutable std::mutex m_statsMutex;
    boost::shared_ptr<DatabaseManager> m_databaseManager;
    std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> m_ekfCallbackImuOnly;
    std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> m_ekfCallbackWithGps;
};
