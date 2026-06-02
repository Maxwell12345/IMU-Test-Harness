#pragma once

#include <chrono>
#include <functional>

#include <Eigen/Dense> 
#include <boost/shared_ptr.hpp>

#include "IMUManagerStats.hpp"
#include "GpsUpdate.hpp"

extern "C" {
#include <sh2.h>
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

    static void SensorCallback(void* cookie, sh2_SensorEvent_t* event);
    static void UpdateLatestGps(const GpsUpdate& udpate);
    static std::optional<GpsUpdate> GetLatestGps();
    IMUManagerStats GetStats() const;

private:
    static Vector6d buildImuMeasurementVector(const sh2_SensorEvent_t* event);
    static Vector6d buildGpsMeasurementVector(const GpsUpdate& gps);
    static bool validateImuEvent(const sh2_SensorEvent_t* event);

    static IMUManager* s_instance;
    static std::optional<GpsUpdate> s_latesetGps;
    static bool s_gpsSentToEkf;
    static std::mutex s_gpsMutex;
    static std::chrono::steady_clock::time_point s_lastImuTimestamp;
    
    boost::shared_ptr<DatabaseManager> m_databaseManager;
    std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&)> m_ekfCallbackImuOnly;
    std::function<std::pair<Vector6d, Matrix6d>(double, Vector6d&, Vector6d&)> m_ekfCallbackWithGps;
    mutable std::mutex m_statsMutex;
    IMUManagerStats m_stats;
};
