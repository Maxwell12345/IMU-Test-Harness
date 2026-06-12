#include <boost/math/distributions/chi_squared.hpp>
#include <stdexcept>
#include <thread>
#include <cmath>

#include "RadarPositionNavigationController.hpp"

#define GPS_CHI_SQ_LOWER_BOUND 0.20
#define GPS_CHI_SQ_UPPER_BOUND 0.95
#define IMU_CHI_SQ_LOWER_BOUND 0.20
#define IMU_CHI_SQ_UPPER_BOUND 0.95
#define GPS_N 20
#define GPS_L 5
#define IMU_N 100
#define IMU_L 10
#define Q_N 100
#define Q_L 10

static void enable_sensor(sh2_SensorId_t sensor_id, uint32_t interval_us) {
    sh2_SensorConfig_t cfg{};
    cfg.reportInterval_us = interval_us;

    if (sh2_setSensorConfig(sensor_id, &cfg) != SH2_OK) {
        // TODO: Log this
        std::cerr << "[WARN] Failed to enable sensor id=" << sensor_id << "\n";
    }
}

RadarPositionNavigationController::RadarPositionNavigationController():
    m_databaseManager(boost::make_shared<DatabaseManager>("./IMUPROC.db")),
    m_imuManager(m_databaseManager) {
    this->m_sh2ServiceIsRunning = false;
    this->m_isKFConfigured = false;

    this->m_latestX = Vector6d::Zero();
    this->m_latestP = Matrix6d::Zero();
}

RadarPositionNavigationController::~RadarPositionNavigationController() {
    this->TotalDestruction();
}

std::function<void(const GpsUpdate&)> RadarPositionNavigationController::GetGPSCallback() {
    return [this](const GpsUpdate& gpsUpdate) {
        this->_GPSCallback(gpsUpdate);
    };
}

void RadarPositionNavigationController::StartAndConfigureRadarPNT(double lat0, double lon0) {
    if (!this->m_isKFConfigured) {
        this->ConfigureKalmanFilter(lat0, lon0, GPS_CHI_SQ_LOWER_BOUND, GPS_CHI_SQ_UPPER_BOUND, IMU_CHI_SQ_LOWER_BOUND, IMU_CHI_SQ_UPPER_BOUND);

        this->m_isKFConfigured = true;
    }

    IMUManager::InstallEkf(
        [this](double dt, Vector6d& imuVec) {this->KFCallbackImuOnly(dt, imuVec);},
        [this](double dt, Vector6d& imuVec, Vector6d& gpsVec) {this->KFCallbackWithGps(dt, imuVec, gpsVec);}
    );

    StartIMUReader();
}

void RadarPositionNavigationController::StopRadarPNT() {
    this->m_sh2ServiceIsRunning.store(false);

    if (this->m_serviceThread.joinable()) {
        this->m_serviceThread.join();
    }

    if (this->m_sh2IsOpen.exchange(false)) {
        sh2_close();
    }
}

void RadarPositionNavigationController::StartIMUReader() {
    if (this->m_sh2ServiceIsRunning) {
        return;
    }

    this->m_hal = bno085_hal_create();

    if (sh2_open(&this->m_hal, nullptr, nullptr) != SH2_OK) {
        std::cerr << "[ERROR] sh2_open failed\n";
        return;
    }

    this->m_sh2IsOpen.store(true);

    sh2_setSensorCallback(IMUManager::SensorCallback, nullptr);

    enable_sensor(SH2_LINEAR_ACCELERATION, 2500);
    enable_sensor(SH2_ROTATION_VECTOR, 2500);

    this->m_sh2ServiceIsRunning.store(true);

    this->m_serviceThread = std::thread([this]() {
        while (this->m_sh2ServiceIsRunning.load()) {
            sh2_service();
        }
    });
}

void RadarPositionNavigationController::ConfigureKalmanFilter(double lat0, double lon0, double gpsLowerPercentile, double gpsUpperPercentile, double imuLowerPercentile, double imuUpperPercentile) {
    Vector6d x0;
    x0 << lon0, lat0, 1e-15, 1e-15, 1e-16, 1e-16;

    Matrix6d P0 = Matrix6d::Zero();
    P0(0, 0) = 1e-10;
    P0(1, 1) = 1e-10;
    P0(2, 2) = 1e-8;
    P0(3, 3) = 1e-8;
    P0(4, 4) = 1e-10;
    P0(5, 5) = 1e-10;

    Matrix6d R0_GPS = Matrix6d::Zero();
    R0_GPS(0, 0) = 1e-10;
    R0_GPS(1, 1) = 1e-10;

    Matrix6d R0_IMU = Matrix6d::Zero();
    R0_IMU(2, 2) = 1e-8;
    R0_IMU(3, 3) = 1e-8;
    R0_IMU(4, 4) = 1e-10;
    R0_IMU(5, 5) = 1e-10;

    Matrix6d Q0 = Matrix6d::Zero();
    Q0(0, 0) = 1e-14;
    Q0(1, 1) = 1e-14;
    Q0(2, 2) = 1e-12;
    Q0(3, 3) = 1e-12;
    Q0(4, 4) = 1e-14;
    Q0(5, 5) = 1e-14;

    this->m_latestX = x0;
    this->m_latestP = P0;

    auto checkPercentileBounds = [](double percentile) {
        return percentile <= 0.0 || percentile >= 1.0;
    };

    // Calculate Chi SQ stats for df 2 and 4 at percentiles
    if (
        checkPercentileBounds(gpsLowerPercentile) || 
        checkPercentileBounds(gpsUpperPercentile) ||
        checkPercentileBounds(imuLowerPercentile) ||
        checkPercentileBounds(imuUpperPercentile) 
    ) {
        throw std::runtime_error("One or more Fuzzy fusion Chi SQ percentiles are <= 0 and/or >= 1");
    }

    if (gpsLowerPercentile >= gpsUpperPercentile) {
        throw std::runtime_error("GPS Chi SQ lower percentile is >= upper percentile");
    }

    if (imuLowerPercentile >= imuUpperPercentile) {
        throw std::runtime_error("IMU Chi SQ lower percentile is >= upper percentile");
    }

    double chiSquaredBetaLowerBound_GPS = boost::math::quantile(boost::math::chi_squared(2), gpsLowerPercentile); 
    double chiSquaredBetaUpperBound_GPS = boost::math::quantile(boost::math::chi_squared(2), gpsUpperPercentile); 

    double chiSquaredBetaLowerBound_IMU = boost::math::quantile(boost::math::chi_squared(4), imuLowerPercentile); 
    double chiSquaredBetaUpperBound_IMU = boost::math::quantile(boost::math::chi_squared(4), imuUpperPercentile); 

    this->m_kf = IMUGPSFusionKF_2D_ConstantAcceleration(
        x0,
        P0,
        R0_GPS,
        R0_IMU,
        Q0,
        chiSquaredBetaLowerBound_GPS,
        chiSquaredBetaLowerBound_IMU,
        chiSquaredBetaUpperBound_GPS,
        chiSquaredBetaUpperBound_IMU,
        GPS_N,
        GPS_L,
        IMU_N,
        IMU_L,
        Q_N,
        Q_L,
        m_databaseManager
    );
}

void RadarPositionNavigationController::KFCallbackImuOnly(double dt, Vector6d& imuVec) {
    std::lock_guard<std::mutex> kfStepGuard(this->m_sKFUpdateMutex);

    if (!this->m_isKFConfigured.load()) {
        return;
    }

    try {
        std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, imuVec);

        this->m_latestX = output.first;
        this->m_latestP = output.second;
    }
    catch(const std::exception& e) {
        // TODO: Log this
        std::cout << "[ERROR] " << e.what() << std::endl;
        if (this->m_isKFConfigured) {
            double lat = this->m_latestX(1, 0);
            double lon = this->m_latestX(0, 0);

            if (!std::isfinite(lat) || !std::isfinite(lon)) {
                this->m_isKFConfigured.store(false);
                return;
            }

            this->ConfigureKalmanFilter(lat, lon, GPS_CHI_SQ_LOWER_BOUND, GPS_CHI_SQ_UPPER_BOUND, IMU_CHI_SQ_LOWER_BOUND, IMU_CHI_SQ_UPPER_BOUND);
        }
    }
}

void RadarPositionNavigationController::KFCallbackWithGps(double dt, Vector6d& imuVec, Vector6d& gpsVec) {
    std::lock_guard<std::mutex> kfStepGuard(this->m_sKFUpdateMutex);

    if (!this->m_isKFConfigured.load()) {
        return;
    }

    try {
        std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, gpsVec, imuVec);
        
        //TODO: Remove csv logging
        LogKFCSV(output.first, output.second, imuVec, &gpsVec);

        this->m_latestX = output.first;
        this->m_latestP = output.second;
    }
    catch(const std::exception& e) {
        // TODO: Log this
        std::cout << "[ERROR] " << e.what() << std::endl;
        if (this->m_isKFConfigured) {
            double lat = this->m_latestX(1, 0);
            double lon = this->m_latestX(0, 0);

            if (!std::isfinite(lat) || !std::isfinite(lon)) {
                this->m_isKFConfigured.store(false);
                return;
            }

            this->ConfigureKalmanFilter(lat, lon, GPS_CHI_SQ_LOWER_BOUND, GPS_CHI_SQ_UPPER_BOUND, IMU_CHI_SQ_LOWER_BOUND, IMU_CHI_SQ_UPPER_BOUND);
        }
    }
}

void RadarPositionNavigationController::_GPSCallback (const GpsUpdate& gpsUpdate) {
    IMUManager::UpdateLatestGps(gpsUpdate);
}

void RadarPositionNavigationController::TotalDestruction() {
    this->StopRadarPNT();

    std::lock_guard<std::mutex> kfStepGuard(this->m_sKFUpdateMutex);

    if (this->m_isKFConfigured.load()) {
        this->m_kf.Clean();
        this->m_isKFConfigured = false;
    }

    this->m_latestX = Vector6d::Zero();
    this->m_latestP = Matrix6d::Zero();
}