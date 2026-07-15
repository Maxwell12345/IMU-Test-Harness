#include "RadarPositionNavigationController.hpp"
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <thread>

RadarPositionNavigationController::RadarPositionNavigationController(const _KalmanValues& config,
                                                                     std::shared_ptr<DatabaseManager> databaseManager,
                                                                     std::unique_ptr<IMUSerialPortReader> imuSerialPortReader,
                                                                     std::unique_ptr<IMUManager> imuManager):
                                                                     m_config(config),
                                                                     m_databaseManager(std::move(databaseManager)),
                                                                     m_running(false),
                                                                     m_isKFConfigured(false),
                                                                     m_latestX(Vector6d::Zero()),
                                                                     m_latestP(Matrix6d::Zero()),
                                                                     m_imuManager(std::move(imuManager)),
                                                                     m_imuSerialPortReader(std::move(imuSerialPortReader)) {
    auto imuSerialCallback = [&imuManager = this->m_imuManager](std::optional<Raw_RotationVectorWAcc> optRv,
                                                                std::optional<Raw_Accelerometer> optLa,
                                                                std::optional<Raw_RotationRate> optRr){
        imuManager->SensorCallback(optRv, optLa, optRr);
    };
    this->m_imuSerialPortReader->InstallCallback(imuSerialCallback);

    this->m_lastUTMZone = -1;
}

RadarPositionNavigationController::~RadarPositionNavigationController() {
    this->TotalDestruction();
}

std::function<void(const GpsUpdate &)> RadarPositionNavigationController::GetGPSCallback() {
    return [this](const GpsUpdate &gpsUpdate) { this->_GPSCallback(gpsUpdate); };
}

void RadarPositionNavigationController::StartAndConfigureRadarPNT(double lat0, double lon0) {
    this->StopRadarPNT();

    if (!this->m_isKFConfigured) {
        this->ConfigureKalmanFilter(lat0,
                                    lon0,
                                    m_config.gpsChiSqLowerBound,
                                    m_config.gpsChiSqUpperBound,
                                    m_config.imuChiSqLowerBound,
                                    m_config.imuChiSqUpperBound);

        this->m_isKFConfigured = true;
    }

    m_imuManager->InstallEkf([this](double dt, Eigen::Matrix<double, 2, 1> &imuVec) { this->KFCallbackImuOnly(dt, imuVec); },
                           [this](double dt, Eigen::Matrix<double, 2, 1> &imuVec, Eigen::Matrix<double, 2, 1> &gpsVec) { this->KFCallbackWithGps(dt, imuVec, gpsVec); });

    this->StartIMUReader();
    this->m_running = true;
}

void RadarPositionNavigationController::StartIMUReader() {
    this->m_imuSerialPortReader->Start();
}

void RadarPositionNavigationController::StopRadarPNT() {
    this->m_imuSerialPortReader->Stop();
    this->m_isKFConfigured = false;
    this->m_running = false;
}

bool RadarPositionNavigationController::IsRunning() const {
    return m_running;
}

void RadarPositionNavigationController::ConfigureKalmanFilter(double lat0, double lon0, double gpsLowerPercentile,
                                                              double gpsUpperPercentile, double imuLowerPercentile,
                                                              double imuUpperPercentile) {
    Vector6d x0;
    x0 << lon0, lat0, 1e-15, 1e-15, 1e-16, 1e-16;

    Matrix6d P0 = Matrix6d::Zero();
    P0(0, 0) = 20.0001;
    P0(1, 1) = 20;
    P0(2, 2) = 12.0001;
    P0(3, 3) = 12;
    P0(4, 4) = 9;
    P0(5, 5) = 9.0001;

    Eigen::Matrix<double, 2, 2> R0_GPS = Eigen::Matrix<double, 2, 2>::Zero();
    R0_GPS(0, 0) = 3;
    R0_GPS(1, 1) = 3.0001;

    Eigen::Matrix<double, 2, 2> R0_IMU = Eigen::Matrix<double, 2, 2>::Zero();
    R0_IMU(0, 0) = 4.0001;
    R0_IMU(1, 1) = 4;

    Matrix6d Q0 = Matrix6d::Zero();
    Q0(0, 0) = 1.0001;
    Q0(1, 1) = 1;
    Q0(2, 2) = 2.0001;
    Q0(3, 3) = 2;
    Q0(4, 4) = 3;
    Q0(5, 5) = 3.0001;

    this->m_latestX = x0;
    this->m_latestP = P0;

    auto checkPercentileBounds = [](double percentile) { return percentile <= 0.0 || percentile >= 1.0; };

    // Calculate Chi SQ stats for df 2 and 4 at percentiles
    if (checkPercentileBounds(gpsLowerPercentile) ||
        checkPercentileBounds(gpsUpperPercentile) ||
        checkPercentileBounds(imuLowerPercentile) ||
        checkPercentileBounds(imuUpperPercentile)) {
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

    double chiSquaredBetaLowerBound_IMU = boost::math::quantile(boost::math::chi_squared(2), imuLowerPercentile);
    double chiSquaredBetaUpperBound_IMU = boost::math::quantile(boost::math::chi_squared(2), imuUpperPercentile);

    this->m_kf = IMUGPSFusionKF_2D_ConstantAcceleration(x0,
                                                        P0,
                                                        R0_GPS,
                                                        R0_IMU,
                                                        Q0,
                                                        chiSquaredBetaLowerBound_GPS,
                                                        chiSquaredBetaLowerBound_IMU,
                                                        chiSquaredBetaUpperBound_GPS,
                                                        chiSquaredBetaUpperBound_IMU,
                                                        m_config.gpsN,
                                                        m_config.gpsL,
                                                        m_config.imuN,
                                                        m_config.imuL,
                                                        m_config.qN,
                                                        m_config.qL);
}

void RadarPositionNavigationController::KFCallbackImuOnly(double dt, Eigen::Matrix<double, 2, 1> &imuVec) {
    bool needsReconfig = false;
    double reconfigLat = 0.0;
    double reconfigLon = 0.0;
    std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);
    {
        if (!this->m_isKFConfigured.load() || this->m_lastUTMZone == -1) {
            return;
        }

        try {
            if (dt <= 0 || dt > 0.5) {
                dt = 0.01;
            }
            std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, imuVec);

            this->m_latestX = output.first;
            this->m_latestP = output.second;

            this->m_databaseManager->EnqueueEkfOutput(m_latestX, m_latestP);
        } catch (const std::exception &e) {
            // TODO: Log this
            std::cout << "[ERROR] " << e.what() << std::endl;
            if (this->m_isKFConfigured) {
                double lat = this->m_latestX(1, 0);
                double lon = this->m_latestX(0, 0);

                if (!std::isfinite(lat) || !std::isfinite(lon)) {
                    this->m_isKFConfigured.store(false);
                    return;
                }
                // note: original was lat0, lon0
                needsReconfig = true;
                reconfigLat = lat;
                reconfigLon = lon;
            }
        }
    }

    if (needsReconfig) {
        this->ConfigureKalmanFilter(reconfigLat,
                                    reconfigLon,
                                    m_config.gpsChiSqLowerBound,
                                    m_config.gpsChiSqUpperBound,
                                    m_config.imuChiSqLowerBound,
                                    m_config.imuChiSqUpperBound);
    }
}

void RadarPositionNavigationController::KFCallbackWithGps(double dt, Eigen::Matrix<double, 2, 1> &imuVec, Eigen::Matrix<double, 2, 1> &gpsVec) {
    bool needsReconfig = false;
    double reconfigLat = 0.0;
    double reconfigLon = 0.0;
    {
        std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);

        if (!this->m_isKFConfigured.load()) {
            return;
        }

        try {
            if (dt <= 0 || dt > 0.5) {
                dt = 0.01;
            }

            auto utmGps = IMUUtils::WGS84_to_UTM(gpsVec(1, 0), gpsVec(0, 0));

            gpsVec(0, 0) = utmGps[0];
            gpsVec(1, 0) = utmGps[1];

            int zone = utmGps[2];

            if (this->m_lastUTMZone != zone && this->m_lastUTMZone != -1) {
                this->m_kf.UpdatePosition(utmGps[0], utmGps[1]);
            }

            this->m_lastUTMZone = zone;

            std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, gpsVec, imuVec);

            // Check if the KF has surpassed the 
            auto wgs84 = IMUUtils::UTM_to_WGS84(output.first(0, 0), output.first(1, 0), zone, utmGps[3]);
            auto utm = IMUUtils::WGS84_to_UTM(wgs84[0], [1]);

            zone = utm[2];

            if (this->m_lastUTMZone != zone) {
                this->m_kf.UpdatePosition(utm[0], utm[1]);
            }

            this->m_lastUTMZone = zone;

            this->m_latestX = output.first;
            this->m_latestP = output.second;

            std::cout << m_latestX << std::endl;

            this->m_databaseManager->EnqueueEkfOutput(m_latestX, m_latestP);
        } catch (const std::exception &e) {
            // TODO: Log this
            std::cout << "[ERROR] " << e.what() << std::endl;
            if (this->m_isKFConfigured) {
                double lat = this->m_latestX(1, 0);
                double lon = this->m_latestX(0, 0);

                if (!std::isfinite(lat) || !std::isfinite(lon)) {
                this->m_isKFConfigured.store(false);
                return;
                }
                // note: original was lat0, lon0
                needsReconfig = true;
                reconfigLat = lat;
                reconfigLon = lon;
            }
        }
    }
    if (needsReconfig) {
        this->ConfigureKalmanFilter(reconfigLat,
                                    reconfigLon,
                                    m_config.gpsChiSqLowerBound,
                                    m_config.gpsChiSqUpperBound,
                                    m_config.imuChiSqLowerBound,
                                    m_config.imuChiSqUpperBound);
    }
}

void RadarPositionNavigationController::_GPSCallback(const GpsUpdate &gpsUpdate) {
    m_imuManager->UpdateLatestGps(gpsUpdate);
}

void RadarPositionNavigationController::TotalDestruction() {
    this->StopRadarPNT();

    std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);

    this->m_kf.Clean();
    this->m_isKFConfigured = false;

    this->m_latestX = Vector6d::Zero();
    this->m_latestP = Matrix6d::Zero();
}
