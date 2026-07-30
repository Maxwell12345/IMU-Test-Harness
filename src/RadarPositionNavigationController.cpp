#include "RadarPositionNavigationController.hpp"
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <thread>

#define DEFAULT_RADAR_HEIGHT_METERS 3
#define MAX_ALLOWED_ENU_DEVIATION_FROM_ORIGIN_METERS 100

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
        this->ConfigureKalmanFilter(lat0, lon0);

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

void RadarPositionNavigationController::ConfigureKalmanFilter(double lat0, double lon0) {
    Vector6d x0;
    x0 << lon0, lat0, 1e-15, 1e-15, 1e-16, 1e-16;

    Matrix6d P0 = Matrix6d::Zero();
    P0.diagonal() << 25.0, 25.0, 0.59, 1.0, 0.4, 0.5;

    Eigen::Matrix<double, 4, 4> R0 = Eigen::Matrix<double, 4, 4>::Zero();
    R0.diagonal() << 0.05000000001, 0.05, 0.009001, 0.01;

    this->m_latestX = x0;
    this->m_latestP = P0;

    this->m_kf = IMUGPSFusionKF(x0, P0, R0);
}

void RadarPositionNavigationController::KFCallbackImuOnly(double dt, Eigen::Matrix<double, 2, 1> &imuVec) {
    bool needsReconfig = false;
    double reconfigLat = 0.0;
    double reconfigLon = 0.0;
    std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);
    {
        if (!this->m_isKFConfigured) {
            return;
        }

        try {
            if (dt <= 0 || dt > 1.0) {
                dt = 0.01;
            }
            std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, imuVec);

            const double previousOriginLon = this->m_originLatLon.first;
            const double previousOriginLat = this->m_originLatLon.second;
            const bool isENUReset = this->ValidateAndUpdateENUOrigin(output.first);
            
            if (isENUReset) {
                this->RestKFOrigin(previousOriginLat, previousOriginLon);
            }

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
                    this->m_isKFConfigured = false;
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
        this->ConfigureKalmanFilter(reconfigLat, reconfigLon);
    }
}

void RadarPositionNavigationController::KFCallbackWithGps(double dt, Eigen::Matrix<double, 2, 1> &imuVec, Eigen::Matrix<double, 2, 1> &gpsVec) {
    bool needsReconfig = false;
    double reconfigLat = 0.0;
    double reconfigLon = 0.0;
    {
        std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);

        if (!this->m_isKFConfigured) {
            return;
        }

        try {
            if (dt <= 0 || dt > 1.0) {
                dt = 0.01;
            }

            double E;
            double N;
            double previousOriginLon = this->m_originLatLon.first;
            double previousOriginLat = this->m_originLatLon.second;
            bool isENUReset = this->ConvertGPSToENU(E, N, gpsVec(0, 0), gpsVec(1, 0));

            if (isENUReset) {
                this->RestKFOrigin(previousOriginLat, previousOriginLon);
            }

            gpsVec << E, N;

            std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, gpsVec, imuVec);

            previousOriginLon = this->m_originLatLon.first;
            previousOriginLat = this->m_originLatLon.second;
            isENUReset = this->ValidateAndUpdateENUOrigin(output.first);
            
            if (isENUReset) {
                this->RestKFOrigin(previousOriginLat, previousOriginLon);
            }

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
                this->m_isKFConfigured = false;
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
        this->ConfigureKalmanFilter(reconfigLat, reconfigLon);
    }
}

void RadarPositionNavigationController::_GPSCallback(const GpsUpdate &gpsUpdate) {
    m_imuManager->UpdateLatestGps(gpsUpdate);
}

void RadarPositionNavigationController::TotalDestruction() {
    this->StopRadarPNT();

    std::lock_guard<std::mutex> kfStepGuard(this->m_kFUpdateMutex);

    this->m_isKFConfigured = false;

    this->m_latestX = Vector6d::Zero();
    this->m_latestP = Matrix6d::Zero();
}

bool RadarPositionNavigationController::ConvertGPSToENU(double& E, double& N, double lat, double lon) {
    if (!this->m_hasOrigin) {
        this->m_originLatLon = {lon, lat};
    }

    double up;
    IMUUtils::WGS84_To_ENU(this->m_originLatLon.first, this->m_originLatLon.second, DEFAULT_RADAR_HEIGHT_METERS, lon, lat, DEFAULT_RADAR_HEIGHT_METERS, E, N, up);

    if (!this->m_hasOrigin) {
        this->m_originEN = {E, N};
        this->m_hasOrigin = true;
    }

    if (std::hypot(E - this->m_originEN.first, N - this->m_originEN.second) > MAX_ALLOWED_ENU_DEVIATION_FROM_ORIGIN_METERS) {
        this->m_hasOrigin = false;
        this->ConvertGPSToENU(E, N, lat, lon);

        return true;
    }

    return false;
}

bool RadarPositionNavigationController::ValidateAndUpdateENUOrigin(Eigen::Matrix<double, 6, 1> &x) {
    if (!this->m_hasOrigin) {
        throw std::runtime_error("No origin available and attempting to update ENU origin off of KF.");
    }

    double E = x(0, 0);
    double N = x(1, 0);

    if (std::hypot(E - this->m_originEN.first, N - this->m_originEN.second) > MAX_ALLOWED_ENU_DEVIATION_FROM_ORIGIN_METERS) {
        double lat;
        double lon;

        this->ConvertKFStateToWGS84(lat, lon, x);

        this->m_originLatLon = {lon, lat};
        
        double up;
        IMUUtils::WGS84_To_ENU(this->m_originLatLon.first, this->m_originLatLon.second, DEFAULT_RADAR_HEIGHT_METERS, lon, lat, DEFAULT_RADAR_HEIGHT_METERS, E, N, up);

        this->m_originEN = {E, N};

        return true;
    }

    return false;
}

void RadarPositionNavigationController::ConvertKFStateToWGS84(double& lat, double& lon, Eigen::Matrix<double, 6, 1> &x) {
    double E = x(0, 0);
    double N = x(1, 0);

    double _;
    IMUUtils::ENU_To_WGS84(E, N, DEFAULT_RADAR_HEIGHT_METERS, this->m_originLatLon.first, this->m_originLatLon.second, DEFAULT_RADAR_HEIGHT_METERS, lat, lon, _);
}

void RadarPositionNavigationController::RestKFOrigin(double oldLatOrigin, double oldLonOrigin) {
    double kfLat;
    double kfLon;
    this->ConvertKFStateToWGS84(kfLat, kfLon, this->m_latestX);

    double up;
    double E;
    double N;
    IMUUtils::WGS84_To_ENU(this->m_originLatLon.first, this->m_originLatLon.second, DEFAULT_RADAR_HEIGHT_METERS, kfLon, kfLat, DEFAULT_RADAR_HEIGHT_METERS, E, N, up);

    this->m_kf.UpdatePosition(E, N, oldLatOrigin, oldLonOrigin, this->m_originLatLon.second, this->m_originLatLon.first);
}

