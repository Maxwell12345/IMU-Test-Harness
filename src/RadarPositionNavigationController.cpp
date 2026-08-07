/******************************************************************************
 * File:             RadarPositionNavigationController.cpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Coordinates GPS and IMU measurements, maintains the local
 *                   ENU origin, and executes Kalman-filter updates.
 *
 ******************************************************************************/

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
                                                                     m_latestX(Vector6d::Zero()),
                                                                     m_latestP(Matrix6d::Zero()),
                                                                     m_config(config),
                                                                     m_running(false),
                                                                     m_isKFConfigured(false),
                                                                     m_hasOrigin(false),
                                                                     m_originLatLon({0.0, 0.0}),
                                                                     m_originEN({0.0, 0.0}),
                                                                     m_imuManager(std::move(imuManager)),
                                                                     m_imuSerialPortReader(std::move(imuSerialPortReader)),
                                                                     m_databaseManager(std::move(databaseManager)) {
    auto imuSerialCallback = [&imuManager = this->m_imuManager](std::optional<Raw_RotationVectorWAcc> optRv,
                                                                std::optional<Raw_Accelerometer> optLa,
                                                                std::optional<Raw_RotationRate> optRr){
        imuManager->SensorCallback(optRv, optLa, optRr);
    };

    if (this->m_imuSerialPortReader) {
        this->m_imuSerialPortReader->InstallCallback(imuSerialCallback);
    }
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
    if (this->m_imuSerialPortReader) {
        this->m_imuSerialPortReader->Start();
    }
}

void RadarPositionNavigationController::StopRadarPNT() {
    if (this->m_imuSerialPortReader) {
        this->m_imuSerialPortReader->Stop();
    }
    this->m_isKFConfigured = false;
    this->m_running = false;
}

bool RadarPositionNavigationController::IsRunning() const {
    return m_running;
}

Vector6d RadarPositionNavigationController::GetKFState() const {
    std::lock_guard<std::mutex> kfStateGuard(this->m_kFUpdateMutex);
    return this->m_latestX;
}

Matrix6d RadarPositionNavigationController::GetKFCovariance() const {
    std::lock_guard<std::mutex> kfStateGuard(this->m_kFUpdateMutex);
    return this->m_latestP;
}

void RadarPositionNavigationController::ConfigureKalmanFilter(double initialLatitude, double initialLongitude) {
    this->m_originLatLon = {initialLongitude, initialLatitude};
    this->m_originEN = {0.0, 0.0};
    this->m_hasOrigin = true;

    Vector6d initialState;
    initialState << 0.0, 0.0, 1e-15, 1e-15, 1e-16, 1e-16;

    Matrix6d P0 = Matrix6d::Zero();
    P0.diagonal() << 25.0, 25.0, 0.59, 1.0, 0.4, 0.5;

    Eigen::Matrix<double, 4, 4> R0 = Eigen::Matrix<double, 4, 4>::Zero();
    R0.diagonal() << 0.05000000001, 0.05, 0.009001, 0.01;

    this->m_latestX = initialState;
    this->m_latestP = P0;

    this->m_kf = IMUGPSFusionKF(initialState, P0, R0);
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
            if (!std::isfinite(dt) || dt <= 0.0) {
                return;
            }
            std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, imuVec);

            this->ValidateAndUpdateENUOrigin(output.first);

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
            if (!std::isfinite(dt) || dt <= 0.0) {
                return;
            }

            double easting;
            double northing;
            double previousOriginLon = this->m_originLatLon.first;
            double previousOriginLat = this->m_originLatLon.second;
            bool isENUReset = this->ConvertGPSToENU(easting,
                                                    northing,
                                                    gpsVec(1, 0),
                                                    gpsVec(0, 0));

            if (isENUReset) {
                this->RestKFOrigin(previousOriginLat, previousOriginLon);
            }

            gpsVec << easting, northing;

            std::pair<Vector6d, Matrix6d> output = this->m_kf.Step(dt, gpsVec, imuVec);

            this->ValidateAndUpdateENUOrigin(output.first);

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

bool RadarPositionNavigationController::ConvertGPSToENU(double& easting,
                                                        double& northing,
                                                        double latitude,
                                                        double longitude) {
    if (!this->m_hasOrigin) {
        this->m_originLatLon = {longitude, latitude};
    }

    double up;
    IMUUtils::WGS84_To_ENU(this->m_originLatLon.first,
                           this->m_originLatLon.second,
                           DEFAULT_RADAR_HEIGHT_METERS,
                           longitude,
                           latitude,
                           DEFAULT_RADAR_HEIGHT_METERS,
                           easting,
                           northing,
                           up);

    if (!this->m_hasOrigin) {
        this->m_originEN = {easting, northing};
        this->m_hasOrigin = true;
    }

    if (std::hypot(easting - this->m_originEN.first,
                   northing - this->m_originEN.second) > MAX_ALLOWED_ENU_DEVIATION_FROM_ORIGIN_METERS) {
        this->m_hasOrigin = false;
        this->ConvertGPSToENU(easting, northing, latitude, longitude);

        return true;
    }

    return false;
}

bool RadarPositionNavigationController::ValidateAndUpdateENUOrigin(Eigen::Matrix<double, 6, 1> &state) {
    if (!this->m_hasOrigin) {
        throw std::runtime_error("No origin available and attempting to update ENU origin off of KF.");
    }

    double easting = state(0, 0);
    double northing = state(1, 0);

    if (std::hypot(easting - this->m_originEN.first,
                   northing - this->m_originEN.second) > MAX_ALLOWED_ENU_DEVIATION_FROM_ORIGIN_METERS) {
        const double oldLongitude = this->m_originLatLon.first;
        const double oldLatitude = this->m_originLatLon.second;
        double latitude;
        double longitude;

        this->ConvertKFStateToWGS84(latitude, longitude, state);

        this->m_originLatLon = {longitude, latitude};
        this->m_originEN = {0.0, 0.0};

        constexpr double degreesToRadians = 0.01745329251994329576923690768489;
        const double oldLatitudeRadians = oldLatitude * degreesToRadians;
        const double oldLongitudeRadians = oldLongitude * degreesToRadians;
        const double newLatitudeRadians = latitude * degreesToRadians;
        const double newLongitudeRadians = longitude * degreesToRadians;
        const double oldHeading = state(2);
        const double longitudeDelta = newLongitudeRadians - oldLongitudeRadians;
        const double newHeading = std::atan2(
            -std::sin(newLatitudeRadians) * std::sin(longitudeDelta) * std::cos(oldHeading) +
                (std::sin(newLatitudeRadians) * std::sin(oldLatitudeRadians) * std::cos(longitudeDelta) +
                 std::cos(newLatitudeRadians) * std::cos(oldLatitudeRadians)) * std::sin(oldHeading),
            std::cos(longitudeDelta) * std::cos(oldHeading) +
                std::sin(oldLatitudeRadians) * std::sin(longitudeDelta) * std::sin(oldHeading));

        state(0) = 0.0;
        state(1) = 0.0;
        state(2) = newHeading;

        this->m_kf.UpdatePosition(0.0,
                                  0.0,
                                  oldLatitude,
                                  oldLongitude,
                                  latitude,
                                  longitude);

        return true;
    }

    return false;
}

void RadarPositionNavigationController::ConvertKFStateToWGS84(double& latitude,
                                                              double& longitude,
                                                              Eigen::Matrix<double, 6, 1> &state) {
    double easting = state(0, 0);
    double northing = state(1, 0);

    double altitude;
    IMUUtils::ENU_To_WGS84(easting,
                           northing,
                           DEFAULT_RADAR_HEIGHT_METERS,
                           this->m_originLatLon.first,
                           this->m_originLatLon.second,
                           DEFAULT_RADAR_HEIGHT_METERS,
                           latitude,
                           longitude,
                           altitude);
}

void RadarPositionNavigationController::RestKFOrigin(double oldLatOrigin, double oldLonOrigin) {
    double kfLat;
    double kfLon;
    double altitude;
    IMUUtils::ENU_To_WGS84(this->m_latestX(0),
                           this->m_latestX(1),
                           DEFAULT_RADAR_HEIGHT_METERS,
                           oldLonOrigin,
                           oldLatOrigin,
                           DEFAULT_RADAR_HEIGHT_METERS,
                           kfLat,
                           kfLon,
                           altitude);

    double up;
    double easting;
    double northing;
    IMUUtils::WGS84_To_ENU(this->m_originLatLon.first,
                           this->m_originLatLon.second,
                           DEFAULT_RADAR_HEIGHT_METERS,
                           kfLon,
                           kfLat,
                           DEFAULT_RADAR_HEIGHT_METERS,
                           easting,
                           northing,
                           up);

    this->m_kf.UpdatePosition(easting,
                              northing,
                              oldLatOrigin,
                              oldLonOrigin,
                              this->m_originLatLon.second,
                              this->m_originLatLon.first);

    this->m_latestX(0) = easting;
    this->m_latestX(1) = northing;
}

