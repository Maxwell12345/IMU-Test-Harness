/******************************************************************************
 * File:             IMUGPSFusionKF.cpp
 *
 * Author:           Brian R. Atkinson
 * Organization:     Marine Corps Software Factory
 * Created On:       08/07/26
 * Description:      Implements ENU prediction and GPS/IMU covariance updates
 *                   for the vehicle navigation extended Kalman filter.
 *
 ******************************************************************************/

#include "IMUGPSFusionKF.hpp"
#include <string>

IMUGPSFusionKF::IMUGPSFusionKF(
    Eigen::Matrix<double, N_STATE, 1> x0, 
    Eigen::Matrix<double, N_STATE, N_STATE> P0, 
    Eigen::Matrix<double, M_Z, M_Z> R0
) {
    this->m_x = x0;
    this->m_P = P0;
    this->m_R = R0;

    this->m_H_GPS_IMU << 1, 0, 0, 0, 0, 0,
                         0, 1, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 1,
                         0, 0, 0, 0, 1, 0;

    this->m_H_IMU << 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 1,
                     0, 0, 0, 0, 1, 0;

    this->Update_Q(1.0 / 100.0, this->m_x);

    this->m_lastGPSDt = 0.0;

    Eigen::Matrix<double, 2, 2> IMUR;
    IMUR << R0(2, 2), R0(2, 3), R0(3, 2), R0(3, 3);

    this->m_covarianceEstimator = IMUCovarianceEstimator(IMUR, 500, 200);
}

Eigen::Matrix<double, N_STATE, N_STATE>
IMUGPSFusionKF::BuildFk(double dt,
                       Eigen::Matrix<double, N_STATE, 1> state,
                       Eigen::Matrix<double, 2, 1> imuControl) {
    const double measuredYawRate = imuControl(1, 0);

    const double heading = state(2);
    const double speed = state(3);

    const double dt2 = dt * dt;
    const double sinHeading = std::sin(heading);
    const double cosHeading = std::cos(heading);

    Eigen::Matrix<double, N_STATE, N_STATE> transitionJacobian =
        Eigen::Matrix<double, N_STATE, N_STATE>::Identity();

    transitionJacobian(2, 4) = dt;
    transitionJacobian(3, 5) = dt;

    constexpr double yawRateThreshold = 0.04;

    if (std::abs(measuredYawRate) < yawRateThreshold) {
        transitionJacobian(0, 2) = -speed * dt * sinHeading;
        transitionJacobian(0, 3) = dt * cosHeading;
        transitionJacobian(0, 4) = -0.5 * speed * dt2 * sinHeading;

        transitionJacobian(1, 2) = speed * dt * cosHeading;
        transitionJacobian(1, 3) = dt * sinHeading;
        transitionJacobian(1, 4) = 0.5 * speed * dt2 * cosHeading;

        return transitionJacobian;
    }

    const double newHeading = heading + measuredYawRate * dt;
    const double sinNewHeading = std::sin(newHeading);
    const double cosNewHeading = std::cos(newHeading);
    const double yawRateSquared = measuredYawRate * measuredYawRate;

    const double deltaSin = sinNewHeading - sinHeading;
    const double deltaCos = cosHeading - cosNewHeading;

    transitionJacobian(0, 2) = (speed / measuredYawRate) * (cosNewHeading - cosHeading);
    transitionJacobian(0, 3) = deltaSin / measuredYawRate;
    transitionJacobian(0, 4) = speed * (measuredYawRate * dt * cosNewHeading - deltaSin) /
                               yawRateSquared;

    transitionJacobian(1, 2) = (speed / measuredYawRate) * (sinNewHeading - sinHeading);
    transitionJacobian(1, 3) = deltaCos / measuredYawRate;
    transitionJacobian(1, 4) = speed * (measuredYawRate * dt * sinNewHeading - deltaCos) /
                               yawRateSquared;

    return transitionJacobian;
}

Eigen::Matrix<double, N_STATE, 1>
IMUGPSFusionKF::f(double dt,
                 Eigen::Matrix<double, N_STATE, 1> state,
                 Eigen::Matrix<double, 2, 1> imuControl) {
    const double measuredAcceleration = imuControl(0, 0);
    const double measuredYawRate = imuControl(1, 0);

    const double easting = state(0, 0);
    const double northing = state(1, 0);
    const double heading = state(2, 0);
    const double speed = state(3, 0);
    const double yawRate = state(4, 0);
    const double acceleration = state(5, 0);

    Eigen::Matrix<double, N_STATE, 1> updatedState;

    const double cosHeading = std::cos(heading);
    const double sinHeading = std::sin(heading);

    if (std::abs(measuredYawRate) < 0.04) {
        updatedState(0, 0) = easting + speed * dt * cosHeading +
                             0.5 * dt * dt * measuredAcceleration * cosHeading;
        updatedState(1, 0) = northing + speed * dt * sinHeading +
                             0.5 * dt * dt * measuredAcceleration * sinHeading;
        updatedState(2, 0) = heading + dt * measuredYawRate;
        updatedState(3, 0) = speed + dt * measuredAcceleration;
        updatedState(4, 0) = yawRate;
        updatedState(5, 0) = acceleration;
    }
    else {
        const double speedOverYawRate = speed / measuredYawRate;
        const double newHeading = heading + dt * measuredYawRate;
        updatedState(0, 0) = easting + speedOverYawRate * (std::sin(newHeading) - sinHeading);
        updatedState(1, 0) = northing + speedOverYawRate * (-std::cos(newHeading) + cosHeading);
        updatedState(2, 0) = newHeading;
        updatedState(3, 0) = speed + dt * measuredAcceleration;
        updatedState(4, 0) = yawRate;
        updatedState(5, 0) = acceleration;
    }

    return updatedState;
}

std::pair<Eigen::Matrix<double, N_STATE, 1>, Eigen::Matrix<double, N_STATE, N_STATE>> 
IMUGPSFusionKF::Step(double dt, Eigen::Matrix<double, 2, 1>& imuControl) {
    this->m_Q = this->Update_Q(dt, this->m_x);

    this->m_lastGPSDt += dt;

    auto imuMeasurementCovariance = this->m_covarianceEstimator.GetR(imuControl(0, 0), imuControl(1, 0));

    this->m_R(2, 2) = imuMeasurementCovariance(0, 0);
    this->m_R(2, 3) = imuMeasurementCovariance(0, 1);
    this->m_R(3, 2) = imuMeasurementCovariance(1, 0);
    this->m_R(3, 3) = imuMeasurementCovariance(1, 1);

    auto transitionJacobian = this->BuildFk(dt, this->m_x, imuControl);

    auto predictedState = this->f(dt, this->m_x, imuControl);
    auto predictedCovariance = transitionJacobian * this->m_P * transitionJacobian.transpose() + this->m_Q;

    Eigen::Matrix<double, M_Z, 1> measurement;
    measurement << Eigen::Matrix<double, 2, 1>::Zero(), imuControl;

    auto innovation = measurement - this->m_H_IMU * predictedState;
    auto innovationCovariance = this->m_H_IMU * predictedCovariance * this->m_H_IMU.transpose() + this->m_R;
    auto kalmanGain = predictedCovariance * this->m_H_IMU.transpose() * innovationCovariance.inverse();

    this->m_x = predictedState + kalmanGain * innovation;

    auto identity = Eigen::Matrix<double, N_STATE, N_STATE>::Identity();
    auto residualProjection = identity - kalmanGain * this->m_H_IMU;
    this->m_P = residualProjection * predictedCovariance * residualProjection.transpose() +
                kalmanGain * this->m_R * kalmanGain.transpose();

    return {this->m_x, this->m_P};
}

std::pair<Eigen::Matrix<double, N_STATE, 1>, Eigen::Matrix<double, N_STATE, N_STATE>> 
IMUGPSFusionKF::Step(double dt,
                    Eigen::Matrix<double, 2, 1>& gpsMeasurement,
                    Eigen::Matrix<double, 2, 1>& imuControl) {
    this->m_Q = this->Update_Q(this->m_lastGPSDt + dt, this->m_x);

    this->m_lastGPSDt = 0.0;

    auto imuMeasurementCovariance = this->m_covarianceEstimator.GetR(imuControl(0, 0), imuControl(1, 0));

    this->m_R(2, 2) = imuMeasurementCovariance(0, 0);
    this->m_R(2, 3) = imuMeasurementCovariance(0, 1);
    this->m_R(3, 2) = imuMeasurementCovariance(1, 0);
    this->m_R(3, 3) = imuMeasurementCovariance(1, 1);

    auto transitionJacobian = this->BuildFk(dt, this->m_x, imuControl);

    auto predictedState = this->f(dt, this->m_x, imuControl);
    auto predictedCovariance = transitionJacobian * this->m_P * transitionJacobian.transpose() + this->m_Q;

    Eigen::Matrix<double, M_Z, 1> measurement;
    measurement << gpsMeasurement, imuControl;

    auto innovation = measurement - this->m_H_GPS_IMU * predictedState;
    auto innovationCovariance = this->m_H_GPS_IMU * predictedCovariance * this->m_H_GPS_IMU.transpose() + this->m_R;
    auto kalmanGain = predictedCovariance * this->m_H_GPS_IMU.transpose() * innovationCovariance.inverse();

    this->m_x = predictedState + kalmanGain * innovation;

    auto identity = Eigen::Matrix<double, N_STATE, N_STATE>::Identity();
    auto residualProjection = identity - kalmanGain * this->m_H_GPS_IMU;
    this->m_P = residualProjection * predictedCovariance * residualProjection.transpose() +
                kalmanGain * this->m_R * kalmanGain.transpose();

    return {this->m_x, this->m_P};
}

Eigen::Matrix<double, N_STATE, N_STATE>
IMUGPSFusionKF::Update_Q(double dt, Eigen::Matrix<double, N_STATE, 1> state) {
    const double heading = state(2);
    const double speed = state(3);
    const double yawRate = state(4);

    const double jerkPSD = 0.3;
    const double yawAccelerationPSD = 3.14159265358979323846 / 20.0;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    const double middleHeading = heading + 0.5 * yawRate * dt;
    const double c = std::cos(middleHeading);
    const double s = std::sin(middleHeading);

    Eigen::Matrix<double, N_STATE, N_STATE> Q = Eigen::Matrix<double, N_STATE, N_STATE>::Zero();

    auto addSymmetric = [&Q](int row, int column, double value) {
        Q(row, column) += value;

        if (row != column) {
            Q(column, row) += value;
        }
    };

    addSymmetric(0, 0, jerkPSD * c * c * dt5 / 20.0);
    addSymmetric(0, 1, jerkPSD * c * s * dt5 / 20.0);
    addSymmetric(1, 1, jerkPSD * s * s * dt5 / 20.0);

    addSymmetric(0, 3, jerkPSD * c * dt4 / 8.0);
    addSymmetric(1, 3, jerkPSD * s * dt4 / 8.0);

    addSymmetric(0, 5, jerkPSD * c * dt3 / 6.0);
    addSymmetric(1, 5, jerkPSD * s * dt3 / 6.0);

    addSymmetric(3, 3, jerkPSD * dt3 / 3.0);
    addSymmetric(3, 5, jerkPSD * dt2 / 2.0);
    addSymmetric(5, 5, jerkPSD * dt);

    addSymmetric(0, 0, yawAccelerationPSD * speed * speed * s * s * dt5 / 20.0);
    addSymmetric(0, 1, -yawAccelerationPSD * speed * speed * s * c * dt5 / 20.0);
    addSymmetric(1, 1, yawAccelerationPSD * speed * speed * c * c * dt5 / 20.0);
    addSymmetric(0, 2, -yawAccelerationPSD * speed * s * dt4 / 8.0);
    addSymmetric(1, 2, yawAccelerationPSD * speed * c * dt4 / 8.0);
    addSymmetric(0, 4, -yawAccelerationPSD * speed * s * dt3 / 6.0);
    addSymmetric(1, 4, yawAccelerationPSD * speed * c * dt3 / 6.0);

    addSymmetric(2, 2, yawAccelerationPSD * dt3 / 3.0);
    addSymmetric(2, 4, yawAccelerationPSD * dt2 / 2.0);
    addSymmetric(4, 4, yawAccelerationPSD * dt);

    return Q;
}

void 
IMUGPSFusionKF::UpdatePosition(double easting,
                              double northing,
                              double oldLatitude,
                              double oldLongitude,
                              double newLatitude,
                              double newLongitude) {
    constexpr double piOver180 = 0.0174532925199433;
    oldLatitude *= piOver180;
    oldLongitude *= piOver180;
    newLongitude *= piOver180;
    newLatitude *= piOver180;
    const double yaw = this->m_x(2);

    const double longitudeDelta = newLongitude - oldLongitude;
    const double sinNewLatitude = std::sin(newLatitude);
    const double sinLongitudeDelta = std::sin(longitudeDelta);
    const double cosYaw = std::cos(yaw);
    const double sinOldLatitude = std::sin(oldLatitude);
    const double cosLongitudeDelta = std::cos(longitudeDelta);
    const double sinYaw = std::sin(yaw);

    const double newYaw = std::atan2(
        -sinNewLatitude * sinLongitudeDelta * cosYaw +
            (sinNewLatitude * sinOldLatitude * cosLongitudeDelta +
             std::cos(newLatitude) * std::cos(oldLatitude)) * sinYaw,
        cosLongitudeDelta * cosYaw + sinOldLatitude * sinLongitudeDelta * sinYaw);
    
    this->m_x(0, 0) = easting;
    this->m_x(1, 0) = northing;
    this->m_x(2, 0) = newYaw;

}
