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
                         0, 0, 0, 0, 1, 0,
                         0, 0, 0, 0, 0, 1;

    this->m_H_IMU << 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 1, 0,
                     0, 0, 0, 0, 0, 1;

    this->Update_Q(1.0 / 100.0, this->m_x);

    this->m_lastGPSDt = 0.0;

    Eigen::Matrix<double, 2, 2> IMUR;
    IMUR << R0(2, 2), R0(2, 3), R0(3, 2), R0(3, 3);

    this->m_covarianceEstimator = IMUCovarianceEstimator(IMUR, 500, 200);
}

Eigen::Matrix<double, N_STATE, N_STATE>
IMUGPSFusionKF::BuildFk(double dt, Eigen::Matrix<double, N_STATE, 1> x, Eigen::Matrix<double, 2, 1> u) {
    // const double u_a = u(0, 0);
    const double u_phi_prime = u(1, 0);

    const double psi = x(2);
    const double v = x(3);
    // const double yawRate = x(4);

    const double dt2 = dt * dt;
    const double sinPsi = std::sin(psi);
    const double cosPsi = std::cos(psi);

    Eigen::Matrix<double, N_STATE, N_STATE> F = Eigen::Matrix<double, N_STATE, N_STATE>::Identity();

    F(2, 4) = dt;
    F(3, 5) = dt;

    constexpr double yawRateThreshold = 0.04;

    if (std::abs(u_phi_prime) < yawRateThreshold) {
        F(0, 2) = -v * dt * sinPsi;
        F(0, 3) = dt * cosPsi;
        F(0, 4) = -0.5 * v * dt2 * sinPsi;

        F(1, 2) = v * dt * cosPsi;
        F(1, 3) = dt * sinPsi;
        F(1, 4) = 0.5 * v * dt2 * cosPsi;

        return F;
    }

    const double newPsi = psi + u_phi_prime * dt;
    const double sinNewPsi = std::sin(newPsi);
    const double cosNewPsi = std::cos(newPsi);
    const double yawRate2 = u_phi_prime * u_phi_prime;

    const double deltaSin = sinNewPsi - sinPsi;
    const double deltaCos = cosPsi - cosNewPsi;

    F(0, 2) = (v / u_phi_prime) * (cosNewPsi - cosPsi);
    F(0, 3) = deltaSin / u_phi_prime;
    F(0, 4) = v * (u_phi_prime * dt * cosNewPsi - deltaSin) / yawRate2;

    F(1, 2) = (v / u_phi_prime) * (sinNewPsi - sinPsi);
    F(1, 3) = deltaCos / u_phi_prime;
    F(1, 4) = v * (u_phi_prime * dt * sinNewPsi - deltaCos) / yawRate2;

    return F;
}

Eigen::Matrix<double, N_STATE, 1>
IMUGPSFusionKF::f(double dt, Eigen::Matrix<double, N_STATE, 1> x, Eigen::Matrix<double, 2, 1> u) {
    const double u_a = u(0, 0);
    const double u_phi_prime = u(1, 0);

    const double X = x(0, 0);
    const double Y = x(1, 0);
    const double phi = x(2, 0);
    const double v = x(3, 0);
    const double phi_prime = x(4, 0);
    const double a = x(5, 0);

    Eigen::Matrix<double, N_STATE, 1> x_update;

    const double cos_phi = std::cos(phi);
    const double sin_phi = std::sin(phi);

    if (u_phi_prime < 0.04) {
        x_update(0, 0) = X + v * dt * cos_phi + 0.5 * dt * dt * u_a * cos_phi;
        x_update(1, 0) = Y + v * dt * sin_phi + 0.5 * dt * dt * u_a * sin_phi;
        x_update(2, 0) = phi + dt * u_phi_prime;
        x_update(3, 0) = v + dt * a;
        x_update(4, 0) = phi_prime;
        x_update(5, 0) = a;
    }
    else {
        const double v_over_phi_prime = v / u_phi_prime;
        const double new_heading = phi + dt * u_phi_prime;
        x_update(0, 0) = X + v_over_phi_prime * (std::sin(new_heading) - sin_phi);
        x_update(1, 0) = Y + v_over_phi_prime * (-std::cos(new_heading) + cos_phi);
        x_update(2, 0) = new_heading;
        x_update(3, 0) = v + dt * u_a;
        x_update(4, 0) = phi_prime;
        x_update(5, 0) = a;
    }

    return x_update;
}

std::pair<Eigen::Matrix<double, N_STATE, 1>, Eigen::Matrix<double, N_STATE, N_STATE>> 
IMUGPSFusionKF::Step(double dt, Eigen::Matrix<double, 2, 1>& z_IMU) {
    this->m_Q = this->Update_Q(dt, this->m_x);

    this->m_lastGPSDt += dt;

    auto IMUR = this->m_covarianceEstimator.GetR(z_IMU(0,0), z_IMU(1,0));

    this->m_R(2, 2) = IMUR(0, 0); 
    this->m_R(2, 3) = IMUR(0, 1); 
    this->m_R(3, 2) = IMUR(1, 0); 
    this->m_R(3, 3) = IMUR(1, 1); 

    auto Fk = this->BuildFk(dt, this->m_x, z_IMU);

    auto x_priori = this->f(dt, this->m_x, z_IMU);
    auto P_priori = Fk * this->m_P * Fk.transpose() + this->m_Q;

    Eigen::Matrix<double, M_Z, 1> z;
    z << Eigen::Matrix<double, 2, 1>::Zero(), z_IMU;

    auto y = z - this->m_H_IMU * x_priori;
    auto S = this->m_H_IMU * P_priori * this->m_H_IMU.transpose() + this->m_R;
    auto K = P_priori * this->m_H_IMU.transpose() * S.inverse();

    this->m_x = x_priori + K * y;

    auto I = Eigen::Matrix<double, N_STATE, N_STATE>::Identity();
    auto IKH = I - K * this->m_H_IMU;
    this->m_P = IKH * P_priori * IKH.transpose() + K * this->m_R * K.transpose();

    // this->m_R = this->m_covarianceEstimator.R(z, this->m_x, P_priori, this->m_H_IMU);
    // this->m_Q = this->m_covarianceEstimator.Q(z, x_priori, P_priori, K, this->m_H_IMU);

    return {this->m_x, this->m_P};
}

std::pair<Eigen::Matrix<double, N_STATE, 1>, Eigen::Matrix<double, N_STATE, N_STATE>> 
IMUGPSFusionKF::Step(double dt, Eigen::Matrix<double, 2, 1>& z_GPS, Eigen::Matrix<double, 2, 1>& z_IMU) {
    this->m_Q = this->Update_Q(this->m_lastGPSDt + dt, this->m_x);

    this->m_lastGPSDt = 0.0;

    auto IMUR = this->m_covarianceEstimator.GetR(z_IMU(0,0), z_IMU(1,0));

    this->m_R(2, 2) = IMUR(0, 0); 
    this->m_R(2, 3) = IMUR(0, 1); 
    this->m_R(3, 2) = IMUR(1, 0); 
    this->m_R(3, 3) = IMUR(1, 1); 

    auto Fk = this->BuildFk(dt, this->m_x, z_IMU);

    auto x_priori = this->f(dt, this->m_x, z_IMU);
    auto P_priori = Fk * this->m_P * Fk.transpose() + this->m_Q;

    Eigen::Matrix<double, M_Z, 1> z;
    z << z_GPS, z_IMU;

    auto y = z - this->m_H_GPS_IMU * x_priori;
    auto S = this->m_H_GPS_IMU * P_priori * this->m_H_GPS_IMU.transpose() + this->m_R;
    auto K = P_priori * this->m_H_GPS_IMU.transpose() * S.inverse();

    this->m_x = x_priori + K * y;

    auto I = Eigen::Matrix<double, N_STATE, N_STATE>::Identity();
    auto IKH = I - K * this->m_H_GPS_IMU;
    this->m_P = IKH * P_priori * IKH.transpose() + K * this->m_R * K.transpose();

    // this->m_R = this->m_covarianceEstimator.R(z, this->m_x, P_priori, this->m_H_GPS_IMU);
    // this->m_Q = this->m_covarianceEstimator.Q(z, x_priori, P_priori, K, this->m_H_GPS_IMU);

    return {this->m_x, this->m_P};
}

Eigen::Matrix<double, N_STATE, N_STATE>
IMUGPSFusionKF::Update_Q(double dt, Eigen::Matrix<double, N_STATE, 1> x) {
    const double psi = x(2);
    const double v = x(3);
    const double yawRate = x(4);

    const double jerkPSD = 0.3;
    const double yawAccelerationPSD = 3.14159265358979323846 / 10.0;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    const double middleHeading = psi + 0.5 * yawRate * dt;
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

    addSymmetric(0, 0, yawAccelerationPSD * v * v * s * s * dt5 / 20.0);
    addSymmetric(0, 1, -yawAccelerationPSD * v * v * s * c * dt5 / 20.0);
    addSymmetric(1, 1, yawAccelerationPSD * v * v * c * c * dt5 / 20.0);
    addSymmetric(0, 2, -yawAccelerationPSD * v * s * dt4 / 8.0);
    addSymmetric(1, 2, yawAccelerationPSD * v * c * dt4 / 8.0);
    addSymmetric(0, 4, -yawAccelerationPSD * v * s * dt3 / 6.0);
    addSymmetric(1, 4, yawAccelerationPSD * v * c * dt3 / 6.0);

    addSymmetric(2, 2, yawAccelerationPSD * dt3 / 3.0);
    addSymmetric(2, 4, yawAccelerationPSD * dt2 / 2.0);
    addSymmetric(4, 4, yawAccelerationPSD * dt);

    return Q;
}

void 
IMUGPSFusionKF::UpdatePosition(double E, double N, double oldLat, double oldLon, double newLat, double newLon) {
    constexpr double piOver180 = 0.0174532925199433;
    oldLat *= piOver180;
    oldLon *= piOver180;
    newLon *= piOver180;
    newLat *= piOver180;
    const double yaw = this->m_x(2);

    const double dLon = newLon - oldLon;
    const double sinNewLat = std::sin(newLat);
    const double sinDLon = std::sin(dLon);
    const double cosYaw = std::cos(yaw);
    const double sinOldLat = std::sin(oldLat);
    const double cosDLon = std::cos(dLon);
    const double sinYaw = std::sin(yaw);

    const double newYaw = std::atan2(-sinNewLat * sinDLon * cosYaw + ( sinNewLat * sinOldLat * cosDLon + std::cos(newLat) * std::cos(oldLat)) * sinYaw,
                                     cosDLon * cosYaw + sinOldLat * sinDLon * sinYaw);
    
    this->m_x(0, 0) = E;
    this->m_x(1, 0) = N;
    this->m_x(2, 0) = newYaw;

}
