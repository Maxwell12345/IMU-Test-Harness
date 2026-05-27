#include "IMUGPSFusionKF.hpp"

IMUGPSFusionKF_2D_ConstantAcceleration::IMUGPSFusionKF_2D_ConstantAcceleration(
    Vector6d x0, 
    Matrix6d P0, 
    Matrix6d R0_GPS,
    Matrix6d R0_IMU,
    Matrix6d Q0,
    double chiSquaredBetaLowerBound_GPS,
    double chiSquaredBetaLowerBound_IMU,
    double chiSquaredBetaUpperBound_GPS,
    double chiSquaredBetaUpperBound_IMU,
    unsigned N_GPS,
    unsigned L_GPS, 
    unsigned N_IMU, 
    unsigned L_IMU, 
    unsigned N_Q,
    unsigned L_Q 
) {
    this->m_x = x0;
    this->m_P = P0;
    this->m_R_GPS = R0_GPS;
    this->m_R_IMU = R0_IMU;
    this->m_Q = Q0;
    this->m_chiSquaredBetaLowerBound_GPS = chiSquaredBetaLowerBound_GPS;
    this->m_chiSquaredBetaLowerBound_IMU = chiSquaredBetaLowerBound_IMU;
    this->m_chiSquaredBetaUpperBound_GPS = chiSquaredBetaUpperBound_GPS;
    this->m_chiSquaredBetaUpperBound_IMU = chiSquaredBetaUpperBound_IMU;
    this->m_I = Eigen::Matrix2d::Identity();
    this->m_zero = Eigen::Matrix2d::Zero();

    this->m_H_GPS << this->m_I, this->m_zero, this->m_zero,
                     this->m_zero, this->m_zero, this->m_zero,
                     this->m_zero, this->m_zero, this->m_zero;

    this->m_H_IMU << this->m_zero, this->m_zero, this->m_zero,
                     this->m_zero, this->m_zero, this->m_zero,
                     this->m_zero, this->m_zero, this->m_I;

    this->m_jerkPSD = 0.1;

    this->m_N_GPS = N_GPS;
    this->m_L_GPS = L_GPS;
    this->m_l_GPS = 0;
    this->m_N_IMU = N_IMU;
    this->m_L_IMU = L_IMU;
    this->m_l_IMU = 0;
    this->m_N_Q = N_Q;
    this->m_L_Q = L_Q;
    this->m_l_Q = 0;

    this->Update_Q(1.0/100.0, true);
}

Matrix6d IMUGPSFusionKF_2D_ConstantAcceleration::BuildFk(double dt) {
    Matrix6d Fk;
    
    Fk << this->m_I, this->m_I * dt, this->m_I * (dt * dt) * 0.5,
            this->m_zero, this->m_I, this->m_I * dt,
            this->m_zero, this->m_zero, this->m_I;

    return Fk;
}

Eigen::Vector4d 
IMUGPSFusionKF_2D_ConstantAcceleration::CalculateBetas(Matrix6d priori_P, Vector6d innovation_IMU) {
    Matrix6d S_IMU = this->m_H_IMU * priori_P * this->m_H_IMU.transpose() + this->m_R_IMU;

    Eigen::Matrix<double, 1, 1> NIS_IMU = innovation_IMU.transpose() * S_IMU.inverse() * innovation_IMU;

    double mu_IMU = this->CalculateFuzzyLogicMembershipFunction(NIS_IMU(0, 0), false);

    Eigen::Vector4d Betas;
    Betas << std::max(1.0 - mu_IMU, 0.0), 0.0, std::max(mu_IMU, 0.0), 0.0;

    return Betas;
}

Eigen::Vector4d 
IMUGPSFusionKF_2D_ConstantAcceleration::CalculateBetas(Matrix6d priori_P, Vector6d innovation_GPS, Vector6d innovation_IMU) {
    Matrix6d S_IMU = this->m_H_IMU * priori_P * this->m_H_IMU.transpose() + this->m_R_IMU;
    Matrix6d S_GPS = this->m_H_GPS * priori_P * this->m_H_GPS.transpose() + this->m_R_GPS;

    Eigen::Matrix<double, 1, 1> NIS_GPS = innovation_GPS.transpose() * S_GPS.inverse() * innovation_GPS;
    Eigen::Matrix<double, 1, 1> NIS_IMU = innovation_IMU.transpose() * S_IMU.inverse() * innovation_IMU;

    double mu_GPS = this->CalculateFuzzyLogicMembershipFunction(NIS_GPS(0, 0), true);
    double mu_IMU = this->CalculateFuzzyLogicMembershipFunction(NIS_IMU(0, 0), false);
    
    double beta_GPS_IMU = mu_GPS * mu_IMU;

    Eigen::Vector4d Betas;
    Betas << std::max(1.0 - mu_GPS - mu_IMU + beta_GPS_IMU, 0.0), std::max(mu_GPS - beta_GPS_IMU, 0.0), std::max(mu_IMU - beta_GPS_IMU, 0.0), std::max(beta_GPS_IMU, 0.0);
    std::cout << Betas(0) << " " << Betas(1) << " " << Betas(2) << " " << Betas(3)  << std::endl;
   
    return Betas;
}

double 
IMUGPSFusionKF_2D_ConstantAcceleration::CalculateFuzzyLogicMembershipFunction(double chi_squared_stat, bool is_GPS) {
    double lowerBound = this->m_chiSquaredBetaLowerBound_IMU;
    double upperBound = this->m_chiSquaredBetaUpperBound_IMU;

    if (is_GPS) {
        lowerBound = this->m_chiSquaredBetaLowerBound_GPS;
        upperBound = this->m_chiSquaredBetaUpperBound_GPS;
    }

    const double min_GPS = 0.01; 

    if (!std::isfinite(chi_squared_stat)) {
        return 0.0;
    }

    if (chi_squared_stat <= lowerBound) {
        return 1.0;
    }

    if (chi_squared_stat >= upperBound) {
        return 0.0;
    }

    double t = (chi_squared_stat - lowerBound) / (upperBound - lowerBound);

    double mu = 0.5 * (1.0 + std::cos(3.1415926535 * t));

    if (is_GPS) {
        return std::max(mu, min_GPS);
    }


    return mu;
}

std::pair<Matrix6d, Matrix6d> 
IMUGPSFusionKF_2D_ConstantAcceleration::Calculate_GPS_KalmanGains(Matrix6d priori_P_inv, Vector6d innovation_GPS, Matrix6d postiori_P_GPS_IMU) {
    Matrix6d R_GPS_inv = this->m_R_GPS.inverse();
    Matrix6d H_GPS_T = this->m_H_GPS.transpose();
    Matrix6d postiori_P_GPS_inv = priori_P_inv + H_GPS_T * R_GPS_inv * this->m_H_GPS;

    Matrix6d K_GPS = postiori_P_GPS_inv.inverse() * H_GPS_T * R_GPS_inv;

    Matrix6d K_GPS_given_GPS_plus_IMU = postiori_P_GPS_IMU * H_GPS_T * R_GPS_inv;

    return {K_GPS, K_GPS_given_GPS_plus_IMU};
}

std::pair<Matrix6d, Matrix6d> 
IMUGPSFusionKF_2D_ConstantAcceleration::Calculate_IMU_KalmanGains(Matrix6d priori_P_inv, Vector6d innovation_IMU, Matrix6d postiori_P_GPS_IMU) {
    Matrix6d R_IMU_inv = this->m_R_IMU.inverse();
    Matrix6d H_IMU_T = this->m_H_IMU.transpose();
    Matrix6d postiori_P_IMU_inv = priori_P_inv + H_IMU_T * R_IMU_inv * this->m_H_IMU;

    Matrix6d K_IMU = postiori_P_IMU_inv.inverse() * H_IMU_T * R_IMU_inv;

    Matrix6d K_IMU_given_IMU_plus_IMU = postiori_P_GPS_IMU * H_IMU_T * R_IMU_inv;

    return {K_IMU, K_IMU_given_IMU_plus_IMU};
}


std::pair<Vector6d, Matrix6d> 
IMUGPSFusionKF_2D_ConstantAcceleration::Step(double dt, Vector6d& z_IMU) {
    // Propagate process noise covariances
    // this->Update_Q(dt, false);

    // Assemble transition state matrix
    Matrix6d F_k = this->BuildFk(dt);

    // Calculate priori estimates.
    Vector6d priori_x = F_k * this->m_x; 

    this->m_P_propagated_lag = F_k * this->m_P * F_k.transpose();
    Matrix6d priori_P = this->m_P_propagated_lag + this->m_Q;

    // Calculate IMU innovation.
    Vector6d innovation_IMU = z_IMU - this->m_H_IMU * priori_x;

    // Calculate Betas.
    Eigen::Vector4d Betas = this->CalculateBetas(priori_P, innovation_IMU);

    // Calculate IMU Kalman gains.
    Matrix6d priori_P_inv = priori_P.inverse();

    Matrix6d R_GPS_inv = this->m_R_GPS.inverse();
    Matrix6d R_IMU_inv = this->m_R_IMU.inverse();

    Matrix6d HT_Rinv_H = this->m_H_IMU.transpose() * R_IMU_inv * this->m_H_IMU;

    Matrix6d postiori_P_GPS_IMU = (
        priori_P_inv + 
        this->m_H_GPS.transpose() * R_GPS_inv * this->m_H_GPS +
        HT_Rinv_H
    ).inverse();

    std::pair<Matrix6d, Matrix6d> IMU_KalmanGains = this->Calculate_IMU_KalmanGains(priori_P_inv, innovation_IMU, postiori_P_GPS_IMU);

    // Calculate postiori x and P IMU.
    Vector6d postiori_x_IMU = priori_x + IMU_KalmanGains.first * innovation_IMU;
    Matrix6d postiori_P_IMU = (priori_P_inv + HT_Rinv_H).inverse();

    // Calculate fused postiori x and P.
    this->m_x = Betas(0) * priori_x + Betas(2) * postiori_x_IMU;

    Vector6d x_innovation_IMU = this->m_x - postiori_x_IMU;
    this->m_P = Betas(0) * priori_P + 
                Betas(2) * (postiori_P_IMU + x_innovation_IMU * x_innovation_IMU.transpose());

    Vector6d residual_IMU = z_IMU - this->m_H_IMU * this->m_x;

    this->PushInnovationIMU(residual_IMU, this->m_P);

    Vector6d posteriorResidual = this->m_x - priori_x;
    this->PushInnovationQ(posteriorResidual, this->m_P);

    return {this->m_x, priori_P};
}

std::pair<Vector6d, Matrix6d> 
IMUGPSFusionKF_2D_ConstantAcceleration::Step(double dt, Vector6d& z_GPS, Vector6d& z_IMU) {
    // Propagate process noise covariances
    // this->Update_Q(dt, true);

    // Assemble transition state matrix
    Matrix6d F_k = this->BuildFk(dt);

    // Calculate priori estimates.
    Vector6d priori_x = F_k * this->m_x; 

    this->m_P_propagated_lag = F_k * this->m_P * F_k.transpose();
    Matrix6d priori_P = this->m_P_propagated_lag + this->m_Q;

    // Calculate GPS and IMU innovation.
    Vector6d innovation_IMU = z_IMU - this->m_H_IMU * priori_x;
    Vector6d innovation_GPS = z_GPS - this->m_H_GPS * priori_x;

    // Calculate Betas.
    Eigen::Vector4d Betas = this->CalculateBetas(priori_P, innovation_GPS, innovation_IMU);

    // Calculate GPS and IMU Kalman gains.
    Matrix6d priori_P_inv = priori_P.inverse();

    Matrix6d R_GPS_inv = this->m_R_GPS.inverse();
    Matrix6d R_IMU_inv = this->m_R_IMU.inverse();

    Matrix6d HT_Rinv_H_GPS = this->m_H_GPS.transpose() * R_GPS_inv * this->m_H_GPS;
    Matrix6d HT_Rinv_H_IMU = this->m_H_IMU.transpose() * R_IMU_inv * this->m_H_IMU;

    Matrix6d postiori_P_GPS_IMU = (priori_P_inv + HT_Rinv_H_GPS + HT_Rinv_H_IMU).inverse();

    std::pair<Matrix6d, Matrix6d> IMU_KalmanGains = this->Calculate_IMU_KalmanGains(priori_P_inv, innovation_IMU, postiori_P_GPS_IMU);
    std::pair<Matrix6d, Matrix6d> GPS_KalmanGains = this->Calculate_GPS_KalmanGains(priori_P_inv, innovation_GPS, postiori_P_GPS_IMU);

    // Calculate postiori x and P GPS and IMU.
    Vector6d postiori_x_IMU = priori_x + IMU_KalmanGains.first * innovation_IMU;
    Matrix6d postiori_P_IMU = (priori_P_inv + HT_Rinv_H_IMU).inverse();

    Vector6d postiori_x_GPS = priori_x + GPS_KalmanGains.first * innovation_GPS;
    Matrix6d postiori_P_GPS = (priori_P_inv + HT_Rinv_H_GPS).inverse();

    Vector6d postiori_x_GPS_IMU = priori_x + GPS_KalmanGains.second * innovation_GPS + IMU_KalmanGains.second * innovation_IMU;

    // Calculate fused postiori x and P.
    this->m_x = Betas(0) * priori_x + Betas(1) * postiori_x_GPS + Betas(2) * postiori_x_IMU + Betas(3) * postiori_x_GPS_IMU;

    Vector6d x_innovation_IMU = this->m_x - postiori_x_IMU;
    Vector6d x_innovation_GPS = this->m_x - postiori_x_GPS;
    Vector6d x_innovation_GPS_IMU = this->m_x - postiori_x_GPS_IMU;
    this->m_P = Betas(0) * priori_P + 
                Betas(1) * (postiori_P_GPS + x_innovation_GPS * x_innovation_GPS.transpose()) +
                Betas(2) * (postiori_P_IMU + x_innovation_IMU * x_innovation_IMU.transpose()) +
                Betas(3) * (postiori_P_GPS_IMU + x_innovation_GPS_IMU * x_innovation_GPS_IMU.transpose());

    Vector6d residual_GPS = z_GPS - this->m_H_GPS * this->m_x;
    Vector6d residual_IMU = z_IMU - this->m_H_IMU * this->m_x;

    this->PushInnovationGPS(residual_GPS, this->m_P);
    this->PushInnovationIMU(residual_IMU, this->m_P);

    Vector6d posteriorResidual = this->m_x - priori_x;
    this->PushInnovationQ(posteriorResidual, this->m_P);

    return {this->m_x, this->m_P};
}

void IMUGPSFusionKF_2D_ConstantAcceleration::Update_GPS_R(Matrix6d R_GPS) {
    this->m_R_GPS = R_GPS;
}


void IMUGPSFusionKF_2D_ConstantAcceleration::Update_IMU_R(Matrix6d R_IMU) {
    this->m_R_IMU = R_IMU;
}

void IMUGPSFusionKF_2D_ConstantAcceleration::Update_Q(double dt, bool has_GPS) {
    if (!std::isfinite(dt) || dt <= 0.0) {
        this->m_Q = Matrix6d::Zero();
        return;
    }

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;

    const double q_nominal = this->m_jerkPSD;

    const double q_scale_with_gps = 2.0;

    const double q_scale_without_gps = 1.0;

    const double q_eff = q_nominal * (has_GPS ? q_scale_with_gps : q_scale_without_gps);

    const double q11 = q_eff * dt5 / 20.0;
    const double q12 = q_eff * dt4 / 8.0;
    const double q13 = q_eff * dt3 / 6.0;
    const double q22 = q_eff * dt3 / 3.0;
    const double q23 = q_eff * dt2 / 2.0;
    const double q33 = q_eff * dt;

    this->m_Q << this->m_I * q11, this->m_I * q12, this->m_I * q13,
                 this->m_I * q12, this->m_I * q22, this->m_I * q23,
                 this->m_I * q13, this->m_I * q23, this->m_I * q33;

    this->m_Q = 0.5 * (this->m_Q + this->m_Q.transpose());
}

void 
IMUGPSFusionKF_2D_ConstantAcceleration::PushInnovationGPS(Vector6d& residual, Matrix6d& postiori_P) {
    if (this->m_N_GPS == 0) {
        return;
    }

    if (this->m_innocationQueue_GPS.size() < this->m_N_GPS) {
        this->m_innocationQueue_GPS.emplace_back(residual);
        return;
    }

    this->m_innocationQueue_GPS.pop_front();
    this->m_innocationQueue_GPS.emplace_back(residual);

    if (this->m_l_GPS >= this->m_L_GPS) {
        Matrix6d innovationSum = Matrix6d::Zero();
        for (const auto& innovation : this->m_innocationQueue_GPS) {
            innovationSum += innovation * innovation.transpose();
        }
        innovationSum /= this->m_N_GPS;

        this->m_R_GPS = innovationSum + this->m_H_GPS * postiori_P * this->m_H_GPS.transpose(); 

        double epsilon = 1e10;
        this->m_R_GPS(2, 2) = epsilon + 1e-6;
        this->m_R_GPS(3, 3) = epsilon - 1.2e-6;
        this->m_R_GPS(4, 4) = epsilon + 1.07e-6;
        this->m_R_GPS(5, 5) = epsilon - 1.01e-6;

        this->m_R_GPS(0,0) = std::max(this->m_R_GPS(0,0), epsilon + 2e-14);
        this->m_R_GPS(1,1) = std::max(this->m_R_GPS(1,1), epsilon - 2e-14);

        this->m_l_GPS = 0;
    }

    this->m_l_GPS++;
}


void 
IMUGPSFusionKF_2D_ConstantAcceleration::PushInnovationIMU(Vector6d& residual,Matrix6d& postiori_P) {
    if (this->m_N_IMU == 0) {
        return;
    }

    if (this->m_innocationQueue_IMU.size() < this->m_N_IMU) {
        this->m_innocationQueue_IMU.emplace_back(residual);
        return;
    }

    this->m_innocationQueue_IMU.pop_front();
    this->m_innocationQueue_IMU.emplace_back(residual);

    if (this->m_l_IMU >= this->m_L_IMU) {
        Matrix6d innovationSum = Matrix6d::Zero();
        for (const auto& innovation : this->m_innocationQueue_IMU) {
            innovationSum += innovation * innovation.transpose();
        }
        innovationSum /= this->m_N_IMU;

        this->m_R_IMU = innovationSum + this->m_H_IMU * postiori_P * this->m_H_IMU.transpose(); 

        double epsilon = 1e10;
        this->m_R_IMU(0, 0) = epsilon + 1e-6;
        this->m_R_IMU(1, 1) = epsilon - 1.5e-6;
        this->m_R_IMU(2, 2) = epsilon + 1.09e-6;
        this->m_R_IMU(3, 3) = epsilon - 1.2e-6;

        // this->m_R_IMU(2,2) = std::max(this->m_R_IMU(2,2), epsilon);
        // this->m_R_IMU(3,3) = std::max(this->m_R_IMU(3,3), epsilon + 2e-14);

        this->m_R_IMU(4,4) = std::max(this->m_R_IMU(4,4), epsilon - 2e-14);
        this->m_R_IMU(5,5) = std::max(this->m_R_IMU(5,5), epsilon + 3e-14);

        this->m_l_IMU = 0;
    }
    this->m_l_IMU++;
}

void 
IMUGPSFusionKF_2D_ConstantAcceleration::PushInnovationQ(Vector6d &posteriorResidual, Matrix6d &postiori_P) {
    if (this->m_N_Q == 0) {
        return;
    }

    if (this->m_posteriorResidualQueue.size() < this->m_N_Q) {
        this->m_posteriorResidualQueue.emplace_back(posteriorResidual);
        return;
    }

    this->m_posteriorResidualQueue.pop_front();
    this->m_posteriorResidualQueue.emplace_back(posteriorResidual);

    if (this->m_l_Q >= this->m_L_Q) {
        Matrix6d residualSum = Matrix6d::Zero();
        for (const auto& residual : this->m_posteriorResidualQueue) {
            residualSum += residual * residual.transpose();
        }
        residualSum /= this->m_N_Q;

        this->m_Q = residualSum + postiori_P - this->m_P_propagated_lag; 

        double epsilon = 1e-13;
        this->m_Q(0,0) = std::max(this->m_Q(0,0), epsilon + 2e-14);
        this->m_Q(1,1) = std::max(this->m_Q(1,1), epsilon - 2e-14);
        this->m_Q(2,2) = std::max(this->m_Q(2,2), epsilon + 3e-14);
        this->m_Q(3,3) = std::max(this->m_Q(3,3), epsilon - 3e-14);
        this->m_Q(4,4) = std::max(this->m_Q(4,4), epsilon + 4e-14);
        this->m_Q(5,5) = std::max(this->m_Q(5,5), epsilon - 4e-14);

        this->m_l_Q = 0;
    }
    this->m_l_Q++;
}
