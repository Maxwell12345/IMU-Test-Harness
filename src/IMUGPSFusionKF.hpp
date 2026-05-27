#include <iostream>
#include <Eigen/Dense> 
#include <deque>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

class IMUGPSFusionKF_2D_ConstantAcceleration {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    IMUGPSFusionKF_2D_ConstantAcceleration() = default;

    IMUGPSFusionKF_2D_ConstantAcceleration(
        Vector6d x0, // initial state vector
        Matrix6d P0, // initial error covariance matrix
        Matrix6d R0_GPS, // initial GPS measurement noise covariance
        Matrix6d R0_IMU, // initial IUM measurement noise covariance
        Matrix6d Q0, // initial process noise covariance
        double chiSquaredBetaLowerBound_GPS, // lower bound chi squared probability for "valid" GPS innovations
        double chiSquaredBetaLowerBound_IMU, // lower bound chi squared probability for "valid" IMU innovations
        double chiSquaredBetaUpperBound_GPS, // upper bound chi squared probability for "valid" GPS innovations
        double chiSquaredBetaUpperBound_IMU, // upper bound chi squared probability for "valid" IMU innovations
        unsigned N_GPS, // Adaptive R_GPS approx sample depth
        unsigned L_GPS, // Adaptive R_GPS update frequency
        unsigned N_IMU, // Adaptive R_IMU approx sample depth
        unsigned L_IMU, // Adaptive R_IMU update frequency
        unsigned N_Q, // Adaptive Q approx sample depth
        unsigned L_Q // Adaptive Q update frequency
    );

    /**
     * @brief Single shot multiple sensor context aware Kalman Filter approximation.
     *        This overloaded method assumes no GPS measurement. This implies Beta GPS = 0.
     *        Potentially prone to IMU overfitting.
     * 
     * @param dt - delta time from previous IMU innovation
     * @param z_IMU - 4 state IMU measurement column vector, [vx, vy, ax, ay]^T
     * 
     * @return {x(k|k), P(k|k)} - pseudo-fused postiori x and P
     */
    std::pair<Vector6d, Matrix6d> Step(double dt, Vector6d& z_IMU);

    /**
     * @brief Single shot multiple sensor context aware Kalman Filter approximation.
     *        This overloaded method assumes GPS measurement. This implies Beta GPS is nonzero.
     * 
     * @param [in] dt - delta time from previous IMU innovation
     * @param [in] z_GPS - 2 state GPS measurement column vector, [x, y, 0, 0, 0, 0]^T
     * @param [in] z_IMU - 4 state IMU measurement column vector, [0, 0, vx, vy, ax, ay]^T
     * 
     * @return {x(k|k), P(k|k)} - pseudo-fused postiori x and P
     * 
     * @remarks
     * 
     * @exception
     */
    std::pair<Vector6d, Matrix6d> Step(double dt, Vector6d& z_GPS, Vector6d& z_IMU);

    /**
     * @brief Constructs the state transition matrix. It is a constant acceleration
     *        block matrix assuming [x, y, vx, vy, ax, ay]^T state kinematics
     * 
     * @param [in] dt - delta time from previous innovation
     * 
     * @return F_k
     * 
     * @remarks
     * 
     * @exception
     */
    Matrix6d BuildFk(double dt);

    /**
     * @brief Calculates the weighting of the GPS and IMU filter for fusion. Beta GPS is 
     *        assumed 0 when no GPS measurement is online.
     * 
     * @param [in] priori_P - P(k|k-1)
     * @param [in] innovation_IMU - v_GPS(k)
     * 
     * @return B0, B_GPS, B_IMU, B_GPS+IMU
     * 
     * @remarks
     * 
     * @exception
     */
    Eigen::Vector4d CalculateBetas(Matrix6d priori_P, Vector6d innovation_IMU);

    /**
     * @brief Calculates the weighting of the GPS and IMU filter for fusion. Beta GPS is 
     *        assumed nonzero when GPS measurement is online. Reference equations 18 and 19.
     * 
     * @param [in] priori_P - P(k|k-1)
     * @param [in] innovation_GPS - v_GPS(k)
     * @param [in] innovation_IMU - v_IMU(k)
     * 
     * @return B0, B_GPS, B_IMU, B_GPS+IMU
     * 
     * @remarks
     * 
     * @exception
     */
    Eigen::Vector4d CalculateBetas(Matrix6d priori_P, Vector6d innovation_GPS, Vector6d innovation_IMU);

    /**
     * @brief Calculate GPS Kalman Gains.
     * 
     * @param [in] priori_P_inv - P(k|k-1)^-1
     * @param [in] innovation_GPS - z_GPS(k) - H_GPS * x(k|k-1)
     * @param [in] postiori_P_GPS_IMU - P(k|k-1)^-1 + H_GPS^T * R_GPS^-1 * H_GPS + H_IMU^T * R_IMU^-1 * H_IMU
     * 
     * @return K_GPS(k), K_{GPS|(GPS+IMU)}
     * 
     * @remarks
     * 
     * @exception
     */
    std::pair<Matrix6d, Matrix6d> Calculate_GPS_KalmanGains(Matrix6d priori_P_inv, Vector6d innovation_GPS, Matrix6d postiori_P_GPS_IMU);

    /**
     * @brief Calculate IMU Kalman Gains.
     * 
     * @param [in] priori_P_inv - P(k|k-1)^-1
     * @param [in] innovation_IMU - z_IMU(k) - H_IMU * x(k|k-1)
     * @param [in] postiori_P_GPS_IMU - P(k|k-1)^-1 + H_GPS^T * R_GPS^-1 * H_GPS + H_IMU^T * R_IMU^-1 * H_IMU
     * 
     * @return K_IMU(k), K_{IMU|(GPS+IMU)}
     * 
     * @remarks
     * 
     * @exception
     */
    std::pair<Matrix6d, Matrix6d> Calculate_IMU_KalmanGains(Matrix6d priori_P_inv, Vector6d innovation_IMU, Matrix6d postiori_P_GPS_IMU);

    /**
     * @brief Update R_GPS(k) through analytical methods.
     * 
     * @param [in] R_GPS - R_GPS(k)
     * 
     * @return 
     * 
     * @remarks
     * 
     * @exception
     */
    void Update_GPS_R(Matrix6d R_GPS);

    /**
     * @brief Update R_IMU(k) through analytical methods.
     * 
     * @param [in] R_IMU - R_IMU(k)
     * 
     * @return 
     * 
     * @remarks
     * 
     * @exception
     */
    void Update_IMU_R(Matrix6d R_IMU);

    /**
     * @brief Update Q(k) through analytical methods.
     * 
     * @param [in] dt - delta time from previous innovation.
     * @param [in] has_GPS - If GPS is available, change Q calculation.
     * 
     * @return 
     * 
     * @remarks
     * 
     * @exception
     */
    void Update_Q(double dt, bool has_GPS);

private:

    /**
     * @brief Calculation of Fig 4 fuzzy logic to determin membership functions mu GPS and mu IMU.
     * 
     * @param [in] chi_squared_stat - y(k)^T S(k)^-1 y(k), otherwise known as the NIS calculation.
     * @param [in] is_GPS - Determines which membership value to calculate. 
     * 
     * @return mu_{GPS, IMU}
     * 
     * @remarks
     * 
     * @exception
     */
    double CalculateFuzzyLogicMembershipFunction(double chi_squared_stat, bool is_GPS);

    /**
     * @brief Update GPS adaptive filter queue.
     * 
     * @param [in] innovation_GPS - z_IMU(k) - H_IMU * x_IMU(k|k-1)
     * @param [in] priori_P_GPS - P_GPS(k|k)
     * 
     * @return
     * 
     * @remarks
     * 
     * @exception
     */
    void PushInnovationGPS(Vector6d &innovation_GPS, Matrix6d &postiori_P_GPS);

    /**
     * @brief Update IMU adaptive filter queue.
     * 
     * @param [in] innovation_IMU - z_GPS(k) - H_GPS * x_GPS(k|k-1)
     * @param [in] priori_P_IMU - P_IMU(k|k)
     * 
     * @return
     * 
     * @remarks
     * 
     * @exception
     */
    void PushInnovationIMU(Vector6d &innovation_IMU, Matrix6d &postiori_P_IMU);

    /**
     * @brief Update Q adaptive filter queue.
     * 
     * @param [in] posteriorResidual - K(k) * (z_{GPS+IMU}(k) - x(k|k-1))
     * @param [in] postiori_P - P(k|k)
     * 
     * @return
     * 
     * @remarks
     * 
     * @exception
     */
    void PushInnovationQ(Vector6d &posteriorResidual, Matrix6d &postiori_P);

private:
    // Fusion filter members
    Vector6d m_x; 
    Matrix6d m_P; 
    Matrix6d m_R_GPS;
    Matrix6d m_R_IMU;
    Matrix6d m_Q;
    Matrix6d m_H_GPS;
    Matrix6d m_H_IMU;
    Matrix6d m_P_propagated_lag;
    double m_chiSquaredBetaLowerBound_IMU;
    double m_chiSquaredBetaLowerBound_GPS;
    double m_chiSquaredBetaUpperBound_IMU;
    double m_chiSquaredBetaUpperBound_GPS;
    Eigen::Matrix<double, 2, 2> m_I;
    Eigen::Matrix<double, 2, 2> m_zero;
    double m_jerkPSD;

    // Adaptive filter members
    using AlignedVector6dDeque = std::deque<Vector6d, Eigen::aligned_allocator<Vector6d>>;

    unsigned m_N_GPS;
    unsigned m_L_GPS;
    unsigned m_l_GPS;
    unsigned m_N_IMU;
    unsigned m_L_IMU;
    unsigned m_l_IMU;
    unsigned m_N_Q;
    unsigned m_L_Q;
    unsigned m_l_Q;

    AlignedVector6dDeque m_innocationQueue_GPS;
    AlignedVector6dDeque m_innocationQueue_IMU;
    AlignedVector6dDeque m_posteriorResidualQueue;
};

