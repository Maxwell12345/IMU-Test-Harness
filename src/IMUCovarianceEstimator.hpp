#ifndef IMU_COVARIANCE_ESTIMATOR_HPP
#define IMU_COVARIANCE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <vector>

class IMUCovarianceEstimator {
public:
    IMUCovarianceEstimator() = default;

    IMUCovarianceEstimator(const Eigen::Matrix2d& initialR, std::size_t windowSize, std::size_t minimumInliers);

    Eigen::Matrix2d GetR(double acceleration, double dYaw);

private:
    static double Median(std::vector<double> values);

    Eigen::Vector2d CalculateMedian() const;
    
    Eigen::Vector2d CalculateMadSigma(const Eigen::Vector2d& center) const;
    
    static double WrapYawChange(double dYaw);

private:
    Eigen::Matrix2d m_initialR;
    Eigen::Matrix2d m_R;

    std::deque<Eigen::Vector2d> m_samples;

    std::size_t m_windowSize;
    std::size_t m_minimumInliers;
};

#endif // IMU_COVARIANCE_ESTIMATOR_HPP
