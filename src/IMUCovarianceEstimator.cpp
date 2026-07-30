#include "IMUCovarianceEstimator.hpp"

IMUCovarianceEstimator::IMUCovarianceEstimator(const Eigen::Matrix2d& initialR, std::size_t windowSize, std::size_t minimumInliers)
    : m_initialR(0.5 * (initialR + initialR.transpose())), m_R(m_initialR), m_windowSize(windowSize), m_minimumInliers(minimumInliers) {
}

Eigen::Matrix2d IMUCovarianceEstimator::GetR(double acceleration, double dYaw) {
    if (!std::isfinite(acceleration) || !std::isfinite(dYaw)) {
        return this->m_R;
    }

    Eigen::Vector2d sample;
    sample << std::abs(acceleration), this->WrapYawChange(dYaw);

    this->m_samples.push_back(sample);

    if (this->m_samples.size() > this->m_windowSize) {
        this->m_samples.pop_front();
    }

    if (this->m_samples.size() < this->m_minimumInliers) {
        return this->m_R;
    }

    const Eigen::Vector2d median = this->CalculateMedian();
    Eigen::Vector2d sigma = this->CalculateMadSigma(median);

    sigma(0) = std::max(sigma(0), std::sqrt(this->m_initialR(0, 0)));

    sigma(1) = std::max(sigma(1), std::sqrt(this->m_initialR(1, 1)));

    std::vector<Eigen::Vector2d> inliers;
    inliers.reserve(this->m_samples.size());

    for (const Eigen::Vector2d& value : this->m_samples) {
        const Eigen::Vector2d deviation = (value - median).cwiseAbs();

        if (deviation(0) <= 4.0 * sigma(0) && deviation(1) <= 4.0 * sigma(1)) {
            inliers.push_back(value);
        }
    }

    if (inliers.size() < this->m_minimumInliers) {
        return this->m_R;
    }

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();

    for (const Eigen::Vector2d& value : inliers) {
        mean += value;
    }

    mean /= static_cast<double>(inliers.size());

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();

    for (const Eigen::Vector2d& value : inliers) {
        const Eigen::Vector2d residual = value - mean;
        covariance.noalias() += residual * residual.transpose();
    }

    covariance /= static_cast<double>(inliers.size() - 1);
    covariance = 0.5 * (covariance + covariance.transpose());

    if (covariance.allFinite()) {
        this->m_R = covariance;
    }

    return this->m_R;
}

double IMUCovarianceEstimator::Median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }

    std::sort(values.begin(), values.end());

    const std::size_t middle = values.size() / 2;

    if ((values.size() & 1U) != 0U) {
        return values[middle];
    }

    return 0.5 * (values[middle - 1] + values[middle]);
}

Eigen::Vector2d IMUCovarianceEstimator::CalculateMedian() const {
    std::vector<double> accelerationValues;
    std::vector<double> yawValues;

    accelerationValues.reserve(this->m_samples.size());
    yawValues.reserve(this->m_samples.size());

    for (const Eigen::Vector2d& sample : this->m_samples) {
        accelerationValues.push_back(sample(0));
        yawValues.push_back(sample(1));
    }

    return Eigen::Vector2d(IMUCovarianceEstimator::Median(accelerationValues), IMUCovarianceEstimator::Median(yawValues));
}

Eigen::Vector2d IMUCovarianceEstimator::CalculateMadSigma(const Eigen::Vector2d& center) const {

    std::vector<double> accelerationDeviations;
    std::vector<double> yawDeviations;

    accelerationDeviations.reserve(this->m_samples.size());
    yawDeviations.reserve(this->m_samples.size());

    for (const Eigen::Vector2d& sample : this->m_samples) {
        accelerationDeviations.push_back(std::abs(sample(0) - center(0)));

        yawDeviations.push_back(std::abs(sample(1) - center(1)));
    }

    constexpr double madToSigma = 1.482602218505602;

    return Eigen::Vector2d(madToSigma * IMUCovarianceEstimator::Median(accelerationDeviations), madToSigma * IMUCovarianceEstimator::Median(yawDeviations));
}

double IMUCovarianceEstimator::WrapYawChange(double dYaw) {
    constexpr double pi = 3.141592653589793238462643383279502884;

    return std::remainder(dYaw, 2.0 * pi);
}
