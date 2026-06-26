#include <gtest/gtest.h>

#include "YamlConfigService.hpp"

TEST(YamlConfigServiceTest, ConstructorThrowsRuntimeError) {
    EXPECT_ANY_THROW(YamlConfigService service("nonexistence.yaml"));
}

TEST(YamlConfigServiceTest, ConstructorDoesNotThrow) {
    EXPECT_NO_THROW(YamlConfigService service("config.yaml"));
}

TEST(YamlConfigServiceTest, GetConfigReturnsConfig) {
    YamlConfigService service("config.yaml");
    auto config = service.GetConfig();

    EXPECT_DOUBLE_EQ(0.20, config.kalmanValues.gpsChiSqLowerBound);
    EXPECT_DOUBLE_EQ(0.95, config.kalmanValues.gpsChiSqUpperBound);
    EXPECT_DOUBLE_EQ(0.20, config.kalmanValues.imuChiSqLowerBound);
    EXPECT_DOUBLE_EQ(0.95, config.kalmanValues.imuChiSqUpperBound);
}

TEST(YamlConfigTest, TopLevelMissingKalmanValuesThrows) {
    YAML::Node node = YAML::Load(R"(
        not_kalman_values:
          gps_chi_sq_lower_bound: 0.20
          gps_chi_sq_upper_bound: 0.95
          imu_chi_sq_lower_bound: 0.20
          imu_chi_sq_upper_bound: 0.95
    )");

    YamlConfig config;
    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}

TEST(YamlConfigTest, MissingRequiredFieldThrows) {
    YAML::Node node = YAML::Load(R"(
        kalman_values:
          gps_chi_sq_lower_bound: 0.20
          gps_chi_sq_upper_bound: 0.95
          imu_chi_sq_lower_bound: 0.20
    )");

    YamlConfig config;
    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}

TEST(YamlConfigTest, ValidConfigParsesCorrectly) {
    YAML::Node node = YAML::Load(R"(
        kalman_values:
          gps_chi_sq_lower_bound: 0.20
          gps_chi_sq_upper_bound: 0.95
          imu_chi_sq_lower_bound: 0.20
          imu_chi_sq_upper_bound: 0.95
    )");

    YamlConfig config;
    EXPECT_NO_THROW(config.FromNode(node));

    EXPECT_DOUBLE_EQ(0.20, config.kalmanValues.gpsChiSqLowerBound);
    EXPECT_DOUBLE_EQ(0.95, config.kalmanValues.gpsChiSqUpperBound);
    EXPECT_DOUBLE_EQ(0.20, config.kalmanValues.imuChiSqLowerBound);
    EXPECT_DOUBLE_EQ(0.95, config.kalmanValues.imuChiSqUpperBound);
}

TEST(YamlConfigTest, EmptyNodeThrows) {
    YAML::Node node;
    YamlConfig config;

    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}