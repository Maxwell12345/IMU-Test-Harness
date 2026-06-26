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

    EXPECT_EQ(20u, config.kalmanValues.gpsN);
    EXPECT_EQ(5u, config.kalmanValues.gpsL);
    EXPECT_DOUBLE_EQ(0.20, config.kalmanValues.gpsChiSqLowerBound);
    EXPECT_DOUBLE_EQ(0.95, config.kalmanValues.gpsChiSqUpperBound);

    EXPECT_EQ(100u, config.kalmanValues.imuN);
    EXPECT_EQ(10u, config.kalmanValues.imuL);
    EXPECT_DOUBLE_EQ(0.20, config.kalmanValues.imuChiSqLowerBound);
    EXPECT_DOUBLE_EQ(0.95, config.kalmanValues.imuChiSqUpperBound);

    EXPECT_EQ(100u, config.kalmanValues.qN);
    EXPECT_EQ(10u, config.kalmanValues.qL);

    EXPECT_EQ("\\\\COM1", config.imuSerialPort.path);
    EXPECT_EQ(460800u, config.imuSerialPort.baudRate);
}

TEST(YamlConfigTest, TopLevelMissingKalmanValuesThrows) {
    YAML::Node node = YAML::Load(R"(
        imu_serial_port:
          path: \\COM1
          baud_rate: 460800
    )");

    YamlConfig config;
    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}

TEST(YamlConfigTest, TopLevelMissingImuSerialPortThrows) {
    YAML::Node node = YAML::Load(R"(
        kalman_values:
          gps_n: 20
          gps_l: 5
          gps_chi_sq_lower_bound: 0.20
          gps_chi_sq_upper_bound: 0.95
          imu_n: 100
          imu_l: 10
          imu_chi_sq_lower_bound: 0.20
          imu_chi_sq_upper_bound: 0.95
          q_n: 100
          q_l: 10
    )");

    YamlConfig config;
    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}

TEST(YamlConfigTest, MissingRequiredFieldThrows) {
    YAML::Node node = YAML::Load(R"(
        kalman_values:
          gps_n: 20
          gps_l: 5
          gps_chi_sq_lower_bound: 0.20
          gps_chi_sq_upper_bound: 0.95
          imu_n: 100
          imu_l: 10
          imu_chi_sq_lower_bound: 0.20
          imu_chi_sq_upper_bound: 0.95
          q_n: 100

        imu_serial_port:
          path: \\COM1
          baud_rate: 460800
    )");

    YamlConfig config;
    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}

TEST(YamlConfigTest, ValidConfigParsesCorrectly) {
    YAML::Node node = YAML::Load(R"(
        kalman_values:
          gps_n: 20
          gps_l: 5
          gps_chi_sq_lower_bound: 0.20
          gps_chi_sq_upper_bound: 0.95
          imu_n: 100
          imu_l: 10
          imu_chi_sq_lower_bound: 0.20
          imu_chi_sq_upper_bound: 0.95
          q_n: 100
          q_l: 10

        imu_serial_port:
          path: \\COM1
          baud_rate: 460800
    )");

    YamlConfig config;
    EXPECT_NO_THROW(config.FromNode(node));

    EXPECT_EQ(20u, config.kalmanValues.gpsN);
    EXPECT_EQ(5u, config.kalmanValues.gpsL);
    EXPECT_DOUBLE_EQ(0.20, config.kalmanValues.gpsChiSqLowerBound);
    EXPECT_DOUBLE_EQ(0.95, config.kalmanValues.gpsChiSqUpperBound);

    EXPECT_EQ(100u, config.kalmanValues.imuN);
    EXPECT_EQ(10u, config.kalmanValues.imuL);
    EXPECT_DOUBLE_EQ(0.20, config.kalmanValues.imuChiSqLowerBound);
    EXPECT_DOUBLE_EQ(0.95, config.kalmanValues.imuChiSqUpperBound);

    EXPECT_EQ(100u, config.kalmanValues.qN);
    EXPECT_EQ(10u, config.kalmanValues.qL);

    EXPECT_EQ("\\\\COM1", config.imuSerialPort.path);
    EXPECT_EQ(460800u, config.imuSerialPort.baudRate);
}

TEST(YamlConfigTest, EmptyNodeThrows) {
    YAML::Node node;
    YamlConfig config;

    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}