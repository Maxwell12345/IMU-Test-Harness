#include <gtest/gtest.h>

#include "YamlConfigService.hpp"

TEST(YamlConfigServiceTest, ConstructorThrowsRuntimeError) {
    EXPECT_ANY_THROW(YamlConfigService service("nonexistence.yaml"));
}

TEST(YamlConfigServiceTest, ConstructorDoesNotThrow) {
    EXPECT_NO_THROW(YamlConfigService service("test_config.yaml"));
}

TEST(YamlConfigServiceTest, GetConfigReturnsConfig) {
    YamlConfigService service("test_config.yaml");
    auto config = service.GetConfig();
    EXPECT_EQ("/dev/ttyUSB0", config.nmeaConfig.serialPortPath);
    EXPECT_EQ(115200, config.nmeaConfig.serialPortBaudRate);
    EXPECT_EQ("224.0.0.100", config.nmeaConfig.multicastIp);
    EXPECT_EQ(6001, config.nmeaConfig.multicastPort);
}

TEST(YamlConfigTest, TopLevelMissingNmeaConfigThrows) {
    YAML::Node node = YAML::Load(R"(
        not_nmea_config:
          serial_port_path: /dev/ttyUSB0
          serial_port_baud_rate: 115200
          multicast_ip: 224.0.0.100
          multicast_port: 6001
    )");

    YamlConfig config;
    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}

TEST(YamlConfigTest, MissingRequiredFieldThrows) {
    YAML::Node node = YAML::Load(R"(
        nmea_config:
          serial_port_path: /dev/ttyUSB0
          serial_port_baud_rate: 115200
          multicast_ip: 224.0.0.100
    )");

    YamlConfig config;
    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}

TEST(YamlConfigTest, ValidConfigParsesCorrectly) {
    YAML::Node node = YAML::Load(R"(
        nmea_config:
          serial_port_path: \\.\COM1
          serial_port_baud_rate: 115200
          multicast_ip: 224.0.0.100
          multicast_port: 6001
    )");

    YamlConfig config;
    EXPECT_NO_THROW(config.FromNode(node));

    EXPECT_EQ(R"(\\.\COM1)", config.nmeaConfig.serialPortPath);
    EXPECT_EQ(115200u, config.nmeaConfig.serialPortBaudRate);
    EXPECT_EQ("224.0.0.100", config.nmeaConfig.multicastIp);
    EXPECT_EQ(6001u, config.nmeaConfig.multicastPort);
}

TEST(YamlConfigTest, EmptyNodeThrows) {
    YAML::Node node;
    YamlConfig config;

    EXPECT_THROW(config.FromNode(node), std::invalid_argument);
}