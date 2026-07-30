#ifndef YAML_CONFIG_HPP
#define YAML_CONFIG_HPP

#include "yaml-cpp/yaml.h"

#include <string>

struct _KalmanValues {
    unsigned int gpsN;
    unsigned int gpsL;
    double gpsChiSqLowerBound;
    double gpsChiSqUpperBound;
    unsigned int imuN;
    unsigned int imuL;
    double imuChiSqLowerBound;
    double imuChiSqUpperBound;

    void FromNode(YAML::Node node) {
        std::string requiredFields[] = {
            "gps_n",
            "gps_l",
            "gps_chi_sq_lower_bound",
            "gps_chi_sq_upper_bound",
            "imu_n",
            "imu_l",
            "imu_chi_sq_lower_bound",
            "imu_chi_sq_upper_bound"
        };

        for(auto& f : requiredFields) {
            if(!node[f]) {
                throw std::invalid_argument("Yaml kalman_values is missing field: \"" + f + "\"");
            }
        }

        gpsN = node["gps_n"].as<unsigned int>();
        gpsL = node["gps_l"].as<unsigned int>();
        gpsChiSqLowerBound = node["gps_chi_sq_lower_bound"].as<double>();
        gpsChiSqUpperBound = node["gps_chi_sq_upper_bound"].as<double>();
        imuN = node["imu_n"].as<unsigned int>();
        imuL = node["imu_l"].as<unsigned int>();
        imuChiSqLowerBound = node["imu_chi_sq_lower_bound"].as<double>();
        imuChiSqUpperBound = node["imu_chi_sq_upper_bound"].as<double>();
    }
};

struct _ImuSerialPort {
    std::string path;
    unsigned int baudRate;

    void FromNode(YAML::Node node) {
        std::string requiredFields[] = {
            "path",
            "baud_rate"
        };

        for (auto& f : requiredFields) {
            if (!node[f]) {
                throw std::invalid_argument("Yaml imu_serial_port is missing field: \"" + f + "\"");
            }
        }

        path = node["path"].as<std::string>();
        baudRate = node["baud_rate"].as<unsigned int>();
    }

    std::string ToString() const {
        char buff[256];
        const char* format =
            "\tpath: %s\n"
            "\tbaudRate: %u\n";

        snprintf(buff, sizeof(buff), format,
                 path.c_str(),
                 baudRate);

        return std::string(buff);
    }
};

struct YamlConfig {
    _KalmanValues kalmanValues;
    _ImuSerialPort imuSerialPort;

    void FromNode(YAML::Node node) {
        std::string requiredFields[] = {
            "kalman_values",
            "imu_serial_port"
        };

        for (auto& f : requiredFields) {
            if (!node[f]) {
                throw std::invalid_argument("Yaml file top level is missing field: \"" + f + "\"");
            }
        }

        kalmanValues.FromNode(node["kalman_values"]);
        imuSerialPort.FromNode(node["imu_serial_port"]);
    }
};

#endif