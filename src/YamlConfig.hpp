#ifndef YAML_CONFIG_HPP
#define YAML_CONFIG_HPP

#include "yaml-cpp/yaml.h"

#include <string>

struct _KalmanValues {
    double gpsChiSqLowerBound;
    double gpsChiSqUpperBound;
    double imuChiSqLowerBound;
    double imuChiSqUpperBound;

    void FromNode(YAML::Node node) {
        std::string requiredFields[] = {
            "gps_chi_sq_lower_bound",
            "gps_chi_sq_upper_bound",
            "imu_chi_sq_lower_bound",
            "imu_chi_sq_upper_bound"
        };

        for(auto& f : requiredFields) {
            if(!node[f]) {
                throw std::invalid_argument("Yaml nmea_config is missing field: \"" + f + "\"");
            }
        }

        gpsChiSqLowerBound = node["gps_chi_sq_lower_bound"].as<double>();
        gpsChiSqUpperBound = node["gps_chi_sq_upper_bound"].as<double>();
        imuChiSqLowerBound = node["imu_chi_sq_lower_bound"].as<double>();
        imuChiSqUpperBound = node["imu_chi_sq_upper_bound"].as<double>();
    }

    std::string ToString() {
        char buff[256];
        std::string format = "\tgpsChiSqLowerBound: %f\n"
                             "\tgpsChiSqUppderBound: %f\n"
                             "\timuChiSqLowerBound: %f\n"
                             "\timuChiSqUppderBound: %f\n";
        sprintf(buff, format.c_str(), gpsChiSqLowerBound,
                                      gpsChiSqUpperBound,
                                      imuChiSqLowerBound,
                                      imuChiSqUpperBound);
        return std::string(buff);
    }
};

struct YamlConfig {
    _KalmanValues kalmanValues;

    void FromNode(YAML::Node node) {
        if(!node["kalman_values"]) {
            throw std::invalid_argument("Yaml top level is missing field \"kalman_values\"");
        }

        kalmanValues.FromNode(node["kalman_values"]);
    }

    std::string ToString() {
        return "kalmanValues:\n" + kalmanValues.ToString();
    }
};

#endif