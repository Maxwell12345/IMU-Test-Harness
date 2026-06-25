#ifndef YAML_CONFIG_HPP
#define YAML_CONFIG_HPP

#include "yaml-cpp/yaml.h"

#include <string>

struct _NmeaConfig {
    std::string serialPortPath = "";
    unsigned int serialPortBaudRate = 9600;
    std::string multicastIp = "";
    unsigned int multicastPort = 0;

    void FromNode(YAML::Node node) {
        std::string requiredFields[] = {
            "serial_port_path",
            "serial_port_baud_rate",
            "multicast_ip",
            "multicast_port"
        };

        for(auto& f : requiredFields) {
            if(!node[f]) {
                throw std::invalid_argument("Yaml nmea_config is missing field: \"" + f + "\"");
            }
        }

        serialPortPath = node["serial_port_path"].as<std::string>();
        serialPortBaudRate = node["serial_port_baud_rate"].as<unsigned int>();
        multicastIp = node["multicast_ip"].as<std::string>();
        multicastPort = node["multicast_port"].as<unsigned int>();
    }

    std::string ToString() {
        char buff[256];
        std::string format = "\tserialPortPath: %s\n"
                             "\tserialPortBaudRate: %d\n"
                             "\tmulticastIp: %s\n"
                             "\tmulticastPort: %d\n";
        sprintf(buff, format.c_str(), serialPortPath.c_str(),
                                      serialPortBaudRate,
                                      multicastIp.c_str(),
                                      multicastPort);
        return std::string(buff);
    }
};

struct YamlConfig {
    _NmeaConfig nmeaConfig;

    void FromNode(YAML::Node node) {
        if(!node["nmea_config"]) {
            throw std::invalid_argument("Yaml top level is missing field \"nmea_config\"");
        }

        nmeaConfig.FromNode(node["nmea_config"]);
    }

    std::string ToString() {
        return "nmeaConfig:\n" + nmeaConfig.ToString();
    }
};

#endif