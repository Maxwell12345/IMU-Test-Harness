#include <iostream>

#include "YamlConfigService.hpp"
#include "yaml-cpp/yaml.h"

YamlConfigService::YamlConfigService(std::string path) {
    YAML::Node node = YAML::LoadFile(path);
    m_yamlConfig.FromNode(node);
    std::cout << "[INFO] Successfully read " << path << std::endl; 
}

YamlConfig YamlConfigService::GetConfig() const {
    return m_yamlConfig;
}