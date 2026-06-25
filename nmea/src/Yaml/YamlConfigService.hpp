#ifndef YAML_CONFIG_SERVICE_HPP
#define YAML_CONFIG_SERVICE_HPP

#include <string>

#include "YamlConfig.hpp"

class YamlConfigService {
public:
    YamlConfigService(std::string path);

    YamlConfig GetConfig() const;
private:
    YamlConfig m_yamlConfig;    
};

#endif