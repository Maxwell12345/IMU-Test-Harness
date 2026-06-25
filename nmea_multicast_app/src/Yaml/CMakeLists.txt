set(TARGET_NAME yaml)

find_package(yaml-cpp CONFIG REQUIRED)

add_library(${TARGET_NAME} STATIC
    YamlConfigService.cpp
)

target_link_libraries(${TARGET_NAME} PUBLIC
    yaml-cpp
)