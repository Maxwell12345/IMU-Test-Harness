set(TARGET_NAME yaml)

find_package(yaml-cpp CONFIG REQUIRED)

add_library(${TARGET_NAME} STATIC
    YamlConfigService.cpp
)

if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/include)
    target_link_libraries(${TARGET_NAME} PUBLIC
        yaml-cpp::yaml-cpp
    )
elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(${TARGET_NAME} PUBLIC
        yaml-cpp
    )
endif()