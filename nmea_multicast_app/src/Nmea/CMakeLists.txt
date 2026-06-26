set(TARGET_NAME nmea)

find_package(Boost CONFIG REQUIRED regex)

add_library(${TARGET_NAME} STATIC
    NmeaService.cpp
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/Serial
    ${CMAKE_SOURCE_DIR}/src/Networking
)

target_link_libraries(${TARGET_NAME} PRIVATE
    serial
    networking
    Boost::regex
)

if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/include)
endif()

