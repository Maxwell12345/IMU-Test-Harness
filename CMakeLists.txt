cmake_minimum_required(VERSION 3.15)
project(IMUPROC LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "CXX compiler: ${CMAKE_CXX_COMPILER}")

find_package(Eigen3 CONFIG REQUIRED)

set(SH2_DIR ${CMAKE_SOURCE_DIR}/third_party/sh2)

add_library(sh2 STATIC
    ${SH2_DIR}/sh2.c
    ${SH2_DIR}/sh2_util.c
    ${SH2_DIR}/shtp.c
	${SH2_DIR}/sh2_SensorValue.c
)
target_include_directories(sh2 PUBLIC ${SH2_DIR})

find_library(I2C_LIB i2c REQUIRED)

add_executable(bno085_reader
	${CMAKE_SOURCE_DIR}/src/main.cpp
	${CMAKE_SOURCE_DIR}/src/demo.cpp
    ${CMAKE_SOURCE_DIR}/src/demo2.cpp
	${CMAKE_SOURCE_DIR}/src/bno085_hal.cpp
)

target_link_libraries(bno085_reader
    PRIVATE
        sh2
        ${I2C_LIB}
        # your vcpkg deps here, e.g. fmt::fmt
)

# enable_testing()
# add_executable(${PROJECT_NAME}_tests
#     ${CMAKE_SOURCE_DIR}/tests/test_IMUGPSFusionKF.cpp
# )
# target_link_libraries(${PROJECT_NAME}_tests
#     PRIVATE
#         IMUGPSFusionKF_lib
#         GTest::gtest
#         GTest::gtest_main
#         GTest::gmock
#         GTest::gmock_main
# )

# include(GoogleTest)
# gtest_discover_tests(${PROJECT_NAME}_tests)

# add_custom_command(TARGET ${PROJECT_NAME}_tests POST_BUILD
#     COMMAND ${PROJECT_NAME}_tests
#     WORKING_DIRECTORY $<TARGET_FILE_DIR:${PROJECT_NAME}_tests>
#     COMMENT "Running ${PROJECT_NAME}_tests"
# )
