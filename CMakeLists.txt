cmake_minimum_required(VERSION 3.15)
project(IMUPROC LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "CXX compiler: ${CMAKE_CXX_COMPILER}")

find_package(Eigen3 CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

file(GLOB_RECURSE PROJ_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE PROJ_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp")

set(SH2_DIR ${CMAKE_SOURCE_DIR}/third_party/sh2)

add_library(sh2 STATIC
    ${SH2_DIR}/sh2.c
    ${SH2_DIR}/sh2_util.c
    ${SH2_DIR}/shtp.c
	${SH2_DIR}/sh2_SensorValue.c
)
target_include_directories(sh2 PUBLIC ${SH2_DIR})

find_library(I2C_LIB i2c REQUIRED)

add_library(IMUPROC_LIB
		${PROJ_SRC}
)

target_include_directories(IMUPROC_LIB PUBLIC ${CMAKE_SOURCE_DIR}/src ${SH2_DIR})
target_link_libraries(IMUPROC_LIB PUBLIC Eigen3::Eigen)

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE IMUPROC_LIB sh2 ${I2C_LIB})

file(GLOB_RECURSE TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

 enable_testing()
 add_executable(${PROJECT_NAME}_tests
     ${TEST_SRC}
)
target_link_libraries(${PROJECT_NAME}_tests
    PRIVATE
         IMUPROC_LIB
         sh2
         ${I2C_LIB}
         GTest::gtest
         GTest::gtest_main
         GTest::gmock
         GTest::gmock_main
)

target_compile_definitions(${PROJECT_NAME}_tests PRIVATE
    TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test"
)

 include(GoogleTest)
 gtest_discover_tests(${PROJECT_NAME}_tests)

 add_custom_command(TARGET ${PROJECT_NAME}_tests POST_BUILD
     COMMAND ${PROJECT_NAME}_tests
     WORKING_DIRECTORY $<TARGET_FILE_DIR:${PROJECT_NAME}_tests>
     COMMENT "Running ${PROJECT_NAME}_tests"
 )
