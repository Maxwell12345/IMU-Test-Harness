cmake_minimum_required(VERSION 3.15)
project(IMUPROC LANGUAGES CXX C)

option(SIMULATION_MODE "Build the recorded-sensor simulation entry point" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "CXX compiler: ${CMAKE_CXX_COMPILER}")

find_package(Eigen3 CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED filesystem regex atomic)
find_package(SQLiteCpp REQUIRED)
find_package(yaml-cpp REQUIRED)
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    find_package(Boost CONFIG REQUIRED system)
endif()

file(GLOB_RECURSE PROJ_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
list(FILTER PROJ_SRC EXCLUDE REGEX ".*/main2?\\.cpp$")

add_library(IMUPROC_LIB
	${PROJ_SRC}
)

target_include_directories(IMUPROC_LIB PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/vcpkg_installed/x64-windows/include)
target_link_libraries(IMUPROC_LIB
    PUBLIC
        Eigen3::Eigen
        SQLiteCpp
        Boost::regex
        Boost::filesystem)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(IMUPROC_LIB PUBLIC atomic yaml-cpp)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_link_libraries(IMUPROC_LIB PUBLIC yaml-cpp::yaml-cpp Boost::system)
endif()

if(SIMULATION_MODE)
    set(IMUPROC_MAIN "${CMAKE_SOURCE_DIR}/src/main2.cpp")
else()
    set(IMUPROC_MAIN "${CMAKE_SOURCE_DIR}/src/main.cpp")
endif()

add_executable(${PROJECT_NAME} ${IMUPROC_MAIN})

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        IMUPROC_LIB
)

file(GLOB_RECURSE TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

enable_testing()
add_executable(${PROJECT_NAME}_tests
    ${TEST_SRC}
)
target_link_libraries(${PROJECT_NAME}_tests
    PRIVATE
        IMUPROC_LIB
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
