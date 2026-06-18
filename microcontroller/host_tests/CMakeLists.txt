cmake_minimum_required(VERSION 3.16)

project(MICROCONTROLLER_HOST_TESTS LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

find_package(GTest REQUIRED)

set(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

set(SRC
    "${PROJECT_ROOT}/main/utils.cpp"
)

set(TEST_SRC
    "${PROJECT_ROOT}/test/utilstests.cpp"
)

add_executable(host_tests
    ${SRC}
    ${TEST_SRC}
)

target_include_directories(host_tests
    PRIVATE
    "${PROJECT_ROOT}/main"
)

target_link_libraries(host_tests
    PRIVATE
    GTest::gtest
    GTest::gtest_main
)

enable_testing()

include(GoogleTest)
gtest_discover_tests(host_tests)