cmake_minimum_required(VERSION 3.15)
project(NMEA LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest CONFIG REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/src/Serial)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/Nmea)
add_subdirectory(${CMAKE_SOURCE_DIR}/src/Yaml)

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/Serial
    ${CMAKE_SOURCE_DIR}/src/Nmea
    ${CMAKE_SOURCE_DIR}/src/Yaml
)

target_link_libraries(${PROJECT_NAME} 
    PRIVATE
        serial
        nmea
        yaml)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi)
endif()


####################################################################


enable_testing()

file(GLOB_RECURSE TEST_SRC "${CMAKE_SOURCE_DIR}/test/*.cpp")
add_executable(${PROJECT_NAME}_tests
    ${TEST_SRC}
)

target_include_directories(${PROJECT_NAME}_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src/Serial
    ${CMAKE_SOURCE_DIR}/src/Nmea
    ${CMAKE_SOURCE_DIR}/src/Yaml
)

target_link_libraries(${PROJECT_NAME}_tests
    PRIVATE
        serial
        nmea
        yaml
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        GTest::gmock_main
)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(${PROJECT_NAME}_tests PRIVATE atomic)
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME}_tests PRIVATE -Wno-psabi)
endif()

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
