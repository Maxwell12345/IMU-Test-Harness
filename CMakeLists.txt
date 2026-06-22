cmake_minimum_required(VERSION 3.15)
project(IMUPROC LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "CXX compiler: ${CMAKE_CXX_COMPILER}")

find_package(Eigen3 CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED filesystem system)
find_package(SQLiteCpp REQUIRED)

file(GLOB_RECURSE PROJ_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
file(GLOB_RECURSE PROJ_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")

add_library(IMUPROC_LIB
		${PROJ_SRC}
)

target_include_directories(IMUPROC_LIB PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/vcpkg_installed/x64-windows/include)
target_link_libraries(IMUPROC_LIB PUBLIC Eigen3::Eigen SQLiteCpp)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(IMUPROC_LIB PUBLIC atomic)
endif()

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME} 
    PRIVATE
        Boost::filesystem
        Boost::system
        IMUPROC_LIB
        SQLiteCpp)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(IMUPROC_LIB PRIVATE -Wno-psabi)
endif()

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
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(${PROJECT_NAME}_tests PUBLIC atomic)
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
