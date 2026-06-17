cmake_minimum_required(VERSION 3.15)

project(GPS_SERVICE LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

find_package(GTest CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS filesystem)

file(GLOB_RECURSE PROJ_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

list(REMOVE_ITEM PROJ_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
)

add_library(${PROJECT_NAME} ${PROJ_SRC})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_SOURCE_DIR}
        ${Boost_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        -Wno-psabi
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        atomic
)

add_executable(${PROJECT_NAME}_app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_app
    PRIVATE
        ${PROJECT_NAME}
        Boost::filesystem
)

file(GLOB_RECURSE TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

enable_testing()
add_executable(${PROJECT_NAME}_tests
    ${TEST_SRC}
    ${PROJ_SRC}
)

target_include_directories(${PROJECT_NAME}_tests
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_tests
    PRIVATE
         GTest::gtest
         GTest::gtest_main
         GTest::gmock
         GTest::gmock_main
         atomic
         SQLiteCpp
)
target_compile_options(${PROJECT_NAME}_tests PRIVATE -Wno-psabi)

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