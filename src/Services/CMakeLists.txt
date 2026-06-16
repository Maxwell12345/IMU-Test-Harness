cmake_minimum_required(VERSION 3.15)

project(GPS_SERVICE LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

find_package(GTest CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS filesystem)

file(GLOB_RECURSE PROJ_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

list(REMOVE_ITEM PROJ_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
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
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_link_libraries(${PROJECT_NAME}_app
    PRIVATE
        ${PROJECT_NAME}
        Boost::filesystem
)