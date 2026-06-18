cmake_minimum_required(VERSION 3.15)
project(DATA_COLLECTION LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE PROJ_SRC "../${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE PROJ_HEADERS "../${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp")

set(SH2_DIR ../${CMAKE_SOURCE_DIR}/third_party/sh2)

add_library(sh2 STATIC
    ${SH2_DIR}/sh2.c
    ${SH2_DIR}/sh2_util.c
    ${SH2_DIR}/shtp.c
	${SH2_DIR}/sh2_SensorValue.c
)

target_include_directories(sh2 PUBLIC ${SH2_DIR})

find_library(I2C_LIB i2c REQUIRED)

add_library(DATA_COLLECTION_LIB
		${PROJ_SRC}
)

target_include_directories(DATA_COLLECTION_LIB PUBLIC ${CMAKE_SOURCE_DIR}/src ${SH2_DIR})

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE DATA_COLLECTION_LIB sh2 ${I2C_LIB})
target_compile_options(DATA_COLLECTION_LIB PRIVATE -Wno-psabi)

