cmake_minimum_required(VERSION 3.15)
project(DATA_COLLECTION LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SH2_DIR ${CMAKE_SOURCE_DIR}/../third_party/sh2)
set(PROJ_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

add_library(sh2 STATIC
    ${SH2_DIR}/sh2.c
    ${SH2_DIR}/sh2_util.c
    ${SH2_DIR}/shtp.c
    ${SH2_DIR}/sh2_SensorValue.c
)

target_include_directories(sh2 PUBLIC ${SH2_DIR} ${PROJ_INCLUDE_DIR})

find_library(I2C_LIB i2c)
find_package(SQLite3 REQUIRED)
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/bno085_hal.cpp
    ${CMAKE_SOURCE_DIR}/src/Logger.cpp
    ${CMAKE_SOURCE_DIR}/src/UsbSerialLocator.cpp
    ${CMAKE_SOURCE_DIR}/src/SqliteManager.cpp
    ${CMAKE_SOURCE_DIR}/src/BrokerService.cpp
    ${CMAKE_SOURCE_DIR}/src/ImuService.cpp
    ${CMAKE_SOURCE_DIR}/src/GpsService.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJ_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE
    sh2
    SQLite::SQLite3
    Threads::Threads
)

if(I2C_LIB)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${I2C_LIB})
endif()
