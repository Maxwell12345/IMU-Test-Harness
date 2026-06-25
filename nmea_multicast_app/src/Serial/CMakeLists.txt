set(TARGET_NAME serial)

find_package(Boost CONFIG REQUIRED filesystem regex atomic)
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    find_package(Boost CONFIG REQUIRED system)
endif()

add_library(${TARGET_NAME} STATIC
    BoostSerialPort.cpp
    SerialComService.cpp
)

target_link_libraries(${TARGET_NAME} PRIVATE
    Boost::regex
    Boost::filesystem
)

if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_link_libraries(${TARGET_NAME} PRIVATE Boost::system)
    target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/include)
endif()
