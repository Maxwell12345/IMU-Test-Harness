set(TARGET_NAME networking)

find_package(Boost CONFIG)
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    find_package(Boost CONFIG REQUIRED system)
endif()

add_library(${TARGET_NAME} STATIC
    MulticastService.cpp
)

if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/include)
    target_link_libraries(${TARGET_NAME} PRIVATE Boost::system)
endif()
