set(TARGET_NAME nmea)

add_library(${TARGET_NAME} STATIC
    NmeaService.cpp
)

target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/Serial
    ${CMAKE_SOURCE_DIR}/src/Networking
)

target_link_libraries(${TARGET_NAME} PRIVATE
    serial
    networking
)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_libraries(${TARGET_NAME} PRIVATE
        atomic
    )
endif()