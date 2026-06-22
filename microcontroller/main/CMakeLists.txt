set(SH2_DIR "${CMAKE_SOURCE_DIR}/third_party/sh2")

idf_component_register(
    SRCS
        "main.c"
        "${SH2_DIR}/sh2.c"
        "${SH2_DIR}/sh2_util.c"
        "${SH2_DIR}/shtp.c"
        "${SH2_DIR}/sh2_SensorValue.c"
    INCLUDE_DIRS
        "."
        "${SH2_DIR}"
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)