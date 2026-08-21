scan_tracking_require_thickness_measure_v2_sdk()

add_library(scan_tracking_thickness_measure_v2 STATIC
    src/thickness_measure_v2_service.cpp
    include/scan_tracking/thickness_measure_v2/thickness_measure_v2_types.h
    include/scan_tracking/thickness_measure_v2/thickness_measure_v2_service.h
)

set_target_properties(scan_tracking_thickness_measure_v2 PROPERTIES
    FOLDER "modules"
)

target_include_directories(scan_tracking_thickness_measure_v2
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_thickness_measure_v2
    PUBLIC
        Qt5::Core
    PRIVATE
        ThicknessMeasureV2Sdk::Api
)

target_compile_features(scan_tracking_thickness_measure_v2 PUBLIC cxx_std_17)

add_executable(scan_tracking_thickness_measure_v2_smoke
    tests/thickness_measure_v2_smoke_runner.cpp
)

target_link_libraries(scan_tracking_thickness_measure_v2_smoke
    PRIVATE
        Qt5::Core
        scan_tracking_thickness_measure_v2
)

set_target_properties(scan_tracking_thickness_measure_v2_smoke PROPERTIES
    AUTOMOC ON
    FOLDER "tests"
)

scan_tracking_deploy_thickness_measure_v2_runtime(scan_tracking_thickness_measure_v2_smoke)
