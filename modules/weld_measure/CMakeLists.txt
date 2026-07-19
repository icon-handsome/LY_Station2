scan_tracking_require_weld_measure_sdk()

add_library(scan_tracking_weld_measure STATIC
    src/weld_measure_service.cpp
    include/scan_tracking/weld_measure/weld_measure_types.h
    include/scan_tracking/weld_measure/weld_measure_service.h
)

set_target_properties(scan_tracking_weld_measure PROPERTIES
    FOLDER "modules"
)

target_include_directories(scan_tracking_weld_measure
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_weld_measure
    PUBLIC
        Qt5::Core
    PRIVATE
        WeldMeasureSdk::Api
)

target_compile_features(scan_tracking_weld_measure PUBLIC cxx_std_17)

add_executable(scan_tracking_weld_measure_smoke
    tests/weld_measure_smoke_runner.cpp
)

target_link_libraries(scan_tracking_weld_measure_smoke
    PRIVATE
        Qt5::Core
        scan_tracking_weld_measure
)

set_target_properties(scan_tracking_weld_measure_smoke PROPERTIES
    AUTOMOC ON
    FOLDER "tests"
)

scan_tracking_deploy_weld_measure_runtime(scan_tracking_weld_measure_smoke)
