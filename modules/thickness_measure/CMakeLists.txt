scan_tracking_require_thickness_measure_sdk()

add_library(scan_tracking_thickness_measure STATIC
    src/thickness_measure_service.cpp
    include/scan_tracking/thickness_measure/thickness_measure_types.h
    include/scan_tracking/thickness_measure/thickness_measure_service.h
)

set_target_properties(scan_tracking_thickness_measure PROPERTIES
    FOLDER "modules"
)

target_include_directories(scan_tracking_thickness_measure
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_thickness_measure
    PUBLIC
        Qt5::Core
    PRIVATE
        ThicknessMeasureSdk::Api
)

target_compile_features(scan_tracking_thickness_measure PUBLIC cxx_std_17)
