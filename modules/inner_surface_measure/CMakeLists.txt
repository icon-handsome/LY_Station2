scan_tracking_require_inner_surface_measure_sdk()

add_library(scan_tracking_inner_surface_measure STATIC
    src/inner_surface_measure_service.cpp
    include/scan_tracking/inner_surface_measure/inner_surface_measure_types.h
    include/scan_tracking/inner_surface_measure/inner_surface_measure_service.h
)

set_target_properties(scan_tracking_inner_surface_measure PROPERTIES
    FOLDER "modules"
)

target_include_directories(scan_tracking_inner_surface_measure
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_inner_surface_measure
    PUBLIC
        Qt5::Core
    PRIVATE
        InnerSurfaceMeasureSdk::Api
)

target_compile_features(scan_tracking_inner_surface_measure PUBLIC cxx_std_17)
