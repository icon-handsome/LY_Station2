scan_tracking_require_container_total_length_sdk()

add_library(scan_tracking_container_total_length_measure STATIC
    src/container_total_length_service.cpp
    include/scan_tracking/container_total_length_measure/container_total_length_types.h
    include/scan_tracking/container_total_length_measure/container_total_length_service.h
)

set_target_properties(scan_tracking_container_total_length_measure PROPERTIES
    FOLDER "modules"
)

target_include_directories(scan_tracking_container_total_length_measure
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_container_total_length_measure
    PUBLIC
        Qt5::Core
    PRIVATE
        ContainerTotalLengthSdk::Api
)

target_compile_features(scan_tracking_container_total_length_measure PUBLIC cxx_std_17)
