add_library(scan_tracking_hoist_assist STATIC
    src/hoist_assist_service.cpp
    include/scan_tracking/hoist_assist/hoist_assist_types.h
    include/scan_tracking/hoist_assist/hoist_assist_service.h
)

set_target_properties(scan_tracking_hoist_assist PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_hoist_assist
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_hoist_assist
    PUBLIC
        Qt5::Core
        scan_tracking_collision_monitor
)

target_compile_features(scan_tracking_hoist_assist PUBLIC cxx_std_17)
