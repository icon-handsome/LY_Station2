scan_tracking_require_collision_detection_sdk()

add_library(scan_tracking_collision_monitor STATIC
    src/collision_monitor_service.cpp
    src/collision_monitor_pipeline.cpp
    include/scan_tracking/collision_monitor/collision_monitor_types.h
    include/scan_tracking/collision_monitor/collision_monitor_service.h
    include/scan_tracking/collision_monitor/collision_monitor_pipeline.h
)

set_target_properties(scan_tracking_collision_monitor PROPERTIES
    FOLDER "modules"
    AUTOMOC ON
)

target_include_directories(scan_tracking_collision_monitor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_collision_monitor
    PUBLIC
        Qt5::Core
    PRIVATE
        CollisionDetectionSdk::Api
)

target_compile_features(scan_tracking_collision_monitor PUBLIC cxx_std_17)

add_executable(scan_tracking_collision_monitor_smoke
    tests/collision_monitor_smoke_runner.cpp
)

target_link_libraries(scan_tracking_collision_monitor_smoke
    PRIVATE
        Qt5::Core
        scan_tracking_collision_monitor
)

set_target_properties(scan_tracking_collision_monitor_smoke PROPERTIES
    AUTOMOC ON
    FOLDER "tests"
)

scan_tracking_deploy_collision_detection_runtime(scan_tracking_collision_monitor_smoke)
