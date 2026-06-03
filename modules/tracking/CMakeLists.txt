add_library(scan_tracking_tracking STATIC
    src/tracking_service.cpp
    src/lb_pose_check.cpp
)

target_include_directories(scan_tracking_tracking
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(scan_tracking_tracking PUBLIC cxx_std_17)

target_link_libraries(scan_tracking_tracking
    PUBLIC
        Qt5::Core
        scan_tracking_common
        scan_tracking_mech_eye
    PRIVATE
        LbPoseDetection::lb_pose_detection
)

if(SCAN_TRACKING_ENABLE_BEVEL_MEASUREMENT)
    target_link_libraries(scan_tracking_tracking
        PRIVATE
            scan_tracking_bevel_adapter
    )

    add_executable(scan_tracking_tracking_smoke_test
        tests/tracking_service_smoke_test.cpp
    )

    target_link_libraries(scan_tracking_tracking_smoke_test
        PRIVATE
            Qt5::Core
            scan_tracking_tracking
    )

    set_target_properties(scan_tracking_tracking_smoke_test PROPERTIES
        FOLDER "tests"
    )

    scan_tracking_deploy_bevel_runtime(scan_tracking_tracking_smoke_test)
    scan_tracking_deploy_pcl_runtime(scan_tracking_tracking_smoke_test)
    scan_tracking_deploy_opencv_runtime(scan_tracking_tracking_smoke_test)
endif()

set_target_properties(scan_tracking_tracking PROPERTIES
    FOLDER "modules"
)
