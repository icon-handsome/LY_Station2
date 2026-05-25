set(SCAN_TRACKING_VISION_SOURCES
    src/hik_mono_io.cpp
    src/hik_camera_service.cpp
    src/hik_camera_c_controller.cpp
    src/hik_smart_camera_tcp_server.cpp
    src/hik_smart_camera_ftp_monitor.cpp
    src/vision_pipeline_service.cpp
    include/scan_tracking/vision/vision_types.h
    include/scan_tracking/vision/hik_mono_io.h
    include/scan_tracking/vision/hik_camera_service.h
    include/scan_tracking/vision/hik_camera_c_controller.h
    include/scan_tracking/vision/hik_smart_camera_tcp_server.h
    include/scan_tracking/vision/hik_smart_camera_ftp_monitor.h
    include/scan_tracking/vision/vision_pipeline_service.h
)

set(SCAN_TRACKING_VISION_PRIVATE_LIBS
    HikMvsSdk::MvCameraControl
)

if(SCAN_TRACKING_ENABLE_LBN_POSE_DETECTION)
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lbn_pose_detection_adapter.cpp
        include/scan_tracking/vision/lbn_pose_detection_adapter.h
    )
    list(APPEND SCAN_TRACKING_VISION_PRIVATE_LIBS
        LbnPose::lbn_pose
    )
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lb_pose_detection_adapter_stub.cpp
        include/scan_tracking/vision/lb_pose_detection_adapter.h
    )
elseif(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION)
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lb_pose_detection_adapter.cpp
        include/scan_tracking/vision/lb_pose_detection_adapter.h
    )
    list(APPEND SCAN_TRACKING_VISION_PRIVATE_LIBS
        LbPoseDetection::lb_pose_detection
    )
else()
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lb_pose_detection_adapter_stub.cpp
        include/scan_tracking/vision/lb_pose_detection_adapter.h
    )
endif()

add_library(scan_tracking_vision STATIC ${SCAN_TRACKING_VISION_SOURCES})

set_target_properties(scan_tracking_vision PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(SCAN_TRACKING_ENABLE_LBN_POSE_DETECTION)
    target_include_directories(scan_tracking_vision
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/LBN
    )
endif()

target_link_libraries(scan_tracking_vision
    PUBLIC
        Qt5::Core
        Qt5::Network
        scan_tracking_common
        scan_tracking_mech_eye
    PRIVATE
        ${SCAN_TRACKING_VISION_PRIVATE_LIBS}
)

target_compile_features(scan_tracking_vision PUBLIC cxx_std_17)

if(BUILD_TESTING)
    find_package(Qt5 REQUIRED COMPONENTS Test)

    add_executable(scan_tracking_hik_mono_io_test
        tests/hik_mono_io_test.cpp
    )

    target_link_libraries(scan_tracking_hik_mono_io_test
        PRIVATE
            Qt5::Core
            Qt5::Test
            scan_tracking_vision
            scan_tracking_common
    )

    add_test(NAME scan_tracking_hik_mono_io_test
        COMMAND scan_tracking_hik_mono_io_test
    )

    set_target_properties(scan_tracking_hik_mono_io_test PROPERTIES
        FOLDER "tests"
    )
endif()

if(SCAN_TRACKING_ENABLE_LANYOU_DETECTION)
    add_library(scan_tracking_lanyou_test_adapter STATIC
        src/lanyou_detection_adapter.cpp
        src/lanyou_first_station_adapter.cpp
        include/scan_tracking/vision/lanyou_detection_adapter.h
        include/scan_tracking/vision/lanyou_issue_codes.h
        include/scan_tracking/vision/lanyou_first_station_adapter.h
    )

    target_include_directories(scan_tracking_lanyou_test_adapter
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(scan_tracking_lanyou_test_adapter
        PUBLIC
            Qt5::Core
            scan_tracking_mech_eye
            Lanyou::FirstDetection
    )

    target_compile_features(scan_tracking_lanyou_test_adapter PUBLIC cxx_std_17)

    set_target_properties(scan_tracking_lanyou_test_adapter PROPERTIES
        FOLDER "tests"
    )

    add_executable(scan_tracking_lanyou_smoke_test
        tests/lanyou_detection_adapter_smoke_test.cpp
    )

    target_link_libraries(scan_tracking_lanyou_smoke_test
        PRIVATE
            Qt5::Core
            scan_tracking_lanyou_test_adapter
    )

    set_target_properties(scan_tracking_lanyou_smoke_test PROPERTIES
        FOLDER "tests"
    )

    add_executable(scan_tracking_lanyou_first_station_adapter_smoke_test
        tests/lanyou_first_station_adapter_smoke_test.cpp
    )

    target_link_libraries(scan_tracking_lanyou_first_station_adapter_smoke_test
        PRIVATE
            Qt5::Core
            scan_tracking_lanyou_test_adapter
    )

    set_target_properties(scan_tracking_lanyou_first_station_adapter_smoke_test PROPERTIES
        FOLDER "tests"
    )

    add_executable(scan_tracking_lanyou_first_station_ply_runner
        tests/lanyou_first_station_ply_runner.cpp
    )

    target_link_libraries(scan_tracking_lanyou_first_station_ply_runner
        PRIVATE
            Lanyou::FirstDetection
    )

    if(MSVC)
        target_link_options(scan_tracking_lanyou_first_station_ply_runner PRIVATE /STACK:268435456)
    endif()

    set_target_properties(scan_tracking_lanyou_first_station_ply_runner PROPERTIES
        FOLDER "tests"
    )
endif()

if(SCAN_TRACKING_ENABLE_LBN_POSE_DETECTION)
    add_executable(scan_tracking_lbn_offline_runner
        tests/lbn_offline_runner.cpp
    )

    target_include_directories(scan_tracking_lbn_offline_runner
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/LBN
            ${CMAKE_SOURCE_DIR}/third_party/LB/opencv-3.4.3-vc14_vc15/opencv/build/include
    )

    target_link_libraries(scan_tracking_lbn_offline_runner
        PRIVATE
            Qt5::Core
            scan_tracking_vision
            LbnPose::lbn_pose
    )

    if(MSVC)
        target_link_options(scan_tracking_lbn_offline_runner PRIVATE /STACK:268435456)
    endif()

    set_target_properties(scan_tracking_lbn_offline_runner PROPERTIES
        FOLDER "tests"
    )
endif()
