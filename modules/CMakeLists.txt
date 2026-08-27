add_subdirectory(mech_eye)
add_subdirectory(orbbec_gemini)
add_subdirectory(livox_mid360)
add_subdirectory(tfmini_plus)
add_subdirectory(vision)
if(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION)
    add_subdirectory(tracking)
endif()
add_subdirectory(weld_measure)
add_subdirectory(thickness_measure)
# path4 已切 ThicknessMeasureV2；SDK 必须已打包。
set(_tmv2_sdk "${CMAKE_SOURCE_DIR}/third_party/thickness_measure_v2")
if(NOT EXISTS "${_tmv2_sdk}/include/thickness_measure_v2_c_api.h"
   OR NOT EXISTS "${_tmv2_sdk}/lib/Release/ThicknessMeasureV2.lib"
   OR NOT EXISTS "${_tmv2_sdk}/bin/Release/ThicknessMeasureV2.dll")
    message(FATAL_ERROR
        "ThicknessMeasureV2 SDK missing under third_party/thickness_measure_v2.\n"
        "Build ThicknessMeasureV2Dll and run scripts/package_sdk_to_ipc.ps1.")
endif()
add_subdirectory(thickness_measure_v2)
add_subdirectory(inner_surface_measure)
add_subdirectory(undercut_length_measure)
add_subdirectory(length_volume_measure)
add_subdirectory(container_total_length_measure)
set(_collision_sdk "${CMAKE_SOURCE_DIR}/third_party/collision_detection")
if(NOT EXISTS "${_collision_sdk}/include/collision_detection_c_api.h"
   OR NOT EXISTS "${_collision_sdk}/lib/Release/CollisionDetection.lib"
   OR NOT EXISTS "${_collision_sdk}/bin/Release/CollisionDetection.dll")
    message(FATAL_ERROR
        "CollisionDetection SDK missing under third_party/collision_detection.\n"
        "Build CollisionDetectionDll and run scripts/package_sdk_to_ipc.ps1.")
endif()
add_subdirectory(collision_monitor)
add_subdirectory(modbus)
add_subdirectory(flow_control)
add_subdirectory(hmi_server)
