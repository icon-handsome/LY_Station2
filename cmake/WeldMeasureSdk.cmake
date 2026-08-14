include_guard(GLOBAL)

set(
    SCAN_TRACKING_WELD_MEASURE_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/weld_measure"
    CACHE PATH
    "Path to the WeldMeasure SDK directory (headers/lib/bin/models)"
)

function(scan_tracking_require_weld_measure_sdk)
    if(TARGET WeldMeasureSdk::Api)
        return()
    endif()

    set(_sdk_dir "${SCAN_TRACKING_WELD_MEASURE_SDK_DIR}")
    set(_include_dir "${_sdk_dir}/include")
    set(_release_lib "${_sdk_dir}/lib/Release/WeldMeasure.lib")
    set(_debug_lib "${_sdk_dir}/lib/Debug/WeldMeasure.lib")
    set(_release_dll "${_sdk_dir}/bin/Release/WeldMeasure.dll")
    set(_debug_dll "${_sdk_dir}/bin/Debug/WeldMeasure.dll")
    set(_header "${_include_dir}/weld_measure_c_api.h")

    foreach(_required_path IN ITEMS
        "${_header}"
        "${_release_lib}"
        "${_release_dll}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR
                "WeldMeasure SDK file not found: ${_required_path}\n"
                "Build WeldMeasureDll and run scripts/package_sdk_to_ipc.ps1 in the algorithm repo.")
        endif()
    endforeach()

    set(_release_ort "${_sdk_dir}/bin/Release/onnxruntime.dll")
    set(_debug_ort "${_sdk_dir}/bin/Debug/onnxruntime.dll")
    if(NOT EXISTS "${_debug_lib}")
        set(_debug_lib "${_release_lib}")
    endif()
    if(NOT EXISTS "${_debug_dll}")
        set(_debug_dll "${_release_dll}")
    endif()
    if(NOT EXISTS "${_debug_ort}")
        set(_debug_ort "${_release_ort}")
    endif()
    if(NOT EXISTS "${_release_ort}")
        message(FATAL_ERROR "WeldMeasure SDK onnxruntime.dll not found under ${_sdk_dir}/bin")
    endif()

    add_library(WeldMeasureSdk::Api SHARED IMPORTED GLOBAL)
    set_target_properties(WeldMeasureSdk::Api PROPERTIES
        IMPORTED_IMPLIB_RELEASE "${_release_lib}"
        IMPORTED_IMPLIB_RELWITHDEBINFO "${_release_lib}"
        IMPORTED_IMPLIB_MINSIZEREL "${_release_lib}"
        IMPORTED_IMPLIB_DEBUG "${_debug_lib}"
        IMPORTED_LOCATION_RELEASE "${_release_dll}"
        IMPORTED_LOCATION_RELWITHDEBINFO "${_release_dll}"
        IMPORTED_LOCATION_MINSIZEREL "${_release_dll}"
        IMPORTED_LOCATION_DEBUG "${_debug_dll}"
        INTERFACE_INCLUDE_DIRECTORIES "${_include_dir}"
    )

    set_property(GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_SDK_DIR "${_sdk_dir}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_DLL_RELEASE "${_release_dll}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_DLL_DEBUG "${_debug_dll}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_ORT_RELEASE "${_release_ort}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_ORT_DEBUG "${_debug_ort}")
endfunction()

function(scan_tracking_deploy_weld_measure_runtime target_name)
    scan_tracking_require_weld_measure_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_SDK_DIR)
    get_property(_dll_release GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_DLL_RELEASE)
    get_property(_dll_debug GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_DLL_DEBUG)
    get_property(_ort_release GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_ORT_RELEASE)
    get_property(_ort_debug GLOBAL PROPERTY SCAN_TRACKING_WELD_MEASURE_ORT_DEBUG)
    set(_models_dir "${_sdk_dir}/models")
    set(_config_dir "${CMAKE_SOURCE_DIR}/config/weld_measure")
    set(_config_ini "${_config_dir}/weld_measurement.ini")

    set(_copy_cmds
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_dll_debug},${_dll_release}>"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_ort_debug},${_ort_release}>"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target_name}>/models/weld_measure"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_models_dir}/pointnet_weld_seam_V7.3_good.onnx"
            "$<TARGET_FILE_DIR:${target_name}>/models/weld_measure/pointnet_weld_seam_V7.3_good.onnx"
    )

    if(EXISTS "${_config_ini}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_ini}"
                "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/weld_measurement.ini"
        )
        # Prefer ASCII-safe aliases; fall back to Chinese names if present.
        set(_arm_ini "${_config_dir}/weld_measurement-arm.ini")
        set(_tele_ini "${_config_dir}/weld_measurement-telescopic.ini")
        if(NOT EXISTS "${_arm_ini}")
            set(_arm_ini "${_config_dir}/weld_measurement-机械臂直焊缝.ini")
        endif()
        if(NOT EXISTS "${_tele_ini}")
            set(_tele_ini "${_config_dir}/weld_measurement-伸缩杆直焊缝.ini")
        endif()
        if(EXISTS "${_arm_ini}")
            list(APPEND _copy_cmds
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_arm_ini}"
                    "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/weld_measurement-arm.ini"
            )
        endif()
        if(EXISTS "${_tele_ini}")
            list(APPEND _copy_cmds
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_tele_ini}"
                    "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/weld_measurement-telescopic.ini"
            )
        endif()

        set(_ring_arm_ini "${_config_dir}/weld_measurement-ring-arm.ini")
        set(_ring_tele_ini "${_config_dir}/weld_measurement-ring-telescopic.ini")
        if(EXISTS "${_ring_arm_ini}")
            list(APPEND _copy_cmds
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_ring_arm_ini}"
                    "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/weld_measurement-ring-arm.ini"
            )
        endif()
        if(EXISTS "${_ring_tele_ini}")
            list(APPEND _copy_cmds
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_ring_tele_ini}"
                    "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/weld_measurement-ring-telescopic.ini"
            )
        endif()

        # V2.1 shared templates (arm 1/2/9 + telescopic 1/4/7), relative to config/weld_measure/
        set(_template_dir "${_config_dir}/Data/path1")
        if(EXISTS "${_template_dir}")
            list(APPEND _copy_cmds
                COMMAND ${CMAKE_COMMAND} -E make_directory
                    "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/Data/path1"
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${_template_dir}"
                    "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/Data/path1"
            )
        endif()
        set(_ring_template_dir "${_config_dir}/Data/path5")
        if(EXISTS "${_ring_template_dir}")
            list(APPEND _copy_cmds
                COMMAND ${CMAKE_COMMAND} -E make_directory
                    "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/Data/path5"
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${_ring_template_dir}"
                    "$<TARGET_FILE_DIR:${target_name}>/config/weld_measure/Data/path5"
            )
        endif()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        ${_copy_cmds}
        COMMENT "Deploying WeldMeasure SDK runtime"
    )

    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY
            VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk_dir}/bin/Release;${_sdk_dir}/bin/Debug;%PATH%"
        )
    endif()
endfunction()
