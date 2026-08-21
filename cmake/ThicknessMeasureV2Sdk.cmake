include_guard(GLOBAL)

set(
    SCAN_TRACKING_THICKNESS_MEASURE_V2_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/thickness_measure_v2"
    CACHE PATH
    "Path to the ThicknessMeasureV2 SDK directory (headers/lib/bin)"
)

set(_SCAN_TRACKING_THICKNESS_V2_PCL_DLLS
    pcl_common.dll
    pcl_filters.dll
    pcl_kdtree.dll
    pcl_search.dll
    pcl_features.dll
    pcl_registration.dll
    pcl_sample_consensus.dll
    pcl_octree.dll
)

function(scan_tracking_require_thickness_measure_v2_sdk)
    if(TARGET ThicknessMeasureV2Sdk::Api)
        return()
    endif()

    set(_sdk_dir "${SCAN_TRACKING_THICKNESS_MEASURE_V2_SDK_DIR}")
    set(_include_dir "${_sdk_dir}/include")
    set(_release_lib "${_sdk_dir}/lib/Release/ThicknessMeasureV2.lib")
    set(_debug_lib "${_sdk_dir}/lib/Debug/ThicknessMeasureV2.lib")
    set(_release_dll "${_sdk_dir}/bin/Release/ThicknessMeasureV2.dll")
    set(_debug_dll "${_sdk_dir}/bin/Debug/ThicknessMeasureV2.dll")
    set(_header "${_include_dir}/thickness_measure_v2_c_api.h")

    foreach(_required_path IN ITEMS
        "${_header}"
        "${_release_lib}"
        "${_release_dll}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR
                "ThicknessMeasureV2 SDK file not found: ${_required_path}\n"
                "Build ThicknessMeasureV2Dll and run scripts/package_sdk_to_ipc.ps1 in "
                "第二工位测量源码/path4/厚度测量-VS2022-V2.0/厚度测量-VS2022")
        endif()
    endforeach()

    if(NOT EXISTS "${_debug_lib}")
        set(_debug_lib "${_release_lib}")
    endif()
    if(NOT EXISTS "${_debug_dll}")
        set(_debug_dll "${_release_dll}")
    endif()

    add_library(ThicknessMeasureV2Sdk::Api SHARED IMPORTED GLOBAL)
    set_target_properties(ThicknessMeasureV2Sdk::Api PROPERTIES
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

    set_property(GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_V2_SDK_DIR "${_sdk_dir}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_V2_DLL_RELEASE "${_release_dll}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_V2_DLL_DEBUG "${_debug_dll}")
endfunction()

function(scan_tracking_deploy_thickness_measure_v2_runtime target_name)
    scan_tracking_require_thickness_measure_v2_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_V2_SDK_DIR)
    get_property(_dll_release GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_V2_DLL_RELEASE)
    get_property(_dll_debug GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_V2_DLL_DEBUG)
    set(_config_ini "${_sdk_dir}/thickness_measurement.ini")
    set(_bin_release "${_sdk_dir}/bin/Release")
    set(_bin_debug "${_sdk_dir}/bin/Debug")
    if(NOT EXISTS "${_bin_debug}/ThicknessMeasureV2.dll")
        set(_bin_debug "${_bin_release}")
    endif()
    set(_model "${_sdk_dir}/models/pointnet_weld_seam_V7.3_good.onnx")
    set(_outer_tpl "${_sdk_dir}/Data/0_template_outer_sample.pcd")
    set(_inner_scan "${_sdk_dir}/Data/0_scan_inner_sample.pcd")
    set(_outer_scan "${_sdk_dir}/Data/0_scan_outer_sample.pcd")

    set(_copy_cmds
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_dll_debug},${_dll_release}>"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_bin_debug}/onnxruntime.dll,${_bin_release}/onnxruntime.dll>"
            "$<TARGET_FILE_DIR:${target_name}>"
    )

    foreach(_pcl_dll IN LISTS _SCAN_TRACKING_THICKNESS_V2_PCL_DLLS)
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<IF:$<CONFIG:Debug>,${_bin_debug}/${_pcl_dll},${_bin_release}/${_pcl_dll}>"
                "$<TARGET_FILE_DIR:${target_name}>"
        )
    endforeach()

    if(EXISTS "${_config_ini}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure_v2"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_ini}"
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure_v2/thickness_measurement.ini"
        )
    endif()
    if(EXISTS "${_model}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/models/thickness_measure_v2"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_model}"
                "$<TARGET_FILE_DIR:${target_name}>/models/thickness_measure_v2/pointnet_weld_seam_V7.3_good.onnx"
        )
    endif()
    if(EXISTS "${_outer_tpl}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure_v2/Data"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_outer_tpl}"
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure_v2/Data/0_template_outer_sample.pcd"
        )
    endif()
    if(EXISTS "${_inner_scan}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure_v2/Data"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_inner_scan}"
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure_v2/Data/0_scan_inner_sample.pcd"
        )
    endif()
    if(EXISTS "${_outer_scan}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure_v2/Data"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_outer_scan}"
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure_v2/Data/0_scan_outer_sample.pcd"
        )
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        ${_copy_cmds}
        COMMENT "Deploying ThicknessMeasureV2 SDK runtime"
    )

    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY
            VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk_dir}/bin/Release;${_sdk_dir}/bin/Debug;%PATH%"
        )
    endif()
endfunction()
