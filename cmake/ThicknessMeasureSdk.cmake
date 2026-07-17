include_guard(GLOBAL)

set(
    SCAN_TRACKING_THICKNESS_MEASURE_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/thickness_measure"
    CACHE PATH
    "Path to the ThicknessMeasure SDK directory (headers/lib/bin)"
)

set(_SCAN_TRACKING_THICKNESS_PCL_DLLS
    pcl_common.dll
    pcl_filters.dll
    pcl_kdtree.dll
    pcl_search.dll
    pcl_registration.dll
    pcl_sample_consensus.dll
    pcl_octree.dll
)

function(scan_tracking_require_thickness_measure_sdk)
    if(TARGET ThicknessMeasureSdk::Api)
        return()
    endif()

    set(_sdk_dir "${SCAN_TRACKING_THICKNESS_MEASURE_SDK_DIR}")
    set(_include_dir "${_sdk_dir}/include")
    set(_release_lib "${_sdk_dir}/lib/Release/ThicknessMeasure.lib")
    set(_debug_lib "${_sdk_dir}/lib/Debug/ThicknessMeasure.lib")
    set(_release_dll "${_sdk_dir}/bin/Release/ThicknessMeasure.dll")
    set(_debug_dll "${_sdk_dir}/bin/Debug/ThicknessMeasure.dll")
    set(_header "${_include_dir}/thickness_measure_c_api.h")

    foreach(_required_path IN ITEMS
        "${_header}"
        "${_release_lib}"
        "${_release_dll}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR
                "ThicknessMeasure SDK file not found: ${_required_path}\n"
                "Build ThicknessMeasureDll and run scripts/package_sdk_to_ipc.ps1 in the algorithm repo.")
        endif()
    endforeach()

    if(NOT EXISTS "${_debug_lib}")
        set(_debug_lib "${_release_lib}")
    endif()
    if(NOT EXISTS "${_debug_dll}")
        set(_debug_dll "${_release_dll}")
    endif()

    add_library(ThicknessMeasureSdk::Api SHARED IMPORTED GLOBAL)
    set_target_properties(ThicknessMeasureSdk::Api PROPERTIES
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

    set_property(GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_SDK_DIR "${_sdk_dir}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_DLL_RELEASE "${_release_dll}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_DLL_DEBUG "${_debug_dll}")
endfunction()

function(scan_tracking_deploy_thickness_measure_runtime target_name)
    scan_tracking_require_thickness_measure_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_SDK_DIR)
    get_property(_dll_release GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_DLL_RELEASE)
    get_property(_dll_debug GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_DLL_DEBUG)
    set(_config_json "${_sdk_dir}/thickness_config.json")
    set(_bin_release "${_sdk_dir}/bin/Release")
    set(_bin_debug "${_sdk_dir}/bin/Debug")
    if(NOT EXISTS "${_bin_debug}/ThicknessMeasure.dll")
        set(_bin_debug "${_bin_release}")
    endif()

    set(_copy_cmds
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_dll_debug},${_dll_release}>"
            "$<TARGET_FILE_DIR:${target_name}>"
    )

    foreach(_pcl_dll IN LISTS _SCAN_TRACKING_THICKNESS_PCL_DLLS)
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<IF:$<CONFIG:Debug>,${_bin_debug}/${_pcl_dll},${_bin_release}/${_pcl_dll}>"
                "$<TARGET_FILE_DIR:${target_name}>"
        )
    endforeach()

    if(EXISTS "${_config_json}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_json}"
                "$<TARGET_FILE_DIR:${target_name}>/config/thickness_measure/thickness_config.json"
        )
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        ${_copy_cmds}
        COMMENT "Deploying ThicknessMeasure SDK runtime"
    )

    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY
            VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk_dir}/bin/Release;${_sdk_dir}/bin/Debug;%PATH%"
        )
    endif()
endfunction()
