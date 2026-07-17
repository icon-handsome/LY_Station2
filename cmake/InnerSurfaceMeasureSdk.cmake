include_guard(GLOBAL)

set(
    SCAN_TRACKING_INNER_SURFACE_MEASURE_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/inner_surface_measure"
    CACHE PATH
    "Path to the InnerSurfaceMeasure SDK directory (headers/lib/bin)"
)

set(_SCAN_TRACKING_INNER_SURFACE_PCL_DLLS
    pcl_common.dll
    pcl_filters.dll
    pcl_kdtree.dll
    pcl_search.dll
    pcl_registration.dll
    pcl_sample_consensus.dll
    pcl_octree.dll
)

function(scan_tracking_require_inner_surface_measure_sdk)
    if(TARGET InnerSurfaceMeasureSdk::Api)
        return()
    endif()

    set(_sdk_dir "${SCAN_TRACKING_INNER_SURFACE_MEASURE_SDK_DIR}")
    set(_include_dir "${_sdk_dir}/include")
    set(_release_lib "${_sdk_dir}/lib/Release/InnerSurfaceMeasure.lib")
    set(_debug_lib "${_sdk_dir}/lib/Debug/InnerSurfaceMeasure.lib")
    set(_release_dll "${_sdk_dir}/bin/Release/InnerSurfaceMeasure.dll")
    set(_debug_dll "${_sdk_dir}/bin/Debug/InnerSurfaceMeasure.dll")
    set(_header "${_include_dir}/inner_surface_measure_c_api.h")

    foreach(_required_path IN ITEMS
        "${_header}"
        "${_release_lib}"
        "${_release_dll}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR
                "InnerSurfaceMeasure SDK file not found: ${_required_path}\n"
                "Build InnerSurfaceMeasureDll and run scripts/package_sdk_to_ipc.ps1 in the algorithm repo.")
        endif()
    endforeach()

    if(NOT EXISTS "${_debug_lib}")
        set(_debug_lib "${_release_lib}")
    endif()
    if(NOT EXISTS "${_debug_dll}")
        set(_debug_dll "${_release_dll}")
    endif()

    add_library(InnerSurfaceMeasureSdk::Api SHARED IMPORTED GLOBAL)
    set_target_properties(InnerSurfaceMeasureSdk::Api PROPERTIES
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

    set_property(GLOBAL PROPERTY SCAN_TRACKING_INNER_SURFACE_MEASURE_SDK_DIR "${_sdk_dir}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_INNER_SURFACE_MEASURE_DLL_RELEASE "${_release_dll}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_INNER_SURFACE_MEASURE_DLL_DEBUG "${_debug_dll}")
endfunction()

function(scan_tracking_deploy_inner_surface_measure_runtime target_name)
    scan_tracking_require_inner_surface_measure_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_INNER_SURFACE_MEASURE_SDK_DIR)
    get_property(_dll_release GLOBAL PROPERTY SCAN_TRACKING_INNER_SURFACE_MEASURE_DLL_RELEASE)
    get_property(_dll_debug GLOBAL PROPERTY SCAN_TRACKING_INNER_SURFACE_MEASURE_DLL_DEBUG)
    set(_config_ini "${_sdk_dir}/config.ini")
    set(_bin_release "${_sdk_dir}/bin/Release")
    set(_bin_debug "${_sdk_dir}/bin/Debug")
    if(NOT EXISTS "${_bin_debug}/InnerSurfaceMeasure.dll")
        set(_bin_debug "${_bin_release}")
    endif()

    set(_copy_cmds
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_dll_debug},${_dll_release}>"
            "$<TARGET_FILE_DIR:${target_name}>"
    )

    foreach(_pcl_dll IN LISTS _SCAN_TRACKING_INNER_SURFACE_PCL_DLLS)
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<IF:$<CONFIG:Debug>,${_bin_debug}/${_pcl_dll},${_bin_release}/${_pcl_dll}>"
                "$<TARGET_FILE_DIR:${target_name}>"
        )
    endforeach()

    if(EXISTS "${_config_ini}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/inner_surface_measure"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_ini}"
                "$<TARGET_FILE_DIR:${target_name}>/config/inner_surface_measure/config.ini"
        )
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        ${_copy_cmds}
        COMMENT "Deploying InnerSurfaceMeasure SDK runtime"
    )

    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY
            VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk_dir}/bin/Release;${_sdk_dir}/bin/Debug;%PATH%"
        )
    endif()
endfunction()
