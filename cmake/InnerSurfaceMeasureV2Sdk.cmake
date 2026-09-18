include_guard(GLOBAL)

set(SCAN_TRACKING_INNER_SURFACE_MEASURE_V2_SDK_DIR
    "${CMAKE_SOURCE_DIR}/third_party/inner_surface_measure_v2"
    CACHE PATH "Path to the path4 InnerSurfaceAndVolume V2 SDK")

function(scan_tracking_require_inner_surface_measure_v2_sdk)
    if(TARGET InnerSurfaceMeasureV2Sdk::Api)
        return()
    endif()
    set(_sdk "${SCAN_TRACKING_INNER_SURFACE_MEASURE_V2_SDK_DIR}")
    set(_include "${_sdk}/include")
    set(_lib "${_sdk}/lib/Release/InnerSurfaceAndVolume.lib")
    set(_dll "${_sdk}/bin/Release/InnerSurfaceAndVolume.dll")
    foreach(_path IN ITEMS "${_include}/InnerSurfaceMeasure.h" "${_lib}" "${_dll}")
        if(NOT EXISTS "${_path}")
            message(FATAL_ERROR "InnerSurfaceMeasure V2 SDK file not found: ${_path}")
        endif()
    endforeach()
    add_library(InnerSurfaceMeasureV2Sdk::Api SHARED IMPORTED GLOBAL)
    set_target_properties(InnerSurfaceMeasureV2Sdk::Api PROPERTIES
        IMPORTED_IMPLIB "${_lib}"
        IMPORTED_LOCATION "${_dll}"
        INTERFACE_INCLUDE_DIRECTORIES "${_include}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_INNER_SURFACE_MEASURE_V2_SDK_DIR "${_sdk}")
endfunction()

function(scan_tracking_deploy_inner_surface_measure_v2_runtime target_name)
    scan_tracking_require_inner_surface_measure_v2_sdk()
    get_property(_sdk GLOBAL PROPERTY SCAN_TRACKING_INNER_SURFACE_MEASURE_V2_SDK_DIR)
    set(_bin_release "${_sdk}/bin/Release")
    set(_runtime_reldir "inner_surface_measure_v2_runtime")
    set(_runtime_dir "$<TARGET_FILE_DIR:${target_name}>/${_runtime_reldir}")

    # Direct imports of InnerSurfaceAndVolume.dll (All-in-one PCL naming).
    # Must be local to this function: file-level vars are not visible when
    # app/CMakeLists.txt calls us from a different directory scope.
    # Transitive VTK/OpenNI/MSVCR120 live in the private runtime dir / PATH.
    set(_pcl_dlls
        pcl_common_release.dll
        pcl_io_release.dll
        pcl_filters_release.dll
        pcl_search_release.dll
    )

    set(_copy_cmds
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_bin_release}/InnerSurfaceAndVolume.dll"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${_runtime_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_bin_release}"
            "${_runtime_dir}"
    )
    foreach(_pcl_dll IN LISTS _pcl_dlls)
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_bin_release}/${_pcl_dll}"
                "$<TARGET_FILE_DIR:${target_name}>"
        )
    endforeach()
    list(APPEND _copy_cmds
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target_name}>/config/inner_surface_measure"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/third_party/inner_surface_measure/config.ini"
            "$<TARGET_FILE_DIR:${target_name}>/config/inner_surface_measure/config.ini"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/third_party/inner_surface_measure/Data"
            "$<TARGET_FILE_DIR:${target_name}>/config/inner_surface_measure/Data"
    )

    add_custom_command(TARGET ${target_name} POST_BUILD
        ${_copy_cmds}
        COMMENT "Deploying path4 InnerSurfaceAndVolume V2 runtime")
    if(MSVC)
        # Qt Creator / Ninja ignore this; launch.json and scan_tracking_dev.cmd
        # prepend app/inner_surface_measure_v2_runtime for VTK/OpenNI.
        set_property(TARGET ${target_name} APPEND PROPERTY VS_DEBUGGER_ENVIRONMENT
            "PATH=${_bin_release};%PATH%")
    endif()
endfunction()
