include_guard(GLOBAL)

set(
    SCAN_TRACKING_UNDERCUT_LENGTH_MEASURE_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/undercut_length_measure"
    CACHE PATH
    "Path to the UndercutLengthMeasure SDK directory (headers/lib/bin)"
)

function(scan_tracking_require_undercut_length_measure_sdk)
    if(TARGET UndercutLengthMeasureSdk::Api)
        return()
    endif()

    set(_sdk_dir "${SCAN_TRACKING_UNDERCUT_LENGTH_MEASURE_SDK_DIR}")
    set(_include_dir "${_sdk_dir}/include")
    set(_release_lib "${_sdk_dir}/lib/Release/UndercutLengthMeasure.lib")
    set(_debug_lib "${_sdk_dir}/lib/Debug/UndercutLengthMeasure.lib")
    set(_release_dll "${_sdk_dir}/bin/Release/UndercutLengthMeasure.dll")
    set(_debug_dll "${_sdk_dir}/bin/Debug/UndercutLengthMeasure.dll")
    set(_header "${_include_dir}/undercut_length_measure_c_api.h")

    foreach(_required_path IN ITEMS
        "${_header}"
        "${_release_lib}"
        "${_release_dll}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR
                "UndercutLengthMeasure SDK file not found: ${_required_path}\n"
                "Build UndercutLengthMeasureDll and run scripts/package_sdk_to_ipc.ps1 in the algorithm repo.")
        endif()
    endforeach()

    if(NOT EXISTS "${_debug_lib}")
        set(_debug_lib "${_release_lib}")
    endif()
    if(NOT EXISTS "${_debug_dll}")
        set(_debug_dll "${_release_dll}")
    endif()

    add_library(UndercutLengthMeasureSdk::Api SHARED IMPORTED GLOBAL)
    set_target_properties(UndercutLengthMeasureSdk::Api PROPERTIES
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

    set_property(GLOBAL PROPERTY SCAN_TRACKING_UNDERCUT_LENGTH_MEASURE_SDK_DIR "${_sdk_dir}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_UNDERCUT_LENGTH_MEASURE_DLL_RELEASE "${_release_dll}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_UNDERCUT_LENGTH_MEASURE_DLL_DEBUG "${_debug_dll}")
endfunction()

function(scan_tracking_deploy_undercut_length_measure_runtime target_name)
    scan_tracking_require_undercut_length_measure_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_UNDERCUT_LENGTH_MEASURE_SDK_DIR)
    get_property(_dll_release GLOBAL PROPERTY SCAN_TRACKING_UNDERCUT_LENGTH_MEASURE_DLL_RELEASE)
    get_property(_dll_debug GLOBAL PROPERTY SCAN_TRACKING_UNDERCUT_LENGTH_MEASURE_DLL_DEBUG)
    set(_config_ini "${_sdk_dir}/undercut_length.ini")

    set(_copy_cmds
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_dll_debug},${_dll_release}>"
            "$<TARGET_FILE_DIR:${target_name}>"
    )

    if(EXISTS "${_config_ini}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/undercut_length_measure"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_ini}"
                "$<TARGET_FILE_DIR:${target_name}>/config/undercut_length_measure/undercut_length.ini"
        )
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        ${_copy_cmds}
        COMMENT "Deploying UndercutLengthMeasure SDK runtime"
    )

    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY
            VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk_dir}/bin/Release;${_sdk_dir}/bin/Debug;%PATH%"
        )
    endif()
endfunction()
