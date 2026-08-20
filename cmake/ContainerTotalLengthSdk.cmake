include_guard(GLOBAL)

set(
    SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/container_total_length"
    CACHE PATH
    "Path to the ContainerTotalLength SDK directory (headers/lib/bin)"
)

set(_SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_PCL_DLLS
    pcl_common.dll
    pcl_filters.dll
    pcl_kdtree.dll
    pcl_search.dll
    pcl_features.dll
    pcl_registration.dll
    pcl_sample_consensus.dll
    pcl_octree.dll
)

function(scan_tracking_require_container_total_length_sdk)
    if(TARGET ContainerTotalLengthSdk::Api)
        return()
    endif()

    set(_sdk_dir "${SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR}")
    set(_include_dir "${_sdk_dir}/include")
    set(_release_lib "${_sdk_dir}/lib/Release/ContainerTotalLength.lib")
    set(_debug_lib "${_sdk_dir}/lib/Debug/ContainerTotalLength.lib")
    set(_release_dll "${_sdk_dir}/bin/Release/ContainerTotalLength.dll")
    set(_debug_dll "${_sdk_dir}/bin/Debug/ContainerTotalLength.dll")
    set(_header "${_include_dir}/ContainerTotalLengthApi.h")

    foreach(_required_path IN ITEMS
        "${_header}"
        "${_release_lib}"
        "${_release_dll}"
    )
        if(NOT EXISTS "${_required_path}")
            message(FATAL_ERROR
                "ContainerTotalLength SDK file not found: ${_required_path}\n"
                "Build ContainerTotalLengthDll and run scripts/package_sdk_to_ipc.ps1 in the algorithm repo.")
        endif()
    endforeach()

    if(NOT EXISTS "${_debug_lib}")
        set(_debug_lib "${_release_lib}")
    endif()
    if(NOT EXISTS "${_debug_dll}")
        set(_debug_dll "${_release_dll}")
    endif()

    add_library(ContainerTotalLengthSdk::Api SHARED IMPORTED GLOBAL)
    set_target_properties(ContainerTotalLengthSdk::Api PROPERTIES
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

    set_property(GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR "${_sdk_dir}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_DLL_RELEASE "${_release_dll}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_DLL_DEBUG "${_debug_dll}")
endfunction()

function(scan_tracking_deploy_container_total_length_runtime target_name)
    scan_tracking_require_container_total_length_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR)
    get_property(_dll_release GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_DLL_RELEASE)
    get_property(_dll_debug GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_DLL_DEBUG)
    set(_bin_release "${_sdk_dir}/bin/Release")
    set(_bin_debug "${_sdk_dir}/bin/Debug")
    if(NOT EXISTS "${_bin_debug}/ContainerTotalLength.dll")
        set(_bin_debug "${_bin_release}")
    endif()

    set(_config_dir "${CMAKE_SOURCE_DIR}/config/container_total_length")
    set(_config_ini "${_config_dir}/config.ini")
    if(NOT EXISTS "${_config_ini}")
        set(_config_ini "${_sdk_dir}/config.ini")
    endif()
    set(_config_data_dir "${_config_dir}/Data")

    set(_copy_cmds
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_dll_debug},${_dll_release}>"
            "$<TARGET_FILE_DIR:${target_name}>"
    )

    foreach(_pcl_dll IN LISTS _SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_PCL_DLLS)
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<IF:$<CONFIG:Debug>,${_bin_debug}/${_pcl_dll},${_bin_release}/${_pcl_dll}>"
                "$<TARGET_FILE_DIR:${target_name}>"
        )
    endforeach()

    if(EXISTS "${_config_ini}")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/container_total_length"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_ini}"
                "$<TARGET_FILE_DIR:${target_name}>/config/container_total_length/config.ini"
        )
    endif()

    if(EXISTS "${_config_data_dir}/sample_cylinder.pcd")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/container_total_length/Data"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_data_dir}/sample_cylinder.pcd"
                "$<TARGET_FILE_DIR:${target_name}>/config/container_total_length/Data/sample_cylinder.pcd"
        )
    endif()

    if(EXISTS "${_config_data_dir}/Template_Path3_Arm_All_Samp.pcd")
        list(APPEND _copy_cmds
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${target_name}>/config/container_total_length/Data"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_config_data_dir}/Template_Path3_Arm_All_Samp.pcd"
                "$<TARGET_FILE_DIR:${target_name}>/config/container_total_length/Data/Template_Path3_Arm_All_Samp.pcd"
        )
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        ${_copy_cmds}
        COMMENT "Deploying ContainerTotalLength SDK runtime"
    )

    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY
            VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk_dir}/bin/Release;${_sdk_dir}/bin/Debug;%PATH%"
        )
    endif()
endfunction()
