include_guard(GLOBAL)

set(
    SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/container_total_length"
    CACHE PATH
    "Path to the ContainerTotalLength SDK directory (headers/lib/bin)"
)

option(
    SCAN_TRACKING_USE_CONTAINER_TOTAL_LENGTH_V22
    "Use the separately packaged path3 ContainerTotalLength V2.2 SDK"
    ON
)

if(SCAN_TRACKING_USE_CONTAINER_TOTAL_LENGTH_V22)
    set(
        SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/container_total_length_v2_2"
        CACHE PATH
        "Path to the ContainerTotalLength V2.2 SDK directory (headers/lib/bin)"
        FORCE
    )
endif()

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

# Private runtime dir beside the host exe (DLL + PCL 1.12 live here only).
set(SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_WORKER_RELDIR "workers/container_total_length")

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

    set(_bin_release "${_sdk_dir}/bin/Release")
    foreach(_pcl_dll IN LISTS _SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_PCL_DLLS)
        set(_pcl_path "${_bin_release}/${_pcl_dll}")
        if(NOT EXISTS "${_pcl_path}")
            message(FATAL_ERROR
                "ContainerTotalLength SDK PCL runtime missing: ${_pcl_path}\n"
                "Copy PCL 1.12.0 Release DLLs into ${_bin_release}, or re-run the algorithm repo packaging script.")
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

# Host-side config/templates only. DLL + PCL stay in the private worker dir
# (see scan_tracking_deploy_container_total_length_worker) so the main process
# never loads ContainerTotalLength.dll / PCL 1.12.
function(scan_tracking_deploy_container_total_length_runtime target_name)
    scan_tracking_require_container_total_length_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR)

    set(_config_dir "${CMAKE_SOURCE_DIR}/config/container_total_length")
    if(SCAN_TRACKING_USE_CONTAINER_TOTAL_LENGTH_V22)
        set(_config_ini "${_sdk_dir}/config.ini")
        set(_config_data_dir "${_sdk_dir}/Data")
    else()
        set(_config_ini "${_config_dir}/config.ini")
        if(NOT EXISTS "${_config_ini}")
            set(_config_ini "${_sdk_dir}/config.ini")
        endif()
        set(_config_data_dir "${_config_dir}/Data")
    endif()

    set(_copy_cmds
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target_name}>/config/container_total_length"
    )

    if(EXISTS "${_config_ini}")
        list(APPEND _copy_cmds
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
        COMMENT "Deploying ContainerTotalLength config/templates (DLL lives with worker)"
    )
endfunction()
