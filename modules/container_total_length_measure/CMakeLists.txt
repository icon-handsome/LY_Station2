scan_tracking_require_container_total_length_sdk()

# Host façade: no link to ContainerTotalLength (process isolation).
add_library(scan_tracking_container_total_length_measure STATIC
    src/container_total_length_service.cpp
    include/scan_tracking/container_total_length_measure/container_total_length_types.h
    include/scan_tracking/container_total_length_measure/container_total_length_service.h
    include/scan_tracking/container_total_length_measure/container_total_length_worker_protocol.h
)

set_target_properties(scan_tracking_container_total_length_measure PROPERTIES
    FOLDER "modules"
)

target_include_directories(scan_tracking_container_total_length_measure
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_container_total_length_measure
    PUBLIC
        Qt5::Core
)

target_compile_features(scan_tracking_container_total_length_measure PUBLIC cxx_std_17)

# Isolated worker: loads ContainerTotalLength.dll.
add_executable(container_total_length_worker
    src/container_total_length_worker_main.cpp
)

set_target_properties(container_total_length_worker PROPERTIES
    OUTPUT_NAME "container-total-length-worker"
    FOLDER "modules"
)

target_link_libraries(container_total_length_worker
    PRIVATE
        ContainerTotalLengthSdk::Api
)

target_compile_features(container_total_length_worker PRIVATE cxx_std_17)

# Beside the worker build output (local debug / SCAN_TRACKING_CTL_WORKER override).
get_property(_ctl_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR)
get_property(_ctl_dll_release GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_DLL_RELEASE)
get_property(_ctl_dll_debug GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_DLL_DEBUG)
set(_ctl_bin_release "${_ctl_sdk_dir}/bin/Release")
set(_ctl_bin_debug "${_ctl_sdk_dir}/bin/Debug")
if(NOT EXISTS "${_ctl_bin_debug}/ContainerTotalLength.dll")
    set(_ctl_bin_debug "${_ctl_bin_release}")
endif()

set(_ctl_worker_self_cmds
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<IF:$<CONFIG:Debug>,${_ctl_dll_debug},${_ctl_dll_release}>"
        "$<TARGET_FILE_DIR:container_total_length_worker>"
)
foreach(_pcl_dll IN ITEMS
    pcl_common.dll
    pcl_filters.dll
    pcl_kdtree.dll
    pcl_search.dll
    pcl_features.dll
    pcl_registration.dll
    pcl_sample_consensus.dll
    pcl_octree.dll
)
    list(APPEND _ctl_worker_self_cmds
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_ctl_bin_debug}/${_pcl_dll},${_ctl_bin_release}/${_pcl_dll}>"
            "$<TARGET_FILE_DIR:container_total_length_worker>"
    )
endforeach()
add_custom_command(TARGET container_total_length_worker POST_BUILD
    ${_ctl_worker_self_cmds}
    COMMENT "Deploying ContainerTotalLength DLL/PCL beside worker"
)

function(scan_tracking_deploy_container_total_length_worker target)
    if(NOT TARGET container_total_length_worker)
        message(FATAL_ERROR "container_total_length_worker target missing")
    endif()
    scan_tracking_require_container_total_length_sdk()

    get_property(_sdk_dir GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_SDK_DIR)
    get_property(_dll_release GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_DLL_RELEASE)
    get_property(_dll_debug GLOBAL PROPERTY SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_DLL_DEBUG)
    set(_bin_release "${_sdk_dir}/bin/Release")
    set(_bin_debug "${_sdk_dir}/bin/Debug")
    if(NOT EXISTS "${_bin_debug}/ContainerTotalLength.dll")
        set(_bin_debug "${_bin_release}")
    endif()

    set(_worker_dir "$<TARGET_FILE_DIR:${target}>/${SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_WORKER_RELDIR}")

    set(_cmds
        COMMAND ${CMAKE_COMMAND} -E make_directory "${_worker_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:container_total_length_worker>"
            "${_worker_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<IF:$<CONFIG:Debug>,${_dll_debug},${_dll_release}>"
            "${_worker_dir}"
    )
    foreach(_pcl_dll IN ITEMS
        pcl_common.dll
        pcl_filters.dll
        pcl_kdtree.dll
        pcl_search.dll
        pcl_features.dll
        pcl_registration.dll
        pcl_sample_consensus.dll
        pcl_octree.dll
    )
        list(APPEND _cmds
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<IF:$<CONFIG:Debug>,${_bin_debug}/${_pcl_dll},${_bin_release}/${_pcl_dll}>"
                "${_worker_dir}"
        )
    endforeach()

    add_dependencies(${target} container_total_length_worker)
    add_custom_command(TARGET ${target} POST_BUILD
        ${_cmds}
        COMMENT "Deploying container-total-length-worker into ${SCAN_TRACKING_CONTAINER_TOTAL_LENGTH_WORKER_RELDIR}"
    )
endfunction()

add_executable(scan_tracking_container_total_length_smoke
    tests/container_total_length_smoke_runner.cpp
)

target_link_libraries(scan_tracking_container_total_length_smoke
    PRIVATE
        Qt5::Core
        scan_tracking_container_total_length_measure
)

set_target_properties(scan_tracking_container_total_length_smoke PROPERTIES
    AUTOMOC ON
    FOLDER "tests"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/app"
)

# Config/templates beside smoke; worker under private dir.
scan_tracking_deploy_container_total_length_runtime(scan_tracking_container_total_length_smoke)
scan_tracking_deploy_container_total_length_worker(scan_tracking_container_total_length_smoke)
