scan_tracking_require_thickness_measure_v3_sdk()

# Host façade: no link to ThicknessMeasureV3 (process isolation).
add_library(scan_tracking_thickness_measure_v2 STATIC
    src/thickness_measure_v2_service.cpp
    include/scan_tracking/thickness_measure_v2/thickness_measure_v2_types.h
    include/scan_tracking/thickness_measure_v2/thickness_measure_v2_service.h
    include/scan_tracking/thickness_measure_v2/thickness_measure_v3_worker_protocol.h
)

set_target_properties(scan_tracking_thickness_measure_v2 PROPERTIES
    FOLDER "modules"
)

target_include_directories(scan_tracking_thickness_measure_v2
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_thickness_measure_v2
    PUBLIC
        Qt5::Core
)

target_compile_features(scan_tracking_thickness_measure_v2 PUBLIC cxx_std_17)

# Isolated worker: loads original ThicknessMeasureV3.dll.
add_executable(thickness_measure_v3_worker
    src/thickness_measure_v3_worker_main.cpp
)

set_target_properties(thickness_measure_v3_worker PROPERTIES
    OUTPUT_NAME "thickness-measure-v3-worker"
    FOLDER "modules"
)

target_link_libraries(thickness_measure_v3_worker
    PRIVATE
        ThicknessMeasureV3Sdk::Api
)

target_compile_features(thickness_measure_v3_worker PRIVATE cxx_std_17)

scan_tracking_deploy_thickness_measure_v3_runtime(thickness_measure_v3_worker)

function(scan_tracking_deploy_thickness_measure_v3_worker target)
    if(NOT TARGET thickness_measure_v3_worker)
        message(FATAL_ERROR "thickness_measure_v3_worker target missing")
    endif()
    add_dependencies(${target} thickness_measure_v3_worker)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:thickness_measure_v3_worker>"
            "$<TARGET_FILE_DIR:${target}>"
        COMMENT "Deploying thickness-measure-v3-worker beside ${target}"
    )
    scan_tracking_deploy_thickness_measure_v3_runtime(${target})
endfunction()

add_executable(scan_tracking_thickness_measure_v2_smoke
    tests/thickness_measure_v2_smoke_runner.cpp
)

target_link_libraries(scan_tracking_thickness_measure_v2_smoke
    PRIVATE
        Qt5::Core
        scan_tracking_thickness_measure_v2
)

set_target_properties(scan_tracking_thickness_measure_v2_smoke PROPERTIES
    AUTOMOC ON
    FOLDER "tests"
)

scan_tracking_deploy_thickness_measure_v3_worker(scan_tracking_thickness_measure_v2_smoke)
