include_guard(GLOBAL)

set(SCAN_TRACKING_THICKNESS_MEASURE_V3_1_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/thickness_measure_v3_1"
    CACHE PATH "Path to the ThicknessMeasurement V3.1 SDK")
set(SCAN_TRACKING_PCL_ROOT "C:/Program Files/PCL 1.15.1" CACHE PATH
    "PCL installation used by the ThicknessMeasurement V3.1 worker")

# Private runtime dir beside the host exe. Keeps PCL 1.15.1 away from path3's PCL 1.12.
set(SCAN_TRACKING_THICKNESS_V3_1_WORKER_RELDIR "workers/thickness_v3_1")

function(scan_tracking_require_thickness_measure_v3_1_sdk)
    if(TARGET ThicknessMeasureV3_1Sdk::Api)
        return()
    endif()
    set(d "${SCAN_TRACKING_THICKNESS_MEASURE_V3_1_SDK_DIR}")
    set(inc "${d}/include")
    set(lib "${d}/lib/Release/ThicknessMeasurement.lib")
    set(dll "${d}/bin/Release/ThicknessMeasurement.dll")
    foreach(f "${lib}" "${dll}" "${d}/bin/Release/ThicknessMeasurementDemo.exe")
        if(NOT EXISTS "${f}")
            message(FATAL_ERROR "ThicknessMeasurement V3.1 SDK file not found: ${f}")
        endif()
    endforeach()
    add_library(ThicknessMeasureV3_1Sdk::Api SHARED IMPORTED GLOBAL)
    set_target_properties(ThicknessMeasureV3_1Sdk::Api PROPERTIES
        IMPORTED_IMPLIB_RELEASE "${lib}"
        IMPORTED_IMPLIB_DEBUG "${lib}"
        IMPORTED_LOCATION_RELEASE "${dll}"
        IMPORTED_LOCATION_DEBUG "${dll}"
        INTERFACE_INCLUDE_DIRECTORIES "${inc}")
    set_property(GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_V3_1_SDK_DIR "${d}")
endfunction()

function(scan_tracking_deploy_thickness_measure_v3_1_runtime target)
    scan_tracking_require_thickness_measure_v3_1_sdk()
    get_property(d GLOBAL PROPERTY SCAN_TRACKING_THICKNESS_MEASURE_V3_1_SDK_DIR)
    set(_worker_dir "$<TARGET_FILE_DIR:${target}>/${SCAN_TRACKING_THICKNESS_V3_1_WORKER_RELDIR}")

    # Config stays under the host tree (absolute path passed in request.txt).
    # DLL/runtime goes only into the private worker dir so path3 PCL 1.12 is not overwritten.
    set(_commands
        COMMAND ${CMAKE_COMMAND} -E make_directory "${_worker_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${d}/bin/Release/ThicknessMeasurement.dll" "${_worker_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${d}/bin/Release/ThicknessMeasurementDemo.exe" "${_worker_dir}"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target}>/config/thickness_measure_v3_1"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${d}/config/thickness_measurement.ini"
            "$<TARGET_FILE_DIR:${target}>/config/thickness_measure_v3_1/thickness_measurement.ini")
    file(GLOB _runtime_dlls "${d}/bin/Release/*.dll")
    foreach(runtime_dll IN LISTS _runtime_dlls)
        list(APPEND _commands COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${runtime_dll}" "${_worker_dir}")
    endforeach()
    if(EXISTS "${d}/Data")
        list(APPEND _commands COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target}>/config/thickness_measure_v3_1/Data"
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${d}/Data"
            "$<TARGET_FILE_DIR:${target}>/config/thickness_measure_v3_1/Data"
            COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target}>/config/Data"
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${d}/Data"
            "$<TARGET_FILE_DIR:${target}>/config/Data")
    endif()
    add_custom_command(TARGET ${target} POST_BUILD ${_commands}
        COMMENT "Deploying ThicknessMeasurement V3.1 runtime into ${SCAN_TRACKING_THICKNESS_V3_1_WORKER_RELDIR}")
endfunction()
