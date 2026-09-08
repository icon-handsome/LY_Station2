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
    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_sdk}/bin/Release/InnerSurfaceAndVolume.dll"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_sdk}/bin/Release"
            "$<TARGET_FILE_DIR:${target_name}>/inner_surface_measure_v2_runtime"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:${target_name}>/config/inner_surface_measure"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/third_party/inner_surface_measure/config.ini"
            "$<TARGET_FILE_DIR:${target_name}>/config/inner_surface_measure/config.ini"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/third_party/inner_surface_measure/Data"
            "$<TARGET_FILE_DIR:${target_name}>/config/inner_surface_measure/Data"
        COMMENT "Deploying path4 InnerSurfaceAndVolume V2 runtime")
    if(MSVC)
        set_property(TARGET ${target_name} APPEND PROPERTY VS_DEBUGGER_ENVIRONMENT
            "PATH=${_sdk}/bin/Release;%PATH%")
    endif()
endfunction()
