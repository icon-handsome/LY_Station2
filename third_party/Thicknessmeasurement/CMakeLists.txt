cmake_minimum_required(VERSION 3.21)

project(ThicknessMeasurement LANGUAGES CXX)

find_package(PCL 1.12 REQUIRED COMPONENTS
    common
    io
    filters
    kdtree
    search
    registration
    sample_consensus
)

find_package(Boost REQUIRED COMPONENTS system filesystem)

set(THICKNESS_SOURCES
    src/Config.cpp
    src/ThicknessMeasurement.cpp
)

set(THICKNESS_HEADERS
    src/Config.h
    src/ThicknessMeasurement.h
)

set(THICKNESS_CONFIG_FILES
    config/thickness_config.json
)

add_library(thickness_measure STATIC
    ${THICKNESS_SOURCES}
)

add_library(Thickness::Measurement ALIAS thickness_measure)

target_sources(thickness_measure
    PRIVATE
        ${THICKNESS_HEADERS}
        ${THICKNESS_CONFIG_FILES}
)

set_source_files_properties(${THICKNESS_CONFIG_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "third_party/Thicknessmeasurement" FILES
    ${THICKNESS_SOURCES}
    ${THICKNESS_HEADERS}
    ${THICKNESS_CONFIG_FILES}
)

target_include_directories(thickness_measure
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(thickness_measure
    PUBLIC
        ${PCL_LIBRARIES}
        Boost::system
        Boost::filesystem
)

target_compile_features(thickness_measure PUBLIC cxx_std_17)

target_compile_definitions(thickness_measure
    PUBLIC
        ${PCL_DEFINITIONS}
)

if(MSVC)
    target_compile_options(thickness_measure PRIVATE /utf-8 /W4)
    target_compile_definitions(thickness_measure PRIVATE _CRT_SECURE_NO_WARNINGS _SCL_SECURE_NO_WARNINGS)
else()
    target_compile_options(thickness_measure PRIVATE -Wall -Wextra -pedantic)
endif()

set_target_properties(thickness_measure PROPERTIES
    FOLDER "third_party"
)

option(SCAN_TRACKING_BUILD_THICKNESS_DEMO "Build standalone thickness measurement demo executable" OFF)

if(SCAN_TRACKING_BUILD_THICKNESS_DEMO)
    add_executable(thickness_measure_demo src/main.cpp)
    target_link_libraries(thickness_measure_demo PRIVATE thickness_measure)
    set_target_properties(thickness_measure_demo PROPERTIES FOLDER "third_party")
endif()
