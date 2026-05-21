cmake_minimum_required(VERSION 3.21)

project(LbnPoseDetection LANGUAGES CXX)

# Reuse the validated OpenCV bundle shipped with third_party/LB.
set(LBN_OPENCV_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../LB/opencv-3.4.3-vc14_vc15/opencv/build"
    CACHE PATH
    "Path to the bundled OpenCV build used by LBN"
)

find_package(OpenCV REQUIRED
    PATHS "${LBN_OPENCV_ROOT}/x64/vc15/lib"
    NO_DEFAULT_PATH
)

add_library(lbn_pose STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/FastGeoHash.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/lbn_pose_core.cpp"
)

add_library(LbnPose::lbn_pose ALIAS lbn_pose)

set(LBN_DATA_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/template-3D-ALL-Shift-Cut-Cut.txt"
)

target_sources(lbn_pose
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/FastGeoHash.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/lbn_pose_core.h"
        ${LBN_DATA_FILES}
)

set_source_files_properties(${LBN_DATA_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "third_party/LBN" FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/FastGeoHash.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FastGeoHash.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/lbn_pose_core.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/lbn_pose_core.h"
    ${LBN_DATA_FILES}
)

target_include_directories(lbn_pose
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${LBN_OPENCV_ROOT}/include"
)

target_link_libraries(lbn_pose
    PUBLIC
        ${OpenCV_LIBS}
)

target_compile_features(lbn_pose PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(lbn_pose PRIVATE /utf-8 /bigobj)
endif()

set_target_properties(lbn_pose PROPERTIES
    FOLDER "third_party"
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
