cmake_minimum_required(VERSION 3.21)

project(LbPoseDetection LANGUAGES CXX)

set(LB_OPENCV_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/opencv-3.4.3-vc14_vc15/opencv/build"
    CACHE PATH
    "Path to the bundled OpenCV build"
)

find_package(PCL 1.12 REQUIRED COMPONENTS
    common
    features
    filters
    io
    kdtree
    sample_consensus
    search
    segmentation
    surface
)

find_package(OpenCV REQUIRED
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/opencv-3.4.3-vc14_vc15/opencv/build/x64/vc15/lib"
    NO_DEFAULT_PATH
)

file(GLOB LB_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/TR_Mark_3D_Recon.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/TR_Mark_Track.cpp"
)

file(GLOB LB_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/TR_Mark_3D_Recon.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/TR_Mark_Track.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/Filter.h"
)

add_library(lb_pose_detection STATIC ${LB_SOURCES})
add_library(LbPoseDetection::lb_pose_detection ALIAS lb_pose_detection)

target_sources(lb_pose_detection PRIVATE ${LB_HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "third_party/LB" FILES ${LB_SOURCES} ${LB_HEADERS})

target_include_directories(lb_pose_detection
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/opencv-3.4.3-vc14_vc15/opencv/build/include"
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lb_pose_detection
    PUBLIC
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

target_compile_features(lb_pose_detection PUBLIC cxx_std_17)

target_compile_definitions(lb_pose_detection
    PUBLIC
        ${PCL_DEFINITIONS}
)

if(MSVC)
    target_compile_options(lb_pose_detection PRIVATE /utf-8 /bigobj)
endif()

set_target_properties(lb_pose_detection PROPERTIES
    FOLDER "third_party"
)
