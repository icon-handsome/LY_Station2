cmake_minimum_required(VERSION 3.21)
project(ContainerTotalLengthV22 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PCL_DIR "C:/Program Files/PCL 1.12.0/cmake" CACHE PATH "PCL cmake directory")
find_package(PCL 1.12 REQUIRED COMPONENTS common filters kdtree search features registration sample_consensus octree io)

add_library(ContainerTotalLength SHARED
    src/ContainerTotalLengthApi.cpp
    src/Measurement.cpp
    src/PointCloudIO.cpp
    src/SimplePcdIo.cpp
)
target_compile_definitions(ContainerTotalLength PRIVATE CONTAINER_TOTAL_LENGTH_EXPORTS CONTAINER_TOTAL_LENGTH_BUILD_DLL CONTAINER_TOTAL_LENGTH_NO_PCL_IO)
target_include_directories(ContainerTotalLength PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_directories(ContainerTotalLength PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(ContainerTotalLength PRIVATE ${PCL_LIBRARIES})
target_compile_options(ContainerTotalLength PRIVATE /utf-8 /EHsc)
set_target_properties(ContainerTotalLength PROPERTIES OUTPUT_NAME ContainerTotalLength)

install(TARGETS ContainerTotalLength RUNTIME DESTINATION bin/Release LIBRARY DESTINATION bin/Release ARCHIVE DESTINATION lib/Release)
install(FILES include/ContainerTotalLengthApi.h DESTINATION include)
