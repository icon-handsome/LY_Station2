#pragma once

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** Lightweight XYZ PCD loader (ascii/binary). Avoids pcl_io / VTK / OpenNI. */
bool LoadXyzPcd(const std::string& path,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                std::string* error);
