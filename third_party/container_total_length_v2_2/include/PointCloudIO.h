#ifndef CONTAINER_TOTAL_LENGTH_POINT_CLOUD_IO_H_
#define CONTAINER_TOTAL_LENGTH_POINT_CLOUD_IO_H_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CloudPtr;
typedef CloudT::ConstPtr CloudConstPtr;

struct Config {
    std::string templateCloudPath;
    std::string inputCloudPath;
    bool cropInputCloud;
    Eigen::Vector3f cropMinPoint;
    Eigen::Vector3f cropMaxPoint;
    Eigen::Vector3f templateAxisDirection;

    float voxelSize;
    int outlierK;
    double outlierStd;

    int icpMaxIterations;
    float icpMaxCorrespondenceDistance;
    double icpTransformationEpsilon;
    double icpEuclideanFitnessEpsilon;

    int cylinderFitIterations;
    float cylinderInlierBand;
    bool updateCylinderAxis;
    int normalK;
    double endNormalMinAbsDot;

    std::string endpointDetectionMethod;
    float axialBinWidth;
    float outsideScanPeakSearchWidth;
    int minPointsPerBin;
    int minConsecutiveInsideBins;
    int outsideCheckBins;
    float refineHalfWidth;
    double refineEdgePercentile;
    float endpointMaxRadius;

    Config();
};

bool LoadIniConfig(const std::string& path, Config* config, std::string* error);
CloudPtr LoadCloud(const std::string& path, std::string* error);

#endif
