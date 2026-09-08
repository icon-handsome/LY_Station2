#ifndef CONTAINER_TOTAL_LENGTH_MEASUREMENT_H_
#define CONTAINER_TOTAL_LENGTH_MEASUREMENT_H_

#include "PointCloudIO.h"

#include <vector>

#if defined(CONTAINER_TOTAL_LENGTH_EXPORTS)
#define CONTAINER_TOTAL_LENGTH_API __declspec(dllexport)
#else
#define CONTAINER_TOTAL_LENGTH_API __declspec(dllimport)
#endif

struct CONTAINER_TOTAL_LENGTH_API Cylinder 
{
    Eigen::Vector3f point;
    Eigen::Vector3f axis;
    Cylinder();
};

struct CONTAINER_TOTAL_LENGTH_API EndDetectionResult
{
    double templatePosition;
    double coarsePosition;
    double refinedPosition;
    bool usedEndNormals;
    int coarsePointCount;
    int refinedPointCount;

    EndDetectionResult();
};

struct CONTAINER_TOTAL_LENGTH_API MeasurementResult 
{
    Cylinder cylinder;
    Eigen::Matrix4f icpTransform;
    double icpFitness;
    bool icpConverged;
    EndDetectionResult ends[2];
    double length;

    MeasurementResult();
};

CONTAINER_TOTAL_LENGTH_API CloudPtr VoxelDownsample(CloudConstPtr cloud, float voxelSize);
CONTAINER_TOTAL_LENGTH_API CloudPtr RemoveStatisticalOutliers(CloudConstPtr cloud, int meanK, double stdMultiplier);

CONTAINER_TOTAL_LENGTH_API bool MeasureContainerLength(CloudConstPtr rawInput,
                                                       CloudConstPtr templateCloud,
                                                       const Config& config,
                                                       MeasurementResult* result,
                                                       std::string* error);

#endif
