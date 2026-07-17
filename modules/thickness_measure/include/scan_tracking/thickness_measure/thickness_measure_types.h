#pragma once

#include <cstddef>

#include <QtCore/QString>

namespace scan_tracking::thickness_measure {

enum class ThicknessMethod {
    NearestBetweenSurfaces = 0,
    TangentPlaneProjection = 1,
};

struct ThicknessPoint3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ThicknessPreprocessConfig {
    bool enableOutlierRemoval = true;
    int meanK = 10;
    double stddevMulThresh = 5.0;
    bool enableVoxelDownsample = true;
    double leafSize = 1.0;
};

struct ThicknessIcpConfig {
    int maxIterations = 100;
    double maxCorrespondenceDistance = 100.0;
    double transformationEpsilon = 0.001;
    double euclideanFitnessEpsilon = 0.001;
};

struct ThicknessConfig {
    ThicknessPreprocessConfig preprocess;
    ThicknessIcpConfig icp;
    ThicknessMethod thicknessMethod = ThicknessMethod::NearestBetweenSurfaces;
    ThicknessPoint3d axisPoint;
    ThicknessPoint3d axisDirection;
    ThicknessPoint3d templateFeaturePoints[2];
};

struct ThicknessPairMeasurement {
    double innerIcpFitnessScore = 0.0;
    double outerIcpFitnessScore = 0.0;
    double thicknessMm = 0.0;
    ThicknessMethod thicknessMethod = ThicknessMethod::NearestBetweenSurfaces;
    ThicknessPoint3d templateFeaturePoints[2];
    ThicknessPoint3d nearestScanPoints[2];  // [0]=outer, [1]=inner
    ThicknessPoint3d projectedPoints[2];
    bool valid = false;
};

struct ThicknessAverageMeasurement {
    double thicknessMm = 0.0;
    size_t pairCount = 0;
    size_t successCount = 0;
    bool valid = false;
};

struct ThicknessCloudView {
    const float* xyz = nullptr;  // interleaved x,y,z
    size_t pointCount = 0;
};

struct ThicknessPairClouds {
    ThicknessCloudView inner;
    ThicknessCloudView outer;
};

struct ThicknessMeasureError {
    int statusCode = 0;
    QString message;
};

}  // namespace scan_tracking::thickness_measure
