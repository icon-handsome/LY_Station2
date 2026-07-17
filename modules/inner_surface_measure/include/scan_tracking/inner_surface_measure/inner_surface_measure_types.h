#pragma once

#include <QtCore/QString>

namespace scan_tracking::inner_surface_measure {

struct InnerSurfaceConfig {
    float voxelSize = 2.0f;
    int outlierK = 12;
    double outlierStd = 2.0;

    int fitIterations = 8;
    float cylinderInlierBand = 0.0f;
    float sectionHalfWidth = 2.0f;

    int icpMaxIterations = 50;
    float icpMaxCorrespondenceDistance = 20.0f;
    double icpTransformationEpsilon = 1e-8;
    double icpEuclideanFitnessEpsilon = 1e-6;

    double cylinderPointX = 0.0;
    double cylinderPointY = 0.0;
    double cylinderPointZ = 0.0;
    double cylinderAxisX = 0.0;
    double cylinderAxisY = 0.0;
    double cylinderAxisZ = 1.0;
    float cylinderRadius = 1.0f;
};

struct InnerSurfaceFrameMeasurement {
    double diameterMm = 0.0;
    double circumferenceMm = 0.0;
    double sectionRoundness[3] = {0.0, 0.0, 0.0};
    double averageRoundness = 0.0;
    double icpFitnessScore = 0.0;
    bool icpConverged = false;
    int usedPointCount = 0;
    bool valid = false;
};

struct InnerSurfaceAverageMeasurement {
    double diameterMm = 0.0;
    double circumferenceMm = 0.0;
    double roundness = 0.0;  // mean of two frame average roundness values
    bool valid = false;
};

struct InnerSurfaceMeasureError {
    int statusCode = 0;
    QString message;
};

}  // namespace scan_tracking::inner_surface_measure
