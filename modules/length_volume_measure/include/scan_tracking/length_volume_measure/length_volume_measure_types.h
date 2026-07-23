#pragma once

#include <QtCore/QString>

namespace scan_tracking::length_volume_measure {

struct LengthVolumeConfig {
    bool cropInputCloud = false;
    float cropMinX = -3.402823466e+38f;
    float cropMinY = -3.402823466e+38f;
    float cropMinZ = -3.402823466e+38f;
    float cropMaxX = 3.402823466e+38f;
    float cropMaxY = 3.402823466e+38f;
    float cropMaxZ = 3.402823466e+38f;

    float axisDirectionX = 0.0f;
    float axisDirectionY = 0.0f;
    float axisDirectionZ = 1.0f;
    float endPoint1X = 0.0f;
    float endPoint1Y = 0.0f;
    float endPoint1Z = -500.0f;
    float endPoint2X = 0.0f;
    float endPoint2Y = 0.0f;
    float endPoint2Z = 500.0f;

    float voxelSize = 3.0f;
    int outlierK = 12;
    double outlierStd = 2.0;

    int icpMaxIterations = 80;
    float icpMaxCorrespondenceDistance = 30.0f;
    double icpTransformationEpsilon = 1e-8;
    double icpEuclideanFitnessEpsilon = 1e-6;

    int cylinderFitIterations = 6;
    float cylinderInlierBand = 0.0f;
    bool updateCylinderAxis = false;

    int normalK = 20;
    double endNormalMinAbsDot = 0.90;

    /// "outsideScan" or "templateWindow"
    QString endpointDetectionMethod = QStringLiteral("outsideScan");
    float axialBinWidth = 2.0f;
    float endSearchHalfWidth = 80.0f;
    float outsideScanPeakSearchWidth = 20.0f;
    int minPointsPerBin = 20;
    int minConsecutiveInsideBins = 3;
    int outsideCheckBins = 2;
    float refineHalfWidth = 6.0f;
    double refineEdgePercentile = 0.0001;  // V2.0 outsideScan edge percentile
    float endpointMaxRadius = 0.0f;
};

struct LengthVolumeEndResult {
    double templatePos = 0.0;
    double coarsePos = 0.0;
    double refinedPos = 0.0;
    bool usedEndNormals = false;
    int coarsePointCount = 0;
    int refinedPointCount = 0;
};

struct LengthVolumeMeasurement {
    double lengthMm = 0.0;
    double volumeLiters = 0.0;
    double volumeRadiusMm = 0.0;       // radius used for volume (from caller)
    double fittedOuterRadiusMm = 0.0;

    double cylinderPointX = 0.0;
    double cylinderPointY = 0.0;
    double cylinderPointZ = 0.0;
    double cylinderAxisX = 0.0;
    double cylinderAxisY = 0.0;
    double cylinderAxisZ = 0.0;

    LengthVolumeEndResult end1;
    LengthVolumeEndResult end2;

    double icpFitnessScore = 0.0;
    bool icpConverged = false;
    bool valid = false;
};

struct LengthVolumeMeasureError {
    int statusCode = 0;
    QString message;
};

}  // namespace scan_tracking::length_volume_measure
