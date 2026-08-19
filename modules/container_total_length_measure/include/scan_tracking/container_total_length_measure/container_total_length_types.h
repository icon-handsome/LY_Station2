#pragma once

#include <QtCore/QString>

namespace scan_tracking::container_total_length_measure {

struct ContainerTotalLengthMeasurement {
    double lengthMm = 0.0;
    double leftEndPosition = 0.0;
    double rightEndPosition = 0.0;
    double icpFitness = 0.0;
    float fittedRadiusMm = 0.0f;
    float cylinderPointX = 0.0f;
    float cylinderPointY = 0.0f;
    float cylinderPointZ = 0.0f;
    float cylinderAxisX = 0.0f;
    float cylinderAxisY = 0.0f;
    float cylinderAxisZ = 0.0f;
    bool icpConverged = false;
    int inputPointCount = 0;
    bool valid = false;
};

struct ContainerTotalLengthError {
    int statusCode = 0;
    QString message;
};

}  // namespace scan_tracking::container_total_length_measure
