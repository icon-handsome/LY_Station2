#pragma once

#include <QtCore/QString>

namespace scan_tracking::weld_measure {

struct WeldSectionMeasurement {
    double mismatchMm = 0.0;
    double reinforcementMm = 0.0;
    double angularityMm = 0.0;
    double includedAngleRad = 0.0;
    double toeCenterX = 0.0;
    double leftToeX = 0.0;
    double leftToeY = 0.0;
    double leftToeZ = 0.0;
    double rightToeX = 0.0;
    double rightToeY = 0.0;
    double rightToeZ = 0.0;
    double leftUndercutMm = 0.0;
    double rightUndercutMm = 0.0;
    double maxUndercutMm = 0.0;
    int undercutRawFlag = 0;
    bool valid = false;
};

struct WeldMeasureError {
    int statusCode = 0;
    QString message;
};

}  // namespace scan_tracking::weld_measure
