#pragma once

#include <cstddef>

#include <QtCore/QString>

namespace scan_tracking::undercut_length_measure {

struct UndercutPoint3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct UndercutLengthOptions {
    double sectionIntervalMm = 5.0;
    double depthThresholdMm = 0.5;
    int medianFilterWindow = 3;
};

/// One weld section from upstream weld measure (toe + undercut depths).
struct UndercutSectionInput {
    double position = 0.0;
    UndercutPoint3d leftToe;
    double leftDepthMm = 0.0;
    UndercutPoint3d rightToe;
    double rightDepthMm = 0.0;
};

struct UndercutSideLengthResult {
    double maxDepthMm = 0.0;
    double lengthMm = 0.0;
    int maxContinuousSections = 0;
};

struct UndercutLengthMeasurement {
    UndercutSideLengthResult left;
    UndercutSideLengthResult right;
    bool valid = false;
};

struct UndercutLengthMeasureError {
    int statusCode = 0;
    QString message;
};

}  // namespace scan_tracking::undercut_length_measure
