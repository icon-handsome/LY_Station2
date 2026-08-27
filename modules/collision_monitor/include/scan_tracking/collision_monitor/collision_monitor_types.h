#pragma once

#include <cstddef>

#include <QtCore/QString>
#include <QtCore/QVector>

namespace scan_tracking::collision_monitor {

enum class CollisionAlertLevel {
    None = 0,
    Warning = 1,
    Danger = 2,
    Emergency = 3,
};

struct CollisionObstacleDistance {
    QString name;
    double distanceM = 0.0;
};

struct CollisionDetectResult {
    int roiPointCount = 0;
    int movingVoxelCount = 0;
    int movingPointCount = 0;
    double minDistanceM = 0.0;
    QString nearestObstacle;
    CollisionAlertLevel instantLevel = CollisionAlertLevel::None;
    CollisionAlertLevel confirmedLevel = CollisionAlertLevel::None;
    int consecutiveAlarmFrames = 0;
    QVector<CollisionObstacleDistance> obstacleDistances;
    bool valid = false;
};

struct CollisionCloudView {
    const float* xyz = nullptr;  // interleaved x,y,z in metres
    size_t pointCount = 0;
};

struct CollisionMonitorError {
    int statusCode = 0;
    QString message;
};

}  // namespace scan_tracking::collision_monitor
