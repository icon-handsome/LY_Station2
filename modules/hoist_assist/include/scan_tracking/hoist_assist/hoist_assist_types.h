#pragma once

#include <QtCore/QMetaType>
#include <QtCore/QString>

#include "scan_tracking/collision_monitor/collision_monitor_types.h"

namespace scan_tracking::hoist_assist {

enum class HoistAssistState {
    Idle = 0,
    Running,
    Warning,
    Unsafe,
    Fault,
    Stopped,
};

enum class TfSensorId {
    Sensor1 = 0,
    Sensor2 = 1,
};

struct TfDistanceSample {
    int distanceCm = 0;
    bool valid = false;
};

struct HoistAssistResult {
    TfDistanceSample tf1;
    TfDistanceSample tf2;
    bool tfPassed = false;

    bool collisionResultReceived = false;
    bool collisionSafe = false;
    collision_monitor::CollisionAlertLevel collisionLevel =
        collision_monitor::CollisionAlertLevel::None;

    bool hikResultReceived = false;
    bool hikPassed = false;

    bool allChecksPassed = false;
    QString message;
};

}  // namespace scan_tracking::hoist_assist

Q_DECLARE_METATYPE(scan_tracking::hoist_assist::HoistAssistState)
Q_DECLARE_METATYPE(scan_tracking::hoist_assist::TfSensorId)
Q_DECLARE_METATYPE(scan_tracking::hoist_assist::TfDistanceSample)
Q_DECLARE_METATYPE(scan_tracking::hoist_assist::HoistAssistResult)
