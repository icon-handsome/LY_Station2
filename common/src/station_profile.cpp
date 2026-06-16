#include "scan_tracking/common/station_profile.h"

namespace scan_tracking {
namespace common {

StationId stationIdFromInt(int value)
{
    switch (value) {
    case 1:
        return StationId::FirstEndCap;
    case 2:
    default:
        return StationId::SecondMultiMode;
    }
}

int stationIdToInt(StationId stationId)
{
    return static_cast<int>(stationId);
}

WorkModeId workModeIdFromString(const QString& value, bool* ok)
{
    const QString normalized = value.trimmed().toUpper();
    if (normalized.isEmpty() || normalized == QStringLiteral("UNKNOWN")) {
        if (ok) {
            *ok = true;
        }
        return WorkModeId::Unknown;
    }
    if (normalized == QStringLiteral("MODE_END_CAP")) {
        if (ok) {
            *ok = true;
        }
        return WorkModeId::ModeEndCap;
    }
    if (normalized == QStringLiteral("MODE_CYLINDER_SEMI")) {
        if (ok) {
            *ok = true;
        }
        return WorkModeId::ModeCylinderSemi;
    }
    if (normalized == QStringLiteral("MODE_SEMI_FINISHED")) {
        if (ok) {
            *ok = true;
        }
        return WorkModeId::ModeSemiFinished;
    }

    if (ok) {
        *ok = false;
    }
    return WorkModeId::Unknown;
}

QString workModeIdToString(WorkModeId workMode)
{
    switch (workMode) {
    case WorkModeId::ModeEndCap:
        return QStringLiteral("MODE_END_CAP");
    case WorkModeId::ModeCylinderSemi:
        return QStringLiteral("MODE_CYLINDER_SEMI");
    case WorkModeId::ModeSemiFinished:
        return QStringLiteral("MODE_SEMI_FINISHED");
    case WorkModeId::Unknown:
    default:
        return QStringLiteral("Unknown");
    }
}

}  // namespace common
}  // namespace scan_tracking
