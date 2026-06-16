#pragma once

#include <QString>

namespace scan_tracking {
namespace common {

enum class StationId : int {
    FirstEndCap = 1,     // 第一工位（封头）
    SecondMultiMode = 2  // 第二工位（多模式，本阶段仅骨架）
};

enum class WorkModeId : int {
    Unknown = 0,
    ModeEndCap = 1,           // 第二工位-封头模式（本阶段占位）
    ModeCylinderSemi = 2,     // 第二工位-圆筒半成品（本阶段占位）
    ModeSemiFinished = 3      // 第二工位-半成品/成品（本阶段占位）
};

struct StationProfile {
    StationId stationId = StationId::SecondMultiMode;
    QString stationName = QStringLiteral("第二工位-圆筒半成品");
    WorkModeId defaultWorkMode = WorkModeId::ModeCylinderSemi;
    QString scanPathsConfigPath;
    bool enableLoadGrasp = false;
    bool enableUnloadCalc = false;
    bool enablePoseCheck = false;
    bool enableTelescopicScan = true;
    bool enableHoistAssist = true;
    bool enableCollisionMonitor = true;
};

StationId stationIdFromInt(int value);
int stationIdToInt(StationId stationId);
WorkModeId workModeIdFromString(const QString& value, bool* ok = nullptr);
QString workModeIdToString(WorkModeId workMode);

}  // namespace common
}  // namespace scan_tracking
