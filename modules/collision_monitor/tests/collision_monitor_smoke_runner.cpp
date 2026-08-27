#include <cstdio>
#include <cstdlib>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QString>

#include "scan_tracking/collision_monitor/collision_monitor_service.h"

using scan_tracking::collision_monitor::CollisionAlertLevel;
using scan_tracking::collision_monitor::CollisionDetectResult;
using scan_tracking::collision_monitor::CollisionMonitorError;
using scan_tracking::collision_monitor::CollisionMonitorService;

namespace {

const char* AlertLevelName(CollisionAlertLevel level)
{
    switch (level) {
        case CollisionAlertLevel::Warning:
            return "WARNING";
        case CollisionAlertLevel::Danger:
            return "DANGER";
        case CollisionAlertLevel::Emergency:
            return "EMERGENCY";
        default:
            return "NONE";
    }
}

QString DefaultDataDir()
{
    return QStringLiteral("D:/work/LY/第二工位测量源码/辅助吊装中的碰撞检测-V1.2");
}

QString DefaultConfigPath()
{
    const QString deployed = CollisionMonitorService::defaultConfigPath();
    if (QFileInfo::exists(deployed)) {
        return deployed;
    }
    return QDir(DefaultDataDir()).filePath(QStringLiteral("collision.ini"));
}

bool LoadXyzPointCloud(const QString& path, std::vector<float>* xyz, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("Cannot open: %1").arg(path);
        }
        return false;
    }

    xyz->clear();
    while (!file.atEnd()) {
        QString line = QString::fromUtf8(file.readLine()).trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
            continue;
        }

        line.replace(QLatin1Char(','), QLatin1Char(' '));
        const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (parts.size() < 3) {
            continue;
        }

        bool okX = false;
        bool okY = false;
        bool okZ = false;
        const float x = parts[0].toFloat(&okX);
        const float y = parts[1].toFloat(&okY);
        const float z = parts[2].toFloat(&okZ);
        if (!okX || !okY || !okZ) {
            continue;
        }

        xyz->push_back(x);
        xyz->push_back(y);
        xyz->push_back(z);
    }

    if (xyz->empty()) {
        if (error) {
            *error = QStringLiteral("No points loaded: %1").arg(path);
        }
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString dataDir = (argc >= 2) ? QString::fromLocal8Bit(argv[1]) : DefaultDataDir();
    const QString configPath = (argc >= 3) ? QString::fromLocal8Bit(argv[2]) : DefaultConfigPath();
    const QString currentPath = QDir(dataDir).filePath(QStringLiteral("sample_cloud.xyz"));
    const QString backgroundPath = QDir(dataDir).filePath(QStringLiteral("background_cloud.xyz"));

    std::printf("CollisionMonitor IPC smoke (CollisionMonitorService)\n");
    std::printf("Data dir: %s\n", qPrintable(dataDir));
    std::printf("Config:   %s\n", qPrintable(configPath));
    std::printf("Current:  %s\n", qPrintable(currentPath));
    std::printf("Background: %s\n", qPrintable(backgroundPath));
    std::fflush(stdout);

    std::vector<float> currentXyz;
    std::vector<float> backgroundXyz;
    QString loadError;
    if (!LoadXyzPointCloud(currentPath, &currentXyz, &loadError) ||
        !LoadXyzPointCloud(backgroundPath, &backgroundXyz, &loadError)) {
        std::fprintf(stderr, "Load failed: %s\n", qPrintable(loadError));
        return 2;
    }
    std::printf("Loaded current/background points: %zu / %zu\n",
                currentXyz.size() / 3,
                backgroundXyz.size() / 3);
    std::fflush(stdout);

    CollisionMonitorService service;
    CollisionMonitorError error;
    if (!service.initializeFromIni(configPath, &error)) {
        std::fprintf(stderr, "initializeFromIni failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 3;
    }

    if (!service.setBackground(backgroundXyz.data(), backgroundXyz.size() / 3, &error)) {
        std::fprintf(stderr, "setBackground failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 4;
    }

    CollisionDetectResult result;
    if (!service.detect(currentXyz.data(), currentXyz.size() / 3, &result, &error) || !result.valid) {
        std::fprintf(stderr, "detect failed: %s (code=%d)\n",
                     qPrintable(error.message), error.statusCode);
        return 5;
    }

    std::printf("\n=== collision_monitor smoke result ===\n");
    std::printf("roi_point_count: %d\n", result.roiPointCount);
    std::printf("moving_voxel_count: %d\n", result.movingVoxelCount);
    std::printf("moving_point_count: %d\n", result.movingPointCount);
    std::printf("min_distance_m: %.6f\n", result.minDistanceM);
    std::printf("nearest_obstacle: %s\n", qPrintable(result.nearestObstacle));
    std::printf("instant_level: %s\n", AlertLevelName(result.instantLevel));
    std::printf("confirmed_level: %s\n", AlertLevelName(result.confirmedLevel));
    std::printf("consecutive_alarm_frames: %d\n", result.consecutiveAlarmFrames);
    std::printf("valid: %d\n", result.valid ? 1 : 0);

    for (const auto& item : result.obstacleDistances) {
        std::printf("  obstacle %s: %.6f m\n",
                    qPrintable(item.name),
                    item.distanceM);
    }
    std::fflush(stdout);

    return 0;
}
