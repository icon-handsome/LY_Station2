#include "scan_tracking/collision_monitor/collision_monitor_pipeline.h"

#include <QtCore/QLoggingCategory>

Q_LOGGING_CATEGORY(LOG_COLLISION_PIPELINE, "collision_monitor.pipeline")

namespace scan_tracking::collision_monitor {

QString CollisionMonitorPipeline::alertLevelName(CollisionAlertLevel level)
{
    switch (level) {
        case CollisionAlertLevel::Warning:
            return QStringLiteral("WARNING");
        case CollisionAlertLevel::Danger:
            return QStringLiteral("DANGER");
        case CollisionAlertLevel::Emergency:
            return QStringLiteral("EMERGENCY");
        default:
            return QStringLiteral("NONE");
    }
}

CollisionMonitorPipeline::CollisionMonitorPipeline(QObject* parent)
    : QObject(parent)
{
}

CollisionMonitorPipeline::~CollisionMonitorPipeline()
{
    stop();
}

bool CollisionMonitorPipeline::start(const QString& configPath)
{
    stop();

    CollisionMonitorError error;
    if (!m_service.initializeFromIni(configPath, &error)) {
        qWarning(LOG_COLLISION_PIPELINE).noquote()
            << QStringLiteral("[CollisionMonitor] init failed:")
            << error.message;
        m_ready = false;
        return false;
    }

    m_ready = true;
    m_backgroundSet = false;
    qInfo(LOG_COLLISION_PIPELINE).noquote()
        << QStringLiteral("[CollisionMonitor] pipeline ready, config=")
        << m_service.configPath();
    return true;
}

void CollisionMonitorPipeline::stop()
{
    m_service.shutdown();
    m_ready = false;
    m_backgroundSet = false;
}

void CollisionMonitorPipeline::onPointCloudFrame(QVector<float> xyz)
{
    if (!m_ready || xyz.isEmpty()) {
        return;
    }

    const size_t pointCount = xyz.size() / 3;
    if (pointCount == 0) {
        return;
    }

    CollisionMonitorError error;
    if (!m_backgroundSet) {
        if (!m_service.setBackground(xyz.constData(), pointCount, &error)) {
            qWarning(LOG_COLLISION_PIPELINE).noquote()
                << QStringLiteral("[CollisionMonitor] setBackground failed:")
                << error.message;
            return;
        }
        m_backgroundSet = true;
        qInfo(LOG_COLLISION_PIPELINE).noquote()
            << QStringLiteral("[CollisionMonitor] background captured, points=")
            << pointCount
            << QStringLiteral(" (ensure scene has no hoisted object)");
        return;
    }

    CollisionDetectResult result;
    if (!m_service.detect(xyz.constData(), pointCount, &result, &error)) {
        qWarning(LOG_COLLISION_PIPELINE).noquote()
            << QStringLiteral("[CollisionMonitor] detect failed:")
            << error.message;
        return;
    }

    const QString line = QStringLiteral(
                             "[CollisionMonitor] confirmed=%1 instant=%2 min_dist_m=%3 "
                             "moving_pts=%4 nearest=%5")
                             .arg(alertLevelName(result.confirmedLevel))
                             .arg(alertLevelName(result.instantLevel))
                             .arg(result.minDistanceM, 0, 'f', 3)
                             .arg(result.movingPointCount)
                             .arg(result.nearestObstacle);

    if (result.confirmedLevel == CollisionAlertLevel::None) {
        qInfo(LOG_COLLISION_PIPELINE).noquote() << line;
    } else {
        qWarning(LOG_COLLISION_PIPELINE).noquote() << line;
    }
}

}  // namespace scan_tracking::collision_monitor
