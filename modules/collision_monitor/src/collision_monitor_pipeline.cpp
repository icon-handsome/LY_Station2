// 碰撞检测流水线实现：连接 Livox 点云输入与 CollisionMonitorService，输出告警日志。

#include "scan_tracking/collision_monitor/collision_monitor_pipeline.h"

#include <QtCore/QLoggingCategory>

Q_LOGGING_CATEGORY(LOG_COLLISION_PIPELINE, "collision_monitor.pipeline")

namespace scan_tracking::collision_monitor {

/// 将告警等级枚举转为日志用字符串。
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

/// 启动碰撞检测流水线。
///
/// 先 stop() 清理旧状态，再调用 initializeFromIni 加载配置。
/// 成功后 m_ready=true，等待首帧点云作为背景。
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
    m_lastConfirmedLevel = CollisionAlertLevel::None;
    m_lastInstantLevel = CollisionAlertLevel::None;
    m_lastLogTimer.start();
    qInfo(LOG_COLLISION_PIPELINE).noquote()
        << QStringLiteral("[CollisionMonitor] pipeline ready, config=")
        << m_service.configPath();
    return true;
}

/// 停止流水线：销毁检测上下文并重置所有运行时状态。
void CollisionMonitorPipeline::stop()
{
    m_service.shutdown();
    m_ready = false;
    m_backgroundSet = false;
    m_lastConfirmedLevel = CollisionAlertLevel::None;
    m_lastInstantLevel = CollisionAlertLevel::None;
    m_lastLogTimer.invalidate();
}

/// 点云帧处理入口（核心逻辑）。
///
/// 两阶段处理：
/// 1. 背景阶段（首帧）：将当前帧设为静态背景，要求场景中无吊运物体
/// 2. 检测阶段（后续帧）：调用 detect() 获取 instant/confirmed 告警等级
///
/// 日志节流策略（避免 Livox 高帧率导致 I/O 阻塞）：
/// - 告警等级变化时立即输出
/// - 无告警时每 5 秒、有告警时每 2 秒周期性输出
/// - 有 confirmed 告警时用 qWarning，否则 qInfo
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

    // 阶段一：首帧采集背景
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

    // 阶段二：逐帧碰撞检测
    CollisionDetectResult result;
    if (!m_service.detect(xyz.constData(), pointCount, &result, &error)) {
        qWarning(LOG_COLLISION_PIPELINE).noquote()
            << QStringLiteral("[CollisionMonitor] detect failed:")
            << error.message;
        return;
    }

    emit collisionResultReady(result);

    const QString line = QStringLiteral(
                             "[CollisionMonitor] confirmed=%1 instant=%2 min_dist_m=%3 "
                             "moving_pts=%4 nearest=%5")
                             .arg(alertLevelName(result.confirmedLevel))
                             .arg(alertLevelName(result.instantLevel))
                             .arg(result.minDistanceM, 0, 'f', 3)
                             .arg(result.movingPointCount)
                             .arg(result.nearestObstacle);

    // 检测以 Livox 帧率运行；等级变化立即记录，其余按周期节流
    const bool levelChanged = result.confirmedLevel != m_lastConfirmedLevel
        || result.instantLevel != m_lastInstantLevel;
    const qint64 elapsedMs = m_lastLogTimer.isValid() ? m_lastLogTimer.elapsed() : 0;
    const qint64 periodicMs = result.confirmedLevel == CollisionAlertLevel::None
        ? 5000
        : 2000;
    if (levelChanged || elapsedMs >= periodicMs) {
        if (result.confirmedLevel == CollisionAlertLevel::None) {
            qInfo(LOG_COLLISION_PIPELINE).noquote() << line;
        } else {
            qWarning(LOG_COLLISION_PIPELINE).noquote() << line;
        }
        m_lastLogTimer.restart();
    }
    m_lastConfirmedLevel = result.confirmedLevel;
    m_lastInstantLevel = result.instantLevel;
}

}  // namespace scan_tracking::collision_monitor
