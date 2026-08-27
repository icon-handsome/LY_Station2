// 碰撞检测服务实现：封装 CollisionDetection.dll C API，提供线程安全的 Qt 接口。

#include "scan_tracking/collision_monitor/collision_monitor_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <mutex>

#include "collision_detection_c_api.h"

Q_LOGGING_CATEGORY(LOG_COLLISION_MONITOR, "collision_monitor.service")

namespace scan_tracking::collision_monitor {

namespace {

/// 将底层 C API 告警等级枚举映射为本模块 CollisionAlertLevel。
CollisionAlertLevel ToAlertLevel(int level)
{
    switch (level) {
        case CD_ALERT_WARNING:
            return CollisionAlertLevel::Warning;
        case CD_ALERT_DANGER:
            return CollisionAlertLevel::Danger;
        case CD_ALERT_EMERGENCY:
            return CollisionAlertLevel::Emergency;
        default:
            return CollisionAlertLevel::None;
    }
}

/// 填充错误输出结构；error 为 nullptr 时静默忽略。
void FillError(CollisionMonitorError* error, cd_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    error->message = message.isEmpty()
        ? QString::fromUtf8(cd_status_string(status))
        : message;
}

/// 安全销毁 cd_context，并将指针置空。
void DestroyContext(cd_context*& ctx)
{
    if (ctx != nullptr) {
        cd_destroy(ctx);
        ctx = nullptr;
    }
}

/// 将 cd_detect_result 逐字段拷贝到 CollisionDetectResult，含障碍物距离列表。
void FillResult(const cd_detect_result& src, CollisionDetectResult* dst)
{
    dst->roiPointCount = src.roi_point_count;
    dst->movingVoxelCount = src.moving_voxel_count;
    dst->movingPointCount = src.moving_point_count;
    dst->minDistanceM = src.min_distance_m;
    dst->nearestObstacle = QString::fromUtf8(src.nearest_obstacle);
    dst->instantLevel = ToAlertLevel(src.instant_level);
    dst->confirmedLevel = ToAlertLevel(src.confirmed_level);
    dst->consecutiveAlarmFrames = src.consecutive_alarm_frames;
    dst->valid = src.valid != 0;

    dst->obstacleDistances.clear();
    dst->obstacleDistances.reserve(src.obstacle_count);
    for (int i = 0; i < src.obstacle_count; ++i) {
        CollisionObstacleDistance item;
        item.name = QString::fromUtf8(src.distances[i].name);
        item.distanceM = src.distances[i].distance_m;
        dst->obstacleDistances.push_back(item);
    }
}

}  // namespace

/// 服务私有实现：持有 DLL 上下文、互斥锁及运行状态。
struct CollisionMonitorService::Impl {
    mutable std::mutex mutex;   ///< 串行化所有 C API 调用
    cd_context* ctx = nullptr;  ///< CollisionDetection.dll 检测上下文
    bool backgroundSet = false; ///< 背景点云是否已设置
    QString configPath;         ///< 当前使用的配置文件路径
};

CollisionMonitorService::CollisionMonitorService()
    : m_impl(new Impl)
{
}

CollisionMonitorService::~CollisionMonitorService()
{
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString CollisionMonitorService::defaultConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/collision_detection/collision.ini"));
}

bool CollisionMonitorService::isReady() const
{
    if (m_impl == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->ctx != nullptr;
}

bool CollisionMonitorService::hasBackground() const
{
    if (m_impl == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->ctx != nullptr && m_impl->backgroundSet;
}

QString CollisionMonitorService::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

/// 从 INI 配置文件创建检测上下文。
///
/// 步骤：
/// 1. 销毁已有上下文（若存在）
/// 2. 解析 configPath，空则回退到 defaultConfigPath()
/// 3. 校验文件存在后调用 cd_create_from_ini 创建上下文
///
/// 成功返回 true，ctx 与 configPath 写入 Impl；失败时 error 携带原因。
bool CollisionMonitorService::initializeFromIni(const QString& configPath, CollisionMonitorError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->backgroundSet = false;
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, CD_ERR_CONFIG, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_COLLISION_MONITOR) << "Config missing:" << resolved;
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    cd_context* ctx = nullptr;
    const cd_status status = cd_create_from_ini(pathUtf8.constData(), &ctx);
    if (status != CD_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(cd_status_string(status)));
        qWarning(LOG_COLLISION_MONITOR) << "cd_create_from_ini failed:" << cd_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    qInfo(LOG_COLLISION_MONITOR) << "CollisionMonitor ready, config:" << resolved;
    return true;
}

/// 设置静态背景点云，作为运动检测的参考基准。
///
/// 要求：采集背景时场景中不得有吊运物体，否则后续会将物体误判为静止背景。
/// 调用 cd_set_background 后会重置 DLL 内部连续帧告警状态。
bool CollisionMonitorService::setBackground(
    const float* xyz,
    size_t pointCount,
    CollisionMonitorError* error)
{
    if (xyz == nullptr || pointCount == 0) {
        FillError(error, CD_ERR_INVALID_ARG, QStringLiteral("Background cloud is empty"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, CD_ERR_NOT_INITIALIZED, QStringLiteral("CollisionMonitorService not initialized"));
        return false;
    }

    const cd_status status = cd_set_background(m_impl->ctx, xyz, pointCount);
    if (status != CD_OK) {
        FillError(error, status, QString::fromUtf8(cd_status_string(status)));
        qWarning(LOG_COLLISION_MONITOR) << "cd_set_background failed:" << cd_status_string(status);
        return false;
    }

    m_impl->backgroundSet = true;
    qInfo(LOG_COLLISION_MONITOR) << "Background set, points:" << pointCount;
    return true;
}

bool CollisionMonitorService::setBackground(
    const CollisionCloudView& cloud,
    CollisionMonitorError* error)
{
    return setBackground(cloud.xyz, cloud.pointCount, error);
}

/// 对一帧实时点云执行碰撞检测（核心接口）。
///
/// 前置条件：已 initializeFromIni 且 setBackground。
/// 流程：
/// 1. 校验 out 指针与初始化/背景状态
/// 2. 调用 cd_detect，传入 xyz 与点数
/// 3. 将 cd_detect_result 映射为 CollisionDetectResult
///
/// DLL 内部维护连续帧告警确认逻辑，confirmedLevel 需多帧一致才升级。
bool CollisionMonitorService::detect(
    const float* xyz,
    size_t pointCount,
    CollisionDetectResult* out,
    CollisionMonitorError* error)
{
    if (out == nullptr) {
        FillError(error, CD_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, CD_ERR_NOT_INITIALIZED, QStringLiteral("CollisionMonitorService not initialized"));
        return false;
    }
    if (!m_impl->backgroundSet) {
        FillError(error, CD_ERR_NO_BACKGROUND, QStringLiteral("Background cloud not set"));
        return false;
    }

    cd_detect_result result{};
    char message[512] = {0};
    const cd_status status = cd_detect(
        m_impl->ctx,
        xyz,
        pointCount,
        &result,
        message,
        sizeof(message));

    if (status != CD_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(cd_status_string(status)));
        qWarning(LOG_COLLISION_MONITOR) << "cd_detect failed:" << message;
        return false;
    }

    FillResult(result, out);
    return true;
}

bool CollisionMonitorService::detect(
    const CollisionCloudView& cloud,
    CollisionDetectResult* out,
    CollisionMonitorError* error)
{
    return detect(cloud.xyz, cloud.pointCount, out, error);
}

/// 重置 DLL 连续帧告警状态，保留上下文与背景点云。
void CollisionMonitorService::resetState()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx != nullptr) {
        cd_reset_state(m_impl->ctx);
    }
}

/// 销毁检测上下文，清空背景标志与配置路径。
void CollisionMonitorService::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->backgroundSet = false;
    m_impl->configPath.clear();
}

}  // namespace scan_tracking::collision_monitor
