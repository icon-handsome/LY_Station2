#pragma once

#include <cstddef>

#include <QtCore/QString>

#include "scan_tracking/collision_monitor/collision_monitor_types.h"

namespace scan_tracking::collision_monitor {

// 碰撞检测服务：对底层碰撞检测动态库的薄封装
//
// 职责：
// - 从配置文件创建检测上下文
// - 设置静态背景点云（无吊运物体的空场景）
// - 对实时雷达帧执行运动目标检测与距离告警
//
// 同一实例内的所有调用通过互斥锁串行化，可安全在多线程环境中使用
class CollisionMonitorService {
public:
    CollisionMonitorService();
    ~CollisionMonitorService();

    CollisionMonitorService(const CollisionMonitorService&) = delete;
    CollisionMonitorService& operator=(const CollisionMonitorService&) = delete;

    // 默认配置文件路径：可执行文件目录/config/collision_detection/collision.ini
    static QString defaultConfigPath();

    // 检测上下文是否已成功初始化
    bool isReady() const;

    // 是否已设置背景点云（未设置背景时无法执行检测）
    bool hasBackground() const;

    // 当前使用的配置文件绝对路径
    QString configPath() const;

    // 从配置文件初始化检测上下文
    // 若传入路径为空则使用默认路径；重复调用会先销毁旧上下文再重建，背景与连续帧状态一并重置
    bool initializeFromIni(const QString& configPath = QString(), CollisionMonitorError* error = nullptr);

    // 设置静态背景点云（场景中无吊运物体），会替换先前背景并重置内部连续帧状态
    bool setBackground(
        const float* xyz,
        size_t pointCount,
        CollisionMonitorError* error = nullptr);

    // 设置静态背景点云（点云视图重载）
    bool setBackground(
        const CollisionCloudView& cloud,
        CollisionMonitorError* error = nullptr);

    // 对一帧实时雷达点云执行碰撞检测（核心接口）
    // 连续帧告警确认逻辑保存在检测上下文内部，跨帧状态自动维护
    bool detect(
        const float* xyz,
        size_t pointCount,
        CollisionDetectResult* out,
        CollisionMonitorError* error = nullptr);

    // 对一帧实时雷达点云执行碰撞检测（点云视图重载）
    bool detect(
        const CollisionCloudView& cloud,
        CollisionDetectResult* out,
        CollisionMonitorError* error = nullptr);

    // 重置内部连续帧告警状态，不销毁上下文与背景
    void resetState();

    // 销毁检测上下文并释放资源
    void shutdown();

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::collision_monitor
