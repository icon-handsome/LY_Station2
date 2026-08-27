#pragma once

#include <QtCore/QObject>
#include <QtCore/QElapsedTimer>
#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/collision_monitor/collision_monitor_service.h"

namespace scan_tracking::collision_monitor {

// 碰撞检测流水线：接收 Livox 雷达点云帧，驱动碰撞检测服务并输出告警日志
//
// 典型流程：
// 1. 启动 — 加载碰撞检测配置文件
// 2. 首帧点云 — 自动作为背景（需确保场景中无吊运物体）
// 3. 后续帧 — 逐帧检测并记录确认告警等级与即时告警等级
class CollisionMonitorPipeline : public QObject {
    Q_OBJECT

public:
    explicit CollisionMonitorPipeline(QObject* parent = nullptr);
    ~CollisionMonitorPipeline() override;

    // 流水线是否已成功启动（配置加载完成）
    bool isReady() const { return m_ready; }

public slots:
    // 启动流水线；配置路径为空时使用默认配置文件
    bool start(const QString& configPath = QString());

    // 停止流水线并释放检测上下文
    void stop();

    // 点云帧处理入口（核心接口）
    // 输入为交错排列的 x、y、z 坐标（单位：米）；首帧设背景，后续帧执行检测
    void onPointCloudFrame(QVector<float> xyz);

private:
    // 将告警等级枚举转为日志用字符串
    static QString alertLevelName(CollisionAlertLevel level);

    CollisionMonitorService m_service;
    bool m_ready = false;              // 配置是否加载成功
    bool m_backgroundSet = false;      // 背景点云是否已采集
    CollisionAlertLevel m_lastConfirmedLevel = CollisionAlertLevel::None;  // 上次记录的确认告警等级
    CollisionAlertLevel m_lastInstantLevel = CollisionAlertLevel::None;    // 上次记录的即时告警等级
    QElapsedTimer m_lastLogTimer;      // 用于节流周期性日志输出
};

}  // namespace scan_tracking::collision_monitor
