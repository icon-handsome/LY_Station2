#pragma once

#include <QtCore/QMetaType>
#include <QtCore/QString>

#include "scan_tracking/collision_monitor/collision_monitor_types.h"

namespace scan_tracking::hoist_assist {

// 吊装辅助运行状态
enum class HoistAssistState {
    Idle = 0,   // 空闲：未启动
    Running,    // 运行中：等待或已通过检查
    Warning,    // 预警（预留，当前未使用）
    Unsafe,     // 不安全：任一类传感器判定未通过
    Fault,      // 故障（预留，当前未使用）
    Stopped,    // 已停止
};

// TF 激光测距传感器编号（现场两路串口）
enum class TfSensorId {
    Sensor1 = 0,  // TF1，对应现场 COM5
    Sensor2 = 1,  // TF2，对应现场 COM6
};

// 单路 TF 测距采样
struct TfDistanceSample {
    int distanceCm = 0;  // 距离（厘米）
    bool valid = false;  // 是否有效；校验和/强度/超时等传输错误时应为 false
};

// 吊装辅助综合判定结果（三类传感器汇总）
struct HoistAssistResult {
    TfDistanceSample tf1;  // TF1 最新采样
    TfDistanceSample tf2;  // TF2 最新采样
    bool tfPassed = false; // 两路 TF 均满足阈值条件

    bool collisionResultReceived = false;  // 是否已收到碰撞检测结果
    bool collisionSafe = false;            // 碰撞检测判定安全（告警等级为 None）
    collision_monitor::CollisionAlertLevel collisionLevel =
        collision_monitor::CollisionAlertLevel::None;  // 当前碰撞告警等级

    bool hikResultReceived = false;  // 是否已收到海康相机判定结果
    bool hikPassed = false;          // 海康 C 焊缝/ROI 模型判定是否通过

    bool allChecksPassed = false;  // TF、碰撞、海康三类检查全部通过
    QString message;               // 面向界面/日志的状态说明
};

}  // namespace scan_tracking::hoist_assist

Q_DECLARE_METATYPE(scan_tracking::hoist_assist::HoistAssistState)
Q_DECLARE_METATYPE(scan_tracking::hoist_assist::TfSensorId)
Q_DECLARE_METATYPE(scan_tracking::hoist_assist::TfDistanceSample)
Q_DECLARE_METATYPE(scan_tracking::hoist_assist::HoistAssistResult)
