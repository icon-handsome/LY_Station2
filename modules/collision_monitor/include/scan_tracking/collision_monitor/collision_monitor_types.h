#pragma once

#include <cstddef>

#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QMetaType>

namespace scan_tracking::collision_monitor {

// 碰撞告警等级，数值越大表示风险越高
enum class CollisionAlertLevel {
    None = 0,       // 无告警
    Warning = 1,    // 预警：障碍物接近但未达危险阈值
    Danger = 2,     // 危险：距离过近，需减速或停止
    Emergency = 3,  // 紧急：极近距离，需立即停机
};

// 单个命名障碍物的最近距离
struct CollisionObstacleDistance {
    QString name;           // 障碍物名称（来自配置或检测算法）
    double distanceM = 0.0; // 到该障碍物的最近距离（米）
};

// 单帧碰撞检测结果，由底层碰撞检测库输出并映射为本结构
struct CollisionDetectResult {
    int roiPointCount = 0;              // 感兴趣区域内点云总数
    int movingVoxelCount = 0;           // 相对背景发生运动的体素数量
    int movingPointCount = 0;           // 运动点数量
    double minDistanceM = 0.0;          // 最近障碍物距离（米）
    QString nearestObstacle;            // 最近障碍物名称
    CollisionAlertLevel instantLevel = CollisionAlertLevel::None;   // 当前帧即时告警等级
    CollisionAlertLevel confirmedLevel = CollisionAlertLevel::None; // 经连续帧确认后的告警等级
    int consecutiveAlarmFrames = 0;       // 连续触发告警的帧数
    QVector<CollisionObstacleDistance> obstacleDistances; // 各障碍物距离列表
    bool valid = false;                   // 结果是否有效
};

// 点云视图，仅持有指针与点数，不拷贝数据
struct CollisionCloudView {
    const float* xyz = nullptr;  // 交错排列的 x、y、z 坐标（单位：米）
    size_t pointCount = 0;       // 点数量（坐标数组长度为 点数 × 3）
};

// 碰撞检测模块错误信息
struct CollisionMonitorError {
    int statusCode = 0;  // 底层 C 接口状态码
    QString message;     // 可读错误描述
};

}  // namespace scan_tracking::collision_monitor

Q_DECLARE_METATYPE(scan_tracking::collision_monitor::CollisionDetectResult)
