/**
 * @file tracking_service.h
 * @brief 跟踪服务头文件，提供检测和位姿校验功能
 *
 * 本模块负责协调点云采集、综合检测和位姿校验等核心业务逻辑，
 * 整合第一工位检测算法和LB位姿检测算法，为状态机提供统一的检测接口。
 */

#pragma once

#include <array>
#include <functional>
#include <string>

#include <QtCore/QJsonObject>
#include <QtCore/QMetaType>
#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking {
namespace tracking {

/// 第一工位算法测量项（对应 HMI event.inspection.finished 协议字段）
struct InspectionMeasurement {
    float headAngleTol = 0.0f;        ///< head_angle_tol 坡口角
    float straightHeightTol = 0.0f;   ///< straight_height_tol 直边高度
    float straightSlopeTol = 0.0f;    ///< straight_slope_tol 直边斜度
    float innerDiameter = 0.0f;       ///< inner_diameter 内径
    float bluntHeightTol = 0.0f;      ///< blunt_height_tol 钝边高度
    float innerDiameterTol = 0.0f;    ///< inner_diameter_tol 内径圆度
    float holeDiameterTol = 0.0f;     ///< hole_diameter_tol 开孔直径
    float headDepthTol = 0.0f;        ///< head_depth_tol 封头深度
};

/// 将测量项写入 JSON payload（协议 snake_case 字段名）
void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement);

/// 检测结果结构体，封装第一工位综合检测的完整输出
struct InspectionResult {
    quint16 resultCode = 0;           ///< 结果码：1-成功，2-失败
    quint16 ngReasonWord0 = 0;        ///< NG 原因字 0
    quint16 ngReasonWord1 = 0;        ///< NG 原因字 1
    quint16 measureItemCount = 0;     ///< 测量项数量
    int segmentCount = 0;             ///< 扫描分段总数
    int totalPointCount = 0;          ///< 总点云点数
    float offsetXmm = 0.0f;           ///< X 方向偏移量（mm）
    float offsetYmm = 0.0f;           ///< Y 方向偏移量（mm）
    float offsetZmm = 0.0f;           ///< Z 方向偏移量（mm）
    float stableOffsetXmm = 0.0f;     ///< 稳定后 X 方向偏移量（mm）
    float stableOffsetYmm = 0.0f;     ///< 稳定后 Y 方向偏移量（mm）
    float stableOffsetZmm = 0.0f;     ///< 稳定后 Z 方向偏移量（mm）
    InspectionMeasurement measurement; ///< 算法测量项（HMI 结构化上报）
    QString outlinerErrorLog;         ///< 外表面错误日志
    QString inlinerErrorLog;          ///< 内表面错误日志
    QString message;                  ///< 结果描述信息
};

/// 位姿校验结果结构体，封装LB位姿检测的输出
struct PoseCheckResult {
    bool invoked = false;             ///< 是否已调用位姿检测算法
    bool success = false;             ///< 位姿检测是否成功
    quint16 resultCode = 7;           ///< 结果码：1-成功，其他-失败
    int inputPointCount = 0;          ///< 输入点云点数
    double poseDeviationMm = 0.0;     ///< 位姿偏差（mm）
    std::array<double, 16> rt = {     ///< 4x4 位姿变换矩阵（RT）
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    QString message;                  ///< 结果描述信息

    /// 判断是否有有效的位姿矩阵
    // @return 成功且结果码为1时返回true
    bool hasPoseMatrix() const { return success && resultCode == 1; }
};

/// 蓝友/第一工位检测结果就绪回调（供 HMI 演示推送等场景注入，避免 tracking 反向依赖 hmi_server）
using InspectionResultNotifier = std::function<void(const InspectionResult&)>;

/// 跟踪服务类，负责管理检测和位姿校验的核心业务逻辑
class TrackingService {
public:
    /// 获取服务状态文本
    // @return 服务状态描述字符串
    std::string statusText() const;

    /// 注册综合检测结果回调：inspectSegments 返回前必定触发（含蓝友失败/缺段等）
    void setInspectionResultNotifier(InspectionResultNotifier notifier);

    /// 执行分段点云的综合检测（第一工位）
    // @param segmentCaptureResults 分段采集结果映射表（分段索引 -> 采集结果）
    // @param notifyListener 是否触发 InspectionResultNotifier（调试命令应传 false，由 HMI 统一推送一次）
    // @return 检测结果结构体
    InspectionResult inspectSegments(
        const QMap<int, scan_tracking::mech_eye::CaptureResult>& segmentCaptureResults,
        bool notifyListener = true) const;

    /// 执行位姿校验（LB位姿检测）
    // @return 位姿校验结果结构体
    PoseCheckResult checkPose() const;

private:
    /// 统一出口：触发回调后再把结果返回给状态机
    InspectionResult deliverInspectionResult(InspectionResult result, bool notifyListener) const;

    InspectionResultNotifier m_inspectionResultNotifier;
};

}  // namespace tracking
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::tracking::InspectionMeasurement)
