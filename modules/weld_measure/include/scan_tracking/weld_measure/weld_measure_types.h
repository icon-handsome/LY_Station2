#pragma once

#include <QtCore/QString>

namespace scan_tracking::weld_measure {

/// 单截面测量结果（错边、余高、咬边、焊趾等）。
struct WeldSectionMeasurement {
    double mismatchMm = 0.0;           ///< 错边量 (mm)
    double reinforcementMm = 0.0;      ///< 余高 (mm)
    double angularityMm = 0.0;         ///< 角变形量 (mm)
    double includedAngleRad = 0.0;     ///< 夹角 (rad)
    double toeCenterX = 0.0;           ///< 焊趾中心 X
    double leftToeX = 0.0;             ///< 左焊趾 X
    double leftToeY = 0.0;             ///< 左焊趾 Y
    double leftToeZ = 0.0;             ///< 左焊趾 Z
    double rightToeX = 0.0;            ///< 右焊趾 X
    double rightToeY = 0.0;            ///< 右焊趾 Y
    double rightToeZ = 0.0;            ///< 右焊趾 Z
    double leftUndercutMm = 0.0;       ///< 左侧咬边深度 (mm)
    double rightUndercutMm = 0.0;      ///< 右侧咬边深度 (mm)
    double maxUndercutMm = 0.0;        ///< 最大咬边深度 (mm)
    int undercutRawFlag = 0;           ///< 算法原始咬边标志位
    bool valid = false;                ///< 本截面结果是否有效
};

/// 单帧正式流水线结果：多截面均值 + 咬边长度/最大深度统计。
struct WeldFrameMeasurement {
    WeldSectionMeasurement average;    ///< 各有效截面指标的平均值
    int validSections = 0;             ///< 有效截面数
    int totalSections = 0;             ///< 总截面数
    double leftUndercutLengthMm = 0.0; ///< 左侧咬边累计长度 (mm)
    double rightUndercutLengthMm = 0.0;///< 右侧咬边累计长度 (mm)
    double leftMaxUndercutDepthMm = 0.0;  ///< 左侧最大咬边深度 (mm)
    double rightMaxUndercutDepthMm = 0.0; ///< 右侧最大咬边深度 (mm)
};

/// 调用失败时的错误信息（对应 DLL 的 wm_status + 可读文案）。
struct WeldMeasureError {
    int statusCode = 0;   ///< 对应 wm_status 枚举值
    QString message;      ///< 可读错误描述
};

}  // namespace scan_tracking::weld_measure
