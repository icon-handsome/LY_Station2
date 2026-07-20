#pragma once

// 点云坐标变换工具（Eigen 实现，不依赖系统 PCL）。
// 用于 LB 位姿解算后的分段点云统一坐标系变换：p' = p × (T0' × T)。

#include <QtCore/QString>

#include <array>
#include <mutex>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/// 进程级点云算法串行化锁（与第一工位约定一致，避免并发写同一缓冲）。
std::mutex& pointCloudAlgorithmMutex();

/// 行优先 4×4 矩阵乘法：out = left × right
std::array<float, 16> multiplyRowMajor4x4(
    const std::array<float, 16>& left,
    const std::array<float, 16>& right);

/* 将点云变换到统一坐标系，等价于 p' = p × (T0' × T)（行向量约定）
 *
 * @param calibrationMatrixT0Prime  标定矩阵 T0'（缺省单位阵）
 * @param stereoTrackingMatrixT     LB 跟踪矩阵 T（Rt_global）
 */
bool transformPointCloudFrame(
    const PointCloudFrame& input,
    const std::array<float, 16>& calibrationMatrixT0Prime,
    const std::array<float, 16>& stereoTrackingMatrixT,
    PointCloudFrame* output,
    QString* message = nullptr);

}  // namespace scan_tracking::mech_eye
