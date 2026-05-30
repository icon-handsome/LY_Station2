#pragma once

#include <QtCore/QString>

#include <array>
#include <mutex>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/// 进程内 PCL/Eigen 全局串行化锁（Windows 下多线程并发会触发 aligned_free 崩溃）
std::mutex& pointCloudAlgorithmMutex();

struct PointCloudProcessReport {
    int inputPointCount = 0;
    int outputPointCount = 0;
    QString message;
};

/// 对 Mech-Eye 点云执行可配置后处理；enabled=false 时深拷贝直通。
bool processPointCloudFrame(
    const PointCloudFrame& input,
    const common::PointCloudProcessingConfig& config,
    PointCloudFrame* output,
    PointCloudProcessReport* report = nullptr);

/// 行优先 4×4 矩阵乘法：out = left × right（与 StateMachine / LBN 链一致）
std::array<float, 16> multiplyRowMajor4x4(
    const std::array<float, 16>& left,
    const std::array<float, 16>& right);

/// 拼接：将点云变换到统一坐标系，等价于 p' = p × (T0' × T)（行向量约定，与文档一致）
bool transformPointCloudFrame(
    const PointCloudFrame& input,
    const std::array<float, 16>& calibrationMatrixT0Prime,
    const std::array<float, 16>& stereoTrackingMatrixT,
    PointCloudFrame* output,
    QString* message = nullptr);

}  // namespace scan_tracking::mech_eye
