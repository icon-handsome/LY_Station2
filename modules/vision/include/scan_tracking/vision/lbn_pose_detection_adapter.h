#pragma once

// LBN（转盘段）位姿检测适配层。
//
// 基于 Mech-Eye 2D 灰度纹理 + 组织化 3D 点云，调用第三方 LBN 库完成：
//   1. 2D 圆心检测 → 3D 圆心关联
//   2. 与模板点云几何匹配，输出 4×4 位姿矩阵 Rt
//
// 由 VisionPipelineService 在转盘段（needMechEye2D=true）的后台线程中调用。
// 封头段不调用本模块，对应 bundle.lbnPoseResult.invoked=false。

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

/// LBN 标记圆心调试快照（仅调试重载填充，最多 32 个点）
struct LbnPoseMarkerDebug {
    int centers2dCount = 0;   ///< 2D 检测到的圆心数
    int centers3dCount = 0;   ///< 关联到 3D 的圆心数
    float centers2dU[32] = {};  ///< 2D 圆心 u 坐标
    float centers2dV[32] = {};  ///< 2D 圆心 v 坐标
    float centers3dX[32] = {};  ///< 3D 圆心 X（mm）
    float centers3dY[32] = {};  ///< 3D 圆心 Y（mm）
    float centers3dZ[32] = {};  ///< 3D 圆心 Z（mm）
};

/* 执行 LBN 位姿检测（同步、阻塞调用，应在后台线程中使用）
 *
 * @param mechEyeResult Mech-Eye Capture2DAnd3D 采集结果（含纹理与组织化点云）
 * @param config        [LbnPose] 模板路径、数据根目录、匹配阈值等
 * @return LbnPoseResult，含 poseMatrix（4×4 Rt）及 matchedPointCount
 */
LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig& config);

/* 与上一重载相同，并可选填充 2D/3D 标记圆心调试信息（离线标定/联调） */
LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig& config,
    LbnPoseMarkerDebug* markerDebug);

}  // namespace vision
}  // namespace scan_tracking
