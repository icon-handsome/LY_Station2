#pragma once

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

struct LbnPoseMarkerDebug {
    int centers2dCount = 0;
    int centers3dCount = 0;
    /** 最多记录 32 个圆心，避免离线工具输出过长。 */
    float centers2dU[32] = {};
    float centers2dV[32] = {};
    float centers3dX[32] = {};
    float centers3dY[32] = {};
    float centers3dZ[32] = {};
};

LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig& config);

/** 与 runLbnPoseDetection 相同，并填充 2D/3D 标记调试信息（可选）。 */
LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig& config,
    LbnPoseMarkerDebug* markerDebug);

}  // namespace vision
}  // namespace scan_tracking
