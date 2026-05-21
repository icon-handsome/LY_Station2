#include "scan_tracking/vision/lbn_pose_detection_adapter.h"

namespace scan_tracking::vision {

LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig&)
{
    LbnPoseResult result;
    result.invoked = false;
    result.textureWidth = mechEyeResult.texture2D.width;
    result.textureHeight = mechEyeResult.texture2D.height;
    result.pointCloudWidth = mechEyeResult.pointCloud.width;
    result.pointCloudHeight = mechEyeResult.pointCloud.height;
    result.success = false;
    result.message = QStringLiteral("此构建版本中 LBN 位姿检测已禁用。");
    return result;
}

}  // namespace scan_tracking::vision
