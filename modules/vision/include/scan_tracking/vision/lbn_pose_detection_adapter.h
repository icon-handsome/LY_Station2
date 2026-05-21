#pragma once

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig& config);

}  // namespace vision
}  // namespace scan_tracking
