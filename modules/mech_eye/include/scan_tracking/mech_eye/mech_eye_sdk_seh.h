#pragma once

// Mech-Eye SDK 在多网卡同网段时可能触发原生崩溃（Access Violation）。
// C++ try/catch 拦不住；Windows 下用 SEH 兜住，保证进程可继续跑。

#include <string>
#include <vector>

#include "ErrorStatus.h"
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/CameraProperties.h"
#include "area_scan_3d_camera/Frame2D.h"
#include "area_scan_3d_camera/Frame2DAnd3D.h"
#include "area_scan_3d_camera/Frame3D.h"

namespace scan_tracking {
namespace mech_eye {
namespace sdk_seh {

/// @return 0=未发生 SEH；非 0=Windows 异常码（相机对象可能已损坏，调用方应重建）
unsigned createCamera(mmind::eye::Camera** outCamera);

unsigned connectByIp(
    mmind::eye::Camera* camera,
    const std::string& ip,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

unsigned connectByInfo(
    mmind::eye::Camera* camera,
    const mmind::eye::CameraInfo& info,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

unsigned discoverCameras(
    unsigned timeoutMs,
    std::vector<mmind::eye::CameraInfo>* outList);

unsigned disconnect(mmind::eye::Camera* camera);

unsigned getCameraInfo(
    mmind::eye::Camera* camera,
    mmind::eye::CameraInfo* outInfo,
    mmind::eye::ErrorStatus* outStatus);

unsigned setHeartbeatInterval(mmind::eye::Camera* camera, int intervalMs);

unsigned capture2D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame2D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

unsigned capture3D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame3D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

unsigned capture2DAnd3D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame2DAnd3D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus);

}  // namespace sdk_seh
}  // namespace mech_eye
}  // namespace scan_tracking
