#include "scan_tracking/mech_eye/mech_eye_sdk_seh.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace scan_tracking {
namespace mech_eye {
namespace sdk_seh {
namespace {

// 真正的 C++ SDK 调用放在独立函数；含 __try 的包装函数内不得出现需析构的局部对象（MSVC C2712）。

int connectByIpBody(
    mmind::eye::Camera* camera,
    const std::string* ip,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
    *outStatus = camera->connect(*ip, timeoutMs);
    return 0;
}

int connectByInfoBody(
    mmind::eye::Camera* camera,
    const mmind::eye::CameraInfo* info,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
    *outStatus = camera->connect(*info, timeoutMs);
    return 0;
}

int discoverBody(unsigned timeoutMs, std::vector<mmind::eye::CameraInfo>* outList)
{
    *outList = mmind::eye::Camera::discoverCameras(timeoutMs);
    return 0;
}

int disconnectBody(mmind::eye::Camera* camera)
{
    camera->disconnect();
    return 0;
}

int getCameraInfoBody(
    mmind::eye::Camera* camera,
    mmind::eye::CameraInfo* outInfo,
    mmind::eye::ErrorStatus* outStatus)
{
    *outStatus = camera->getCameraInfo(*outInfo);
    return 0;
}

int setHeartbeatBody(mmind::eye::Camera* camera, int intervalMs)
{
    camera->setHeartbeatInterval(intervalMs);
    return 0;
}

int capture2DBody(
    mmind::eye::Camera* camera,
    mmind::eye::Frame2D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
    *outStatus = camera->capture2D(*outFrame, timeoutMs);
    return 0;
}

int capture3DBody(
    mmind::eye::Camera* camera,
    mmind::eye::Frame3D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
    *outStatus = camera->capture3D(*outFrame, timeoutMs);
    return 0;
}

int capture2DAnd3DBody(
    mmind::eye::Camera* camera,
    mmind::eye::Frame2DAnd3D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
    *outStatus = camera->capture2DAnd3D(*outFrame, timeoutMs);
    return 0;
}

}  // namespace

unsigned connectByIp(
    mmind::eye::Camera* camera,
    const std::string& ip,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
#ifdef _MSC_VER
    __try {
        connectByIpBody(camera, &ip, timeoutMs, outStatus);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    connectByIpBody(camera, &ip, timeoutMs, outStatus);
    return 0;
#endif
}

unsigned connectByInfo(
    mmind::eye::Camera* camera,
    const mmind::eye::CameraInfo& info,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
#ifdef _MSC_VER
    __try {
        connectByInfoBody(camera, &info, timeoutMs, outStatus);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    connectByInfoBody(camera, &info, timeoutMs, outStatus);
    return 0;
#endif
}

unsigned discoverCameras(
    unsigned timeoutMs,
    std::vector<mmind::eye::CameraInfo>* outList)
{
#ifdef _MSC_VER
    __try {
        discoverBody(timeoutMs, outList);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    discoverBody(timeoutMs, outList);
    return 0;
#endif
}

unsigned disconnect(mmind::eye::Camera* camera)
{
#ifdef _MSC_VER
    __try {
        disconnectBody(camera);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    disconnectBody(camera);
    return 0;
#endif
}

unsigned getCameraInfo(
    mmind::eye::Camera* camera,
    mmind::eye::CameraInfo* outInfo,
    mmind::eye::ErrorStatus* outStatus)
{
#ifdef _MSC_VER
    __try {
        getCameraInfoBody(camera, outInfo, outStatus);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    getCameraInfoBody(camera, outInfo, outStatus);
    return 0;
#endif
}

unsigned setHeartbeatInterval(mmind::eye::Camera* camera, int intervalMs)
{
#ifdef _MSC_VER
    __try {
        setHeartbeatBody(camera, intervalMs);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    setHeartbeatBody(camera, intervalMs);
    return 0;
#endif
}

unsigned capture2D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame2D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
#ifdef _MSC_VER
    __try {
        capture2DBody(camera, outFrame, timeoutMs, outStatus);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    capture2DBody(camera, outFrame, timeoutMs, outStatus);
    return 0;
#endif
}

unsigned capture3D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame3D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
#ifdef _MSC_VER
    __try {
        capture3DBody(camera, outFrame, timeoutMs, outStatus);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    capture3DBody(camera, outFrame, timeoutMs, outStatus);
    return 0;
#endif
}

unsigned capture2DAnd3D(
    mmind::eye::Camera* camera,
    mmind::eye::Frame2DAnd3D* outFrame,
    unsigned timeoutMs,
    mmind::eye::ErrorStatus* outStatus)
{
#ifdef _MSC_VER
    __try {
        capture2DAnd3DBody(camera, outFrame, timeoutMs, outStatus);
        return 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return static_cast<unsigned>(GetExceptionCode());
    }
#else
    capture2DAnd3DBody(camera, outFrame, timeoutMs, outStatus);
    return 0;
#endif
}

}  // namespace sdk_seh
}  // namespace mech_eye
}  // namespace scan_tracking
