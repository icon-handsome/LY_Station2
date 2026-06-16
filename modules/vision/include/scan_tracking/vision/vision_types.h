#pragma once

// 视觉域共享类型：Mech-Eye + 海康 CXP 双目组合采集。

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking {
namespace vision {

enum class VisionErrorCode {
    Success = 0,
    NotStarted = 1,
    Busy = 2,
    InvalidConfig = 3,
    CaptureRejected = 4,
    NotImplemented = 5,
    DeviceNotFound = 6,
    DeviceOpenFailed = 7,
    SdkInitFailed = 8,
    UnknownError = 9,
};

enum class VisionPipelineState {
    Idle = 0,
    Ready = 1,
    Capturing = 2,
    Error = 3,
    Stopped = 4,
};

struct PoseMatrix4x4 {
    std::array<float, 16> values = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
    quint64 frameId = 0;
    qint64 timestampMs = 0;
    QString sourceCameraKey;
    bool valid = false;

    bool isValid() const { return valid; }
};

struct HikMonoFrame {
    std::shared_ptr<std::vector<std::uint8_t>> pixels;
    int width = 0;
    int height = 0;
    int stride = 0;
    quint64 frameId = 0;
    qint64 timestampMs = 0;
    QString sourceCameraKey;
    QString pixelFormat = QStringLiteral("Mono8");

    bool isValid() const
    {
        return pixels != nullptr && !pixels->empty() && width > 0 && height > 0;
    }
};

struct HikPoseCaptureRequest {
    quint64 requestId = 0;
    QString cameraKey;
    QString logicalName;
    int timeoutMs = 1000;
};

struct HikPoseCaptureResult {
    quint64 requestId = 0;
    QString cameraKey;
    QString logicalName;
    VisionErrorCode errorCode = VisionErrorCode::Success;
    QString errorMessage;
    HikMonoFrame frame;
    PoseMatrix4x4 poseMatrix;
    qint64 elapsedMs = 0;

    bool success() const
    {
        return errorCode == VisionErrorCode::Success && frame.isValid();
    }

    bool hasPose() const
    {
        return poseMatrix.isValid();
    }
};

struct HikCameraParams {
    float exposureTimeUs = 0.0f;
    float exposureTimeMinUs = 0.0f;
    float exposureTimeMaxUs = 0.0f;
    bool autoExposureEnabled = false;
    float gainDb = 0.0f;
    float gainMinDb = 0.0f;
    float gainMaxDb = 0.0f;
    bool autoGainEnabled = false;
    float frameRateFps = 0.0f;
    bool frameRateEnabled = false;
    quint32 triggerMode = 0;
    qint64 width = 0;
    qint64 height = 0;
    quint32 pixelFormat = 0;
    QString pixelFormatStr;
    bool valid = false;
    QString errorMessage;
};

enum class CaptureType {
    SurfaceDefect = 0,
    WeldDefect = 1,
    NumberRecognition = 2,
};

struct MultiCameraCaptureRequest {
    quint64 requestId = 0;
    quint32 taskId = 0;
    int segmentIndex = 0;
    bool needMechEye2D = false;
    scan_tracking::mech_eye::CaptureMode mechCaptureMode =
        scan_tracking::mech_eye::CaptureMode::Capture3DOnly;
    QString mechEyeCameraKey;
    int mechEyeTimeoutMs = 5000;
    QString hikCameraAKey;
    QString hikCameraBKey;
    int hikTimeoutMs = 1000;
};

struct MultiCameraCaptureBundle {
    MultiCameraCaptureRequest request;
    scan_tracking::mech_eye::CaptureResult mechEyeResult;
    HikPoseCaptureResult hikCameraAResult;
    HikPoseCaptureResult hikCameraBResult;

    bool success() const
    {
        return mechEyeResult.success() &&
               hikCameraAResult.success() &&
               hikCameraBResult.success();
    }

    QString summary() const
    {
        const auto flag = [](bool ok) {
            return ok ? QStringLiteral("成功") : QStringLiteral("失败");
        };
        return QStringLiteral(
                   "组合采集 requestId=%1 taskId=%2 段号=%3 梅卡=%4 海康A=%5 海康B=%6")
            .arg(request.requestId)
            .arg(request.taskId)
            .arg(request.segmentIndex)
            .arg(flag(mechEyeResult.success()))
            .arg(flag(hikCameraAResult.success()))
            .arg(flag(hikCameraBResult.success()));
    }
};

}  // namespace vision
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::vision::VisionErrorCode)
Q_DECLARE_METATYPE(scan_tracking::vision::VisionPipelineState)
Q_DECLARE_METATYPE(scan_tracking::vision::PoseMatrix4x4)
Q_DECLARE_METATYPE(scan_tracking::vision::HikMonoFrame)
Q_DECLARE_METATYPE(scan_tracking::vision::HikPoseCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::HikPoseCaptureResult)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureBundle)
Q_DECLARE_METATYPE(scan_tracking::vision::CaptureType)
