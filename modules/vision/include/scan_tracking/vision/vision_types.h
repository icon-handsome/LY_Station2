#pragma once

// 视觉域共享类型：第二工位 Orbbec Gemini 分段采集。

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

#include "scan_tracking/orbbec_gemini/orbbec_gemini_types.h"

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

inline bool orbbecCapturePayloadReady(const orbbec_gemini::OrbbecCaptureResult& result)
{
    return result.errorCode == orbbec_gemini::OrbbecCaptureErrorCode::Success
           && result.pointCloudPointCount > 0
           && !result.pointCloudPlyPath.trimmed().isEmpty();
}

struct MultiCameraCaptureRequest {
    quint64 requestId = 0;
    quint32 taskId = 0;
    int segmentIndex = 0;
    /// 对应 scan_paths needRotation：预留彩色/2D 扩展，当前 Orbbec 主流程采集深度+点云。
    bool needColorCapture = false;
    int orbbecTimeoutMs = 5000;
    bool saveToDisk = true;
};

struct MultiCameraCaptureBundle {
    MultiCameraCaptureRequest request;
    orbbec_gemini::OrbbecCaptureResult orbbecResult;

    bool success() const
    {
        return orbbecCapturePayloadReady(orbbecResult);
    }

    QString summary() const
    {
        const auto flag = [](bool ok) {
            return ok ? QStringLiteral("成功") : QStringLiteral("失败");
        };
        return QStringLiteral(
                   "Orbbec 采集 requestId=%1 taskId=%2 段号=%3 Orbbec=%4 点数=%5")
            .arg(request.requestId)
            .arg(request.taskId)
            .arg(request.segmentIndex)
            .arg(flag(success()))
            .arg(orbbecResult.pointCloudPointCount);
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
