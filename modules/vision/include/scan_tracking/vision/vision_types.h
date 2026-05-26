#pragma once

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

// 海康相机可读写的关键参数快照
struct HikCameraParams {
    // ---- 曝光 ----
    float exposureTimeUs = 0.0f;      // 曝光时间（微秒），对应 GenICam 节点 "ExposureTime"
    float exposureTimeMinUs = 0.0f;
    float exposureTimeMaxUs = 0.0f;
    bool  autoExposureEnabled = false; // ExposureAuto != Off

    // ---- 增益 ----
    float gainDb = 0.0f;              // 模拟增益（dB），对应 "Gain"
    float gainMinDb = 0.0f;
    float gainMaxDb = 0.0f;
    bool  autoGainEnabled = false;    // GainAuto != Off

    // ---- 帧率 ----
    float frameRateFps = 0.0f;        // 当前帧率，对应 "AcquisitionFrameRate"
    bool  frameRateEnabled = false;   // AcquisitionFrameRateEnable

    // ---- 触发 ----
    quint32 triggerMode = 0;          // 0=Off(连续), 1=On(触发)，对应 "TriggerMode"

    // ---- 图像尺寸 ----
    qint64 width = 0;                 // 对应 "Width"
    qint64 height = 0;                // 对应 "Height"

    // ---- 像素格式 ----
    quint32 pixelFormat = 0;          // 枚举值，对应 "PixelFormat"
    QString pixelFormatStr;           // 枚举符号名，如 "Mono8"

    // ---- 读取状态 ----
    bool valid = false;               // true 表示本次读取成功
    QString errorMessage;             // 读取失败时的错误描述
};

// 海康智能相机拍照类型
enum class CaptureType {
    SurfaceDefect = 0,      // 表面普通缺陷识别
    WeldDefect = 1,         // 焊缝缺陷识别
    NumberRecognition = 2,  // 编号识别
};

struct LbPoseResult {
    bool invoked = false;
    bool success = false;
    QString message;
    int leftImageWidth = 0;
    int leftImageHeight = 0;
    int rightImageWidth = 0;
    int rightImageHeight = 0;
    int framePointCount = 0;
    PoseMatrix4x4 poseMatrix;
};

struct LbnPoseResult {
    bool invoked = false;
    bool success = false;
    QString message;
    int textureWidth = 0;
    int textureHeight = 0;
    int pointCloudWidth = 0;
    int pointCloudHeight = 0;
    int matchedPointCount = 0;
    PoseMatrix4x4 poseMatrix;
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
    LbPoseResult lbPoseResult;
    LbnPoseResult lbnPoseResult;

    bool success() const
    {
        return mechEyeResult.success() &&
               hikCameraAResult.success() &&
               hikCameraBResult.success() &&
               lbPoseResult.success;
    }

    QString summary() const
    {
        const auto flag = [](bool ok) {
            return ok ? QStringLiteral("成功") : QStringLiteral("失败");
        };
        const QString lbnFlag = lbnPoseResult.invoked
            ? (lbnPoseResult.success ? QStringLiteral("成功") : QStringLiteral("失败"))
            : QStringLiteral("跳过");
        return QStringLiteral(
                   "组合采集 requestId=%1 taskId=%2 段号=%3 梅卡=%4 海康A=%5 海康B=%6 LB=%7 LBN=%8")
            .arg(request.requestId)
            .arg(request.taskId)
            .arg(request.segmentIndex)
            .arg(flag(mechEyeResult.success()))
            .arg(flag(hikCameraAResult.success()))
            .arg(flag(hikCameraBResult.success()))
            .arg(flag(lbPoseResult.success))
            .arg(lbnFlag);
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
Q_DECLARE_METATYPE(scan_tracking::vision::LbPoseResult)
Q_DECLARE_METATYPE(scan_tracking::vision::LbnPoseResult)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureBundle)
Q_DECLARE_METATYPE(scan_tracking::vision::CaptureType)
