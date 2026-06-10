#pragma once

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

namespace scan_tracking {
namespace orbbec_gemini {

enum class OrbbecGeminiRuntimeState {
    Idle = 0,
    Enumerating,
    Opening,
    Ready,
    Capturing,
    Failed,
    Stopped,
};

enum class OrbbecCaptureErrorCode {
    Success = 0,
    NotStarted = 1,
    NotReady = 2,
    Busy = 3,
    Timeout = 4,
    CaptureFailed = 5,
    SaveFailed = 6,
    InvalidRequest = 7,
    UnknownError = 8,
};

struct OrbbecGeminiOpenConfig {
    QString serial;
    int deviceIndex = 0;
    int depthWidth = 640;
    int depthHeight = 480;
    int fps = 15;
    int captureTimeoutMs = 5000;
    int warmupFrameCount = 5;
    bool saveCaptureToDisk = true;
    QString captureCacheRoot;
    bool enableColorStream = false;
};

struct OrbbecGeminiDeviceSummary {
    int index = -1;
    QString name;
    QString serialNumber;
    QString firmwareVersion;
    QString connectionType;
    quint16 pid = 0;
    quint16 vid = 0;
    QString uid;
};

struct OrbbecCaptureRequest {
    quint64 requestId = 0;
    int timeoutMs = 5000;
    bool saveToDisk = true;
};

struct OrbbecCaptureResult {
    quint64 requestId = 0;
    OrbbecCaptureErrorCode errorCode = OrbbecCaptureErrorCode::Success;
    QString errorMessage;
    OrbbecGeminiDeviceSummary deviceInfo;
    int depthWidth = 0;
    int depthHeight = 0;
    float depthValueScale = 0.0f;
    int validDepthPixelCount = 0;
    int pointCloudPointCount = 0;
    qint64 captureDurationMs = 0;
    QString depthRawPngPath;
    QString depthPreviewPngPath;
    QString pointCloudPlyPath;
};

}  // namespace orbbec_gemini
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecCaptureErrorCode)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecCaptureResult)
Q_DECLARE_METATYPE(QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary>)
