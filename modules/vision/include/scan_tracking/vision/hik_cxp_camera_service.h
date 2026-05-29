#pragma once

#include <QtCore/QObject>

#include <atomic>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

/// 海康 CoaXPress（PCIe 采集卡）相机服务，按序列号匹配设备。
class HikCxpCameraService : public QObject {
    Q_OBJECT

public:
    explicit HikCxpCameraService(const QString& roleName, QObject* parent = nullptr);
    ~HikCxpCameraService() override;

    void start(
        const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig,
        int defaultCaptureTimeoutMs,
        float exposureTimeUs = 50000.0f,
        float gain = 0.0f);
    void stop();

    bool isStarted() const;
    bool isConnected() const;
    QString roleName() const;

    quint64 requestMonoCapture(const QString& preferredCameraKey = {}, int timeoutMs = 0);

    /// 与 HikCameraService 对齐，供 VisionPipeline 组合采集调用。
    quint64 requestPoseCapture(const QString& preferredCameraKey = {}, int timeoutMs = 0)
    {
        return requestMonoCapture(preferredCameraKey, timeoutMs);
    }

signals:
    void monoCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void poseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void stateChanged(QString roleName, QString stateText, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private:
    class Impl;

    static void registerMetaTypes();
    QString resolveCameraKey(const QString& preferredCameraKey) const;
    bool ensureConnected(const QString& preferredCameraKey, QString* errorMessage);
    bool captureMonoFrame(
        int timeoutMs,
        const QString& cameraKey,
        QString* errorMessage,
        HikMonoFrame* outFrame = nullptr);
    bool openMatchedDevice(const QString& preferredCameraKey, QString* errorMessage);
    void closeDevice();
    void startAsyncConnect();

    QString m_roleName;
    scan_tracking::common::VisionCameraEndpointConfig m_endpointConfig;
    int m_defaultCaptureTimeoutMs = 5000;
    float m_exposureTimeUs = 50000.0f;
    float m_gain = 0.0f;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    std::atomic_bool m_connectInFlight{false};
    std::atomic_bool m_captureInFlight{false};
    Impl* m_impl = nullptr;
};

}  // namespace vision
}  // namespace scan_tracking
