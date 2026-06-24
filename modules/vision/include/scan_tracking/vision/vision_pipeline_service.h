#pragma once

// 多相机视觉流水线：Mech-Eye + 海康 CXP 双目，或临时 Mech-Eye + 海康智能 C。

#include <QtCore/QObject>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCxpCameraService;
class HikCameraCController;

class VisionPipelineService : public QObject {
    Q_OBJECT

public:
    VisionPipelineService(
        scan_tracking::mech_eye::MechEyeService* mechEyeService,
        HikCxpCameraService* hikCameraAService,
        HikCxpCameraService* hikCameraBService,
        HikCameraCController* hikCameraCController = nullptr,
        QObject* parent = nullptr);
    ~VisionPipelineService() override = default;

    void start(const scan_tracking::common::VisionConfig& config);
    void stop();

    bool isStarted() const { return m_started; }
    VisionPipelineState state() const { return m_state; }

    quint64 requestCaptureBundle(
        int segmentIndex,
        quint32 taskId,
        scan_tracking::mech_eye::CaptureMode mechCaptureMode =
            scan_tracking::mech_eye::CaptureMode::Capture3DOnly);

signals:
    void bundleCaptureFinished(scan_tracking::vision::MultiCameraCaptureBundle bundle);
    void stateChanged(scan_tracking::vision::VisionPipelineState state, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private slots:
    void onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result);
    void onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void onHikCameraCImageReceived(scan_tracking::vision::CaptureType type, QString filePath, qint64 fileSize);
    void onHikCameraCCaptureCompleted(scan_tracking::vision::CaptureType type, QByteArray imageData);

private:
    struct PendingCaptureContext {
        bool active = false;
        bool mechDone = false;
        bool hikADone = false;
        bool hikBDone = false;
        bool hikCDone = false;
        bool useHikCameraC = false;
        quint64 mechRequestId = 0;
        quint64 hikARequestId = 0;
        quint64 hikBRequestId = 0;
        scan_tracking::vision::MultiCameraCaptureBundle bundle;
    };

    static void registerMetaTypes();
    void setState(VisionPipelineState state, const QString& description);
    void startPendingHikCapture();
    void startPendingHikCameraCCapture();
    void completeHikCameraCCapture(const QString& imagePath);
    void onHikCameraCCaptureTimeout();
    void finishBundleIfReady();

    scan_tracking::mech_eye::MechEyeService* m_mechEyeService = nullptr;
    HikCxpCameraService* m_hikCameraAService = nullptr;
    HikCxpCameraService* m_hikCameraBService = nullptr;
    HikCameraCController* m_hikCameraCController = nullptr;
    scan_tracking::common::VisionConfig m_config;
    PendingCaptureContext m_pending;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    VisionPipelineState m_state = VisionPipelineState::Idle;
};

}  // namespace vision
}  // namespace scan_tracking
