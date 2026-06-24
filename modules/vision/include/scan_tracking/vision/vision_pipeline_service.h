#pragma once

// 第二工位视觉流水线：Orbbec Gemini 深度/点云分段采集。

#include <QtCore/QObject>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class VisionPipelineService : public QObject {
    Q_OBJECT

public:
    explicit VisionPipelineService(
        orbbec_gemini::OrbbecGeminiService* orbbecService,
        QObject* parent = nullptr);
    ~VisionPipelineService() override = default;

    void start(const scan_tracking::common::OrbbecGeminiConfig& config);
    void stop();

    bool isStarted() const { return m_started; }
    VisionPipelineState state() const { return m_state; }

    quint64 requestCaptureBundle(
        int segmentIndex,
        quint32 taskId,
        bool needColorCapture = false);

signals:
    void bundleCaptureFinished(scan_tracking::vision::MultiCameraCaptureBundle bundle);
    void stateChanged(scan_tracking::vision::VisionPipelineState state, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private slots:
    void onOrbbecCaptureFinished(scan_tracking::orbbec_gemini::OrbbecCaptureResult result);

private:
    struct PendingCaptureContext {
        bool active = false;
        quint64 orbbecRequestId = 0;
        scan_tracking::vision::MultiCameraCaptureBundle bundle;
    };

    static void registerMetaTypes();
    void setState(VisionPipelineState state, const QString& description);
    void finishCapture(const scan_tracking::orbbec_gemini::OrbbecCaptureResult& result);

    orbbec_gemini::OrbbecGeminiService* m_orbbecService = nullptr;
    scan_tracking::common::OrbbecGeminiConfig m_config;
    PendingCaptureContext m_pending;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    VisionPipelineState m_state = VisionPipelineState::Idle;
};

}  // namespace vision
}  // namespace scan_tracking
