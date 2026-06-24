#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

void StateMachine::notifyScanStarted(int segmentIndex, quint32 taskId)
{
    emit scanStarted(segmentIndex, taskId);
}

void StateMachine::onBundleCaptureFinished(vision::MultiCameraCaptureBundle bundle)
{
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment ||
        m_activeTask.completionAnnounced) {
        return;
    }
    if (bundle.request.requestId != m_activeTask.captureRequestId) {
        return;
    }

    int imageCount = 0;
    int cloudFrameCount = 0;
    countBundleFrames(bundle, &imageCount, &cloudFrameCount);

    if (bundle.success()) {
        m_scanSegmentCache.storeSegment(
            bundle.request.segmentIndex,
            bundle.request.taskId,
            bundle);
        QString persistError;
        if (!m_scanSegmentCache.persistSegment(bundle.request.segmentIndex, &persistError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_ScanSegment：段落盘失败（采集仍成功）")
                << persistError;
        }

        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("Trig_ScanSegment：采集成功") << bundle.summary()
            << QStringLiteral(" imageCount=") << imageCount
            << QStringLiteral(" cloudFrameCount=") << cloudFrameCount
            << QStringLiteral(" runRoot=") << m_scanSegmentCache.runCaptureRoot();
        completeScanSegmentCapture(1, imageCount, cloudFrameCount, protocol::AckState::Completed, true);
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ScanSegment：采集失败") << bundle.summary();
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::onVisionPipelineFatalError(vision::VisionErrorCode code, QString message)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[VisionPipeline] 致命错误：")
        << static_cast<int>(code)
        << message;

    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment ||
        m_activeTask.completionAnnounced) {
        return;
    }

    setAlarm(3, 723, message);
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::completeScanSegmentCapture(
    quint16 resultCode,
    int imageCount,
    int cloudFrameCount,
    protocol::AckState finalAckState,
    bool dataValid)
{
    const int segmentIndex = m_activeTask.scanSegmentIndex;
    writeScanSegmentResult(segmentIndex, imageCount, cloudFrameCount);
    completeActiveTask(resultCode, finalAckState, dataValid);
    emit scanFinished(segmentIndex, resultCode, imageCount, cloudFrameCount);
}

void StateMachine::onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message)
{
    Q_UNUSED(code);
    qCritical(LOG_FLOW) << "[MechEye] 致命错误:" << message;
    emit protocolEvent(QStringLiteral("Mech-Eye: %1").arg(message));

    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment ||
        m_activeTask.completionAnnounced) {
        return;
    }

    setAlarm(3, 723, message);
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::resetScanSegmentCache()
{
    m_scanSegmentCache.reset();
    qInfo(LOG_FLOW).noquote() << QStringLiteral("扫描段缓存已清空。");
}

}  // namespace scan_tracking::flow_control
