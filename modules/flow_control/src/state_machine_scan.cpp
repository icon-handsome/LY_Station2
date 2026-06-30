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
    if (m_activeTask.definition == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：无活动任务 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (!isScanCaptureStage(m_activeTask.definition->stage)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：当前阶段非扫描采集 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (m_activeTask.completionAnnounced) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：任务已收尾 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (bundle.request.requestId != m_activeTask.captureRequestId) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：requestId 不匹配 active=")
            << m_activeTask.captureRequestId
            << QStringLiteral(" bundle=") << bundle.request.requestId;
        return;
    }

    const QString triggerLabel = protocol::triggerName(*m_activeTask.definition);

    int imageCount = 0;
    int cloudFrameCount = 0;
    countBundleFrames(bundle, &imageCount, &cloudFrameCount);

    if (bundle.success()) {
        m_scanSegmentCache.storeSegment(
            bundle.request.segmentIndex,
            bundle.request.taskId,
            bundle);

        qInfo(LOG_FLOW).noquote()
            << triggerLabel << QStringLiteral("：采集成功") << bundle.summary()
            << QStringLiteral(" imageCount=") << imageCount
            << QStringLiteral(" cloudFrameCount=") << cloudFrameCount;
        completeScanSegmentCapture(1, imageCount, cloudFrameCount, protocol::AckState::Completed, true);
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << triggerLabel << QStringLiteral("：采集失败") << bundle.summary();
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::onVisionPipelineFatalError(vision::VisionErrorCode code, QString message)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[VisionPipeline] 致命错误：")
        << static_cast<int>(code)
        << message;

    if (m_activeTask.definition == nullptr ||
        !isScanCaptureStage(m_activeTask.definition->stage) ||
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
    if (m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::TelescopicScan) {
        writeTelescopicScanResult(segmentIndex, imageCount, cloudFrameCount);
    } else {
        writeScanSegmentResult(segmentIndex, imageCount, cloudFrameCount);
    }
    completeActiveTask(resultCode, finalAckState, dataValid);
    emit scanFinished(segmentIndex, resultCode, imageCount, cloudFrameCount);
}

void StateMachine::onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message)
{
    Q_UNUSED(code);
    qCritical(LOG_FLOW) << "[MechEye] 致命错误:" << message;
    emit protocolEvent(QStringLiteral("Mech-Eye: %1").arg(message));

    if (m_activeTask.definition == nullptr ||
        !isScanCaptureStage(m_activeTask.definition->stage) ||
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
