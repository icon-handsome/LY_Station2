#include "scan_tracking/flow_control/handlers/scan_capture_common.h"

#include <cstring>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

namespace {

bool isInternalSegmentKind(const QString& kind)
{
    return kind.startsWith(QLatin1String("internal"));
}

}  // namespace

void executeConfiguredScanCapture(
    TaskHandlerContext& ctx,
    const char* triggerLabel,
    bool telescopicInternal)
{
    const int segmentIndex = ctx.activeTask.scanSegmentIndex;
    const quint32 taskId = ctx.activeTask.taskId;

    auto* vision = ctx.host.visionPipelineService();
    if (vision == nullptr || !vision->isStarted()) {
        qWarning(LOG_FLOW).noquote()
            << QString::fromUtf8(triggerLabel) << QStringLiteral("：视觉流水线不可用。");
        ctx.host.completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    const auto* configMgr = common::ConfigManager::instance();
    const common::ScanPointConfig* point = nullptr;
    QString segmentKind = QStringLiteral("external");
    if (configMgr != nullptr) {
        point = configMgr->findScanPointByIndex(segmentIndex);
        segmentKind = configMgr->segmentKindForPointIndex(segmentIndex);
    }

    if (point == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QString::fromUtf8(triggerLabel)
            << QStringLiteral("：段号 %1 不在 scan_paths 配置中。").arg(segmentIndex);
        ctx.host.completeScanSegmentCapture(5, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    const bool kindIsInternal = isInternalSegmentKind(segmentKind);
    const bool isTelescopicScanTrigger =
        std::strcmp(triggerLabel, "Trig_TelescopicScan") == 0;
    if (isTelescopicScanTrigger && !kindIsInternal) {
        qInfo(LOG_FLOW).noquote()
            << QString::fromUtf8(triggerLabel)
            << QStringLiteral("：段号 %1 为 external 路径，仍执行组合采集。").arg(segmentIndex);
    }
    if (!isTelescopicScanTrigger && kindIsInternal) {
        qWarning(LOG_FLOW).noquote()
            << QString::fromUtf8(triggerLabel)
            << QStringLiteral("：段号 %1 为伸缩杆内部路径，请使用 Trig_TelescopicScan。")
                   .arg(segmentIndex);
        ctx.host.completeScanSegmentCapture(5, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    const auto mechCaptureMode = mech_eye::CaptureMode::Capture2DAnd3D;

    const quint64 requestId = vision->requestCaptureBundle(
        segmentIndex, taskId, mechCaptureMode, telescopicInternal);
    if (requestId == 0) {
        qWarning(LOG_FLOW).noquote()
            << QString::fromUtf8(triggerLabel) << QStringLiteral("：发起组合采集失败。");
        ctx.host.completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    ctx.activeTask.captureRequestId = requestId;
    ctx.host.setTaskProgress(20);
    ctx.host.publishIpcStatus();
    ctx.host.notifyScanStarted(segmentIndex, taskId);
    qInfo(LOG_FLOW).noquote()
        << QString::fromUtf8(triggerLabel) << QStringLiteral("：已发起组合采集 段号=") << segmentIndex
        << QStringLiteral(" requestId=") << requestId
        << QStringLiteral(" mechMode=2D+3D")
        << (telescopicInternal ? QStringLiteral(" hikC=concurrent-trigger") : QString());
}

}  // namespace scan_tracking::flow_control
