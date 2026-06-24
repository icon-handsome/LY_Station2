#include "scan_tracking/flow_control/handlers/scan_segment_handler.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

const char* ScanSegmentHandler::triggerName() const { return "Trig_ScanSegment"; }
int ScanSegmentHandler::trigOffset() const { return 23; }

void ScanSegmentHandler::execute(TaskHandlerContext& ctx)
{
    const int segmentIndex = ctx.activeTask.scanSegmentIndex;
    const quint32 taskId = ctx.activeTask.taskId;

    auto* vision = ctx.host.visionPipelineService();
    if (vision == nullptr || !vision->isStarted()) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("Trig_ScanSegment：视觉流水线不可用。");
        ctx.host.completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    const auto* configMgr = common::ConfigManager::instance();
    const common::ScanPointConfig* point = nullptr;
    if (configMgr != nullptr) {
        point = configMgr->findScanPointByIndex(segmentIndex);
    }
    if (point == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Trig_ScanSegment：段号 %1 不在 scan_paths 配置中。").arg(segmentIndex);
        ctx.host.completeScanSegmentCapture(5, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    const bool needColorCapture = point->needRotation;
    const quint64 requestId = vision->requestCaptureBundle(segmentIndex, taskId, needColorCapture);
    if (requestId == 0) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("Trig_ScanSegment：发起 Orbbec 采集失败。");
        ctx.host.completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    ctx.activeTask.captureRequestId = requestId;
    ctx.host.setTaskProgress(20);
    ctx.host.publishIpcStatus();
    ctx.host.notifyScanStarted(segmentIndex, taskId);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ScanSegment：已发起 Orbbec 采集 段号=") << segmentIndex
        << QStringLiteral(" requestId=") << requestId
        << QStringLiteral(" needColor=") << needColorCapture;
}

}  // namespace scan_tracking::flow_control
