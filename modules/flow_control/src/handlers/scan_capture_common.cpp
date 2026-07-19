#include "scan_tracking/flow_control/handlers/scan_capture_common.h"

#include <cstring>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

void executeConfiguredScanCapture(TaskHandlerContext& ctx, const char* triggerLabel)
{
    const int localIndex = ctx.activeTask.scanSegmentIndex;
    const quint32 taskId = ctx.activeTask.taskId;

    // PLC 不发 ResultReset/也可能不发 Inspection：上一路径齐套后再次下发段号 1，视为开下一路。
    ctx.host.maybeAdvancePathOnNewCycleStart(localIndex);

    auto* vision = ctx.host.visionPipelineService();
    if (vision == nullptr || !vision->isStarted()) {
        qWarning(LOG_FLOW).noquote()
            << QString::fromUtf8(triggerLabel) << QStringLiteral("：视觉流水线不可用。");
        ctx.host.completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    const bool isTelescopicScanTrigger =
        std::strcmp(triggerLabel, "Trig_TelescopicScan") == 0;
    const auto device = isTelescopicScanTrigger
                            ? common::ScanDeviceKind::Telescopic
                            : common::ScanDeviceKind::Arm;
    const bool useTelescopicGroup = isTelescopicScanTrigger;

    const auto* configMgr = common::ConfigManager::instance();
    if (configMgr != nullptr) {
        if (!configMgr->isValidDeviceLocalIndex(device, localIndex)) {
            const int expected = isTelescopicScanTrigger
                                     ? configMgr->enabledTelescopicPointCount()
                                     : configMgr->enabledArmPointCount();
            qWarning(LOG_FLOW).noquote()
                << QString::fromUtf8(triggerLabel)
                << QStringLiteral("：pathId=%1 %2 本地段号 %3 超出配额 1..%4。")
                       .arg(configMgr->activePathId())
                       .arg(common::ConfigManager::scanDeviceKindToString(device))
                       .arg(localIndex)
                       .arg(expected);
            ctx.host.completeScanSegmentCapture(5, 0, 0, protocol::AckState::Failed, false);
            return;
        }
    } else if (localIndex <= 0) {
        qWarning(LOG_FLOW).noquote()
            << QString::fromUtf8(triggerLabel)
            << QStringLiteral("：段号无效 %1。").arg(localIndex);
        ctx.host.completeScanSegmentCapture(5, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    const auto mechCaptureMode = mech_eye::CaptureMode::Capture2DAnd3D;

    const quint64 requestId = vision->requestCaptureBundle(
        localIndex, taskId, mechCaptureMode, useTelescopicGroup);
    if (requestId == 0) {
        qWarning(LOG_FLOW).noquote()
            << QString::fromUtf8(triggerLabel) << QStringLiteral("：发起组合采集失败。");
        ctx.host.completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
        return;
    }

    ctx.activeTask.captureRequestId = requestId;
    ctx.host.setTaskProgress(20);
    ctx.host.publishIpcStatus();
    ctx.host.notifyScanStarted(localIndex, taskId);
    const int pathId = configMgr != nullptr ? configMgr->activePathId() : 0;
    const QString purpose =
        configMgr != nullptr ? configMgr->pointPurpose(localIndex) : QString();
    qInfo(LOG_FLOW).noquote()
        << QString::fromUtf8(triggerLabel) << QStringLiteral("：已发起组合采集")
        << QStringLiteral(" pathId=") << pathId
        << QStringLiteral(" 本地段号=") << localIndex
        << QStringLiteral(" requestId=") << requestId
        << QStringLiteral(" mechMode=2D+3D")
        << QStringLiteral(" device=")
        << common::ConfigManager::scanDeviceKindToString(device)
        << (purpose.isEmpty() ? QString() : QStringLiteral(" purpose=") + purpose);
}

}  // namespace scan_tracking::flow_control
