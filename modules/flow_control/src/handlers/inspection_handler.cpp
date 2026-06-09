#include "scan_tracking/flow_control/handlers/inspection_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking {
namespace flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* InspectionHandler::triggerName() const { return "Trig_Inspection"; }
int InspectionHandler::trigOffset() const { return 24; }
void InspectionHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeInspectionTask(); }

namespace {
constexpr quint16 kInspectionResOk = 1;
constexpr quint16 kInspectionResTimeoutNg = 6;
quint16 inspectionResForPlcHandshake(quint16 actualResultCode)
{
    if (actualResultCode == kInspectionResTimeoutNg) {
        return kInspectionResTimeoutNg;
    }
    return kInspectionResOk;
}
}

void StateMachine::executeInspectionTask()
{
    if (m_activeTask.inspectionPathId <= 0) {
        m_activeTask.inspectionPathId = resolvePathIdForInspection();
    }
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection 开始，检测路径ID=") << m_activeTask.inspectionPathId
        << multiPathCacheStatusText();

    // 检查跟踪服务是否可用
    if (m_tracking == nullptr) {
        qWarning(LOG_FLOW) << QStringLiteral("Trig_Inspection：Tracking 服务不可用。");
        // 写入默认的检测失败结果
        writeInspectionResult({2, 1u << 4, 0, 0});

        // 演示：tracking 不可用时也向显控推送失败结果（与蓝友出口字段一致）
        if (m_inspectionResultPublisher) {
            tracking::InspectionResult failure;
            failure.resultCode = 2;
            failure.ngReasonWord0 = (1u << 4);
            failure.message = QStringLiteral("综合检测失败：Tracking 服务不可用。");
            m_inspectionResultPublisher(failure);
        }

        // TODO(field-commissioning): 真实失败码为 7；PLC 侧临时强制 Res_Inspection=1(OK)
        const quint16 plcRes = inspectionResForPlcHandshake(7);
        completeActiveTask(plcRes, protocol::AckState::Completed, plcRes == kInspectionResOk);
        return;
    }

    QString loadError;
    ensurePoseStitchRunRootDirectory();
    persistLastPoseStitchArtifactToDisk();

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const auto inspectionType = configManager != nullptr
        ? configManager->inspectionTypeForPath(m_activeTask.inspectionPathId)
        : scan_tracking::common::InspectionType::Bevel;

    tracking::InspectionResult trackingResult;
    int segmentCount = 0;

    if (inspectionType == scan_tracking::common::InspectionType::Thickness) {
        scan_tracking::mech_eye::PointCloudFrame innerCloud;
        scan_tracking::mech_eye::PointCloudFrame outerCloud;
        int innerPointCount = 0;
        int outerPointCount = 0;
        int innerSegmentIndex = 0;
        int outerSegmentIndex = 0;
        if (!loadThicknessPointCloudsForInspection(
                &innerCloud,
                &outerCloud,
                &innerPointCount,
                &outerPointCount,
                &innerSegmentIndex,
                &outerSegmentIndex,
                &loadError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 加载厚度点云失败：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError.isEmpty()
                    ? QStringLiteral("综合检测失败：无法加载厚度 inner/outer 点云。")
                    : loadError;
                m_inspectionResultPublisher(failure);
            }
            const quint16 plcRes = inspectionResForPlcHandshake(7);
            completeActiveTask(plcRes, protocol::AckState::Completed, plcRes == kInspectionResOk);
            clearActiveTask();
            m_ipcState = protocol::IpcState::Ready;
            m_currentStage = protocol::Stage::Idle;
            m_progress = 0;
            setState(AppState::Ready);
            publishIpcStatus();
            return;
        }

        segmentCount = 2;
        trackingResult = m_tracking->inspectThicknessPointClouds(
            innerCloud,
            outerCloud,
            innerPointCount,
            outerPointCount,
            m_activeTask.inspectionPathId);
    } else {
        scan_tracking::mech_eye::PointCloudFrame mergedCloud;
        int totalPointCount = 0;
        if (!loadMergedPointCloudForInspection(
                &mergedCloud, &totalPointCount, &segmentCount, &loadError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 加载内存点云失败：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError.isEmpty()
                    ? QStringLiteral("综合检测失败：无法加载必需分段点云。")
                    : loadError;
                m_inspectionResultPublisher(failure);
            }
            const quint16 plcRes = inspectionResForPlcHandshake(7);
            completeActiveTask(plcRes, protocol::AckState::Completed, plcRes == kInspectionResOk);
            clearActiveTask();
            m_ipcState = protocol::IpcState::Ready;
            m_currentStage = protocol::Stage::Idle;
            m_progress = 0;
            setState(AppState::Ready);
            publishIpcStatus();
            return;
        }

        trackingResult = m_tracking->inspectPointCloud(
            mergedCloud, totalPointCount, m_activeTask.inspectionPathId);
    }

    InspectionSummary summary;
    summary.resultCode = trackingResult.resultCode;
    summary.ngReasonWord0 = trackingResult.ngReasonWord0;
    summary.ngReasonWord1 = trackingResult.ngReasonWord1;
    summary.measureItemCount = trackingResult.measureItemCount;

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection 完成")
        << QStringLiteral(" 参与段数=") << segmentCount
        << QStringLiteral(" 总点数=") << trackingResult.sourcePointCount
        << QStringLiteral(" angleDeg=") << trackingResult.measurement.headAngleTol
        << QStringLiteral(" lengthMm=") << trackingResult.measurement.bluntHeightTol
        << QStringLiteral(" thicknessMm=") << trackingResult.measurement.thicknessMm
        << QStringLiteral(" 说明=") << trackingResult.message;

    // 将检测结果写入 PLC 寄存器（NG 字等仍写真实算法结果，供联调日志/后续恢复）
    writeInspectionResult(summary);

    // TODO(field-commissioning): Res_Inspection 仅超时(6)报 NG，其它一律 OK(1)
    const quint16 actualResultCode = summary.resultCode;
    const quint16 plcRes = inspectionResForPlcHandshake(actualResultCode);
    if (plcRes != actualResultCode) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("TODO(field-commissioning): Res_Inspection 临时强制 OK")
            << QStringLiteral(" actualResultCode=") << actualResultCode
            << QStringLiteral(" plcRes=") << plcRes
            << QStringLiteral(" message=") << trackingResult.message;
    }
    completeActiveTask(plcRes, protocol::AckState::Completed, plcRes == kInspectionResOk);
    emit inspectionFinished(
        summary.resultCode, summary.ngReasonWord0, summary.ngReasonWord1,
        summary.measureItemCount, trackingResult.measurement, trackingResult.message);
    // 每条路径检测后保留各路径点云，供后续路径继续扫描；整轮结束由 Trig_ResultReset 清空
}


}  // namespace flow_control
}  // namespace scan_tracking
