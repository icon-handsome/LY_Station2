#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station2_inspection.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/vision_types.h"

#include <cstring>

namespace scan_tracking::flow_control {

int StateMachine::resolveExpectedScanSegmentCount() const
{
    const auto* configMgr = common::ConfigManager::instance();
    if (configMgr == nullptr) {
        return 0;
    }
    const int fromScanPaths = configMgr->enabledScanPointCount();
    if (fromScanPaths > 0) {
        return fromScanPaths;
    }
    return configMgr->trackingConfig().scanSegmentTotal;
}

InspectionResult StateMachine::evaluateCachedInspection(quint32 taskId) const
{
    const quint32 effectiveTaskId = taskId != 0 ? taskId : m_scanSegmentCache.runTaskId();
    InspectionQuota quota;
    if (const auto* configMgr = common::ConfigManager::instance()) {
        quota.expectedArmCount = configMgr->enabledArmPointCount();
        quota.expectedTelescopicCount = configMgr->enabledTelescopicPointCount();
        quota.pathId = configMgr->activePathId();
        quota.pathName = configMgr->activePathName();
        quota.algorithm = configMgr->activePathAlgorithm();
        if (quota.total() <= 0) {
            quota.expectedArmCount = resolveExpectedScanSegmentCount();
        }
    }
    return evaluateStation2Inspection(m_scanSegmentCache, effectiveTaskId, quota);
}

InspectionResult StateMachine::evaluateInspectionForActiveTask() const
{
    return evaluateCachedInspection(m_activeTask.taskId);
}

void StateMachine::finishInspection(const InspectionResult& result)
{
    m_codeReadPending = false;
    m_codeReadCameraIp.clear();

    InspectionSummary summary;
    summary.resultCode = result.resultCode;
    summary.ngReasonWord0 = result.ngReasonWord0;
    summary.ngReasonWord1 = result.ngReasonWord1;
    summary.measureItemCount = result.measureItemCount;
    writeInspectionResult(summary);

    if (!result.measurement.codeValue.isEmpty() && isModbusConnected()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            result.measurement.codeValue);
    }

    const bool dataValid = result.resultCode == 1 || result.resultCode == 2;
    const protocol::AckState ackState = dataValid ? protocol::AckState::Completed
                                                  : protocol::AckState::Failed;
    completeActiveTask(result.resultCode, ackState, dataValid);

    emit inspectionFinished(
        result.resultCode,
        result.ngReasonWord0,
        result.ngReasonWord1,
        result.measureItemCount,
        result.measurement,
        result.message);

    emit inspectionResultReady(result);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection：已完成 Res=") << result.resultCode
        << QStringLiteral(" pathId=") << result.pathId
        << QStringLiteral(" pathName=") << result.pathName
        << QStringLiteral(" algorithm=") << result.algorithm
        << QStringLiteral(" qualityCode=") << result.measurement.qualityCode
        << QStringLiteral(" segments=") << result.sourcePointCount
        << QStringLiteral(" message=") << result.message;
}

bool StateMachine::isActiveCodeReadTrigger() const
{
    return m_activeTask.definition != nullptr &&
           std::strcmp(m_activeTask.definition->name, "Trig_CodeRead") == 0;
}

void StateMachine::startCodeReadCapture()
{
    auto* hik = m_hikCameraCController;
    if (hik == nullptr || !hik->isStarted()) {
        finishCodeRead(2, QString(), QStringLiteral("海康智能相机 C 未启动，无法编号识别。"));
        return;
    }

    QString cameraIp;
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        cameraIp = cfgMgr->visionConfig().armGroup.hikCameraC.ipAddress.trimmed();
    }
    if (cameraIp.isEmpty()) {
        cameraIp = hik->configuredCameraIps().value(0).trimmed();
    }
    if (cameraIp.isEmpty()) {
        finishCodeRead(2, QString(), QStringLiteral("未配置机械臂侧海康 C IP。"));
        return;
    }
    if (!hik->isCameraConnected(cameraIp)) {
        finishCodeRead(
            2,
            QString(),
            QStringLiteral("机械臂侧海康 C 未连接：%1").arg(cameraIp));
        return;
    }

    m_codeReadPending = true;
    m_codeReadCameraIp = cameraIp;
    setTaskProgress(30);
    publishIpcStatus();

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("编号识别：向海康 C 发送 NumberRecognition")
        << QStringLiteral(" ip=") << cameraIp
        << QStringLiteral(" pathId=")
        << (common::ConfigManager::instance()
                ? common::ConfigManager::instance()->activePathId()
                : 0)
        << QStringLiteral(" trigger=")
        << (m_activeTask.definition != nullptr
                ? protocol::triggerName(*m_activeTask.definition)
                : QStringLiteral("-"));

    if (!hik->requestCapture(vision::CaptureType::NumberRecognition, cameraIp)) {
        finishCodeRead(
            2,
            QString(),
            QStringLiteral("向海康 C 发送编号拍照失败：%1").arg(cameraIp));
    }
}

void StateMachine::onHikOcrTextReceived(QString cameraIp, QString text)
{
    if (!m_codeReadPending) {
        return;
    }
    if (!m_codeReadCameraIp.isEmpty() && cameraIp.trimmed() != m_codeReadCameraIp) {
        return;
    }

    const QString code = text.trimmed();
    if (code.isEmpty()) {
        finishCodeRead(2, QString(), QStringLiteral("OCR 回包为空。"));
        return;
    }

    finishCodeRead(1, code, QStringLiteral("编号识别成功：%1").arg(code));
}

void StateMachine::finishCodeRead(quint16 resultCode, const QString& codeValue, const QString& message)
{
    if (m_activeTask.definition == nullptr || m_activeTask.completionAnnounced) {
        m_codeReadPending = false;
        m_codeReadCameraIp.clear();
        return;
    }

    m_codeReadPending = false;
    m_codeReadCameraIp.clear();

    const QString effectiveMessage = message.isEmpty()
        ? (resultCode == 1
               ? QStringLiteral("编号识别成功")
               : QStringLiteral("编号识别失败"))
        : message;

    if (isModbusConnected()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            resultCode == 1 ? codeValue : QString());
    }

    // Trig_Inspection + algorithm=code_read：走 Inspection 结果通道，便于 HMI 统一展示
    if (!isActiveCodeReadTrigger()) {
        InspectionResult result;
        if (const auto* cfgMgr = common::ConfigManager::instance()) {
            result.pathId = cfgMgr->activePathId();
            result.pathName = cfgMgr->activePathName();
            result.algorithm = cfgMgr->activePathAlgorithm();
        }
        result.resultCode = resultCode;
        result.measureItemCount = resultCode == 1 ? 1 : 0;
        result.measurement.qualityCode = resultCode == 1 ? 1 : 2;
        result.measurement.codeValue = codeValue;
        result.message = effectiveMessage;
        finishInspection(result);
        notifyCodeReadFinished(resultCode, codeValue);
        return;
    }

    const bool dataValid = resultCode == 1;
    const protocol::AckState ackState =
        dataValid ? protocol::AckState::Completed : protocol::AckState::Failed;
    completeActiveTask(resultCode, ackState, dataValid);
    notifyCodeReadFinished(resultCode, codeValue);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_CodeRead：已完成 Res=") << resultCode
        << QStringLiteral(" code=") << codeValue
        << QStringLiteral(" message=") << effectiveMessage;
}

}  // namespace scan_tracking::flow_control
