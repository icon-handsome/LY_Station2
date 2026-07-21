#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station2_inspection.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/vision_types.h"

#include <QtCore/QPointer>
#include <QtCore/QTimer>

#include <cstring>

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

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

    // 检测成功后由 IPC 自清缓存并切下一条路径；不依赖 PLC Trig_ResultReset / pathId。
    if (result.resultCode == 1) {
        prepareNextScanPathAfterSuccess();
    }
}

bool StateMachine::isActiveCodeReadTrigger() const
{
    return m_activeTask.definition != nullptr &&
           std::strcmp(m_activeTask.definition->name, "Trig_CodeRead") == 0;
}

void StateMachine::prepareNextScanPathAfterSuccess()
{
    // PLC 不做 ResultReset：检测成功 / 开下一路时 IPC 自行清段缓存与扫描完成寄存器，再切路径。
    // 故意保留本次 Inspection 结果寄存器，供 PLC/HMI 读取。
    resetScanSegmentCache();
    if (isModbusConnected()) {
        clearScanSegmentDoneRegisters();
    }

    auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return;
    }

    const int fromPathId = cfgMgr->activePathId();
    const QString fromName = cfgMgr->activePathName();
    const int toPathId = cfgMgr->advanceToNextEnabledPath();
    if (toPathId <= 0) {
        qWarning(LOG_FLOW) << QStringLiteral("已清缓存，但无下一条启用路径可切换。");
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已自动清缓存并切换扫描路径：")
        << fromPathId << QStringLiteral("(") << fromName << QStringLiteral(")")
        << QStringLiteral(" -> ") << toPathId
        << QStringLiteral("(") << cfgMgr->activePathName() << QStringLiteral(")")
        << QStringLiteral(" algorithm=") << cfgMgr->activePathAlgorithm()
        << QStringLiteral(" 配额 arm=") << cfgMgr->enabledArmPointCount()
        << QStringLiteral(" telescopic=") << cfgMgr->enabledTelescopicPointCount()
        << QStringLiteral("；PLC 可直接按新路径从段号 1 继续采集。");
}

bool StateMachine::applyPlcScanPathId(
    const QVector<quint16>& commandBlock,
    const QVector<quint16>& previousCommandBlock,
    bool onlyOnChange)
{
    namespace regs = protocol::registers;
    if (regs::kScanPathId < 0 || regs::kScanPathId >= commandBlock.size()) {
        return false;
    }

    const int requested = static_cast<int>(
        regs::plcAnalogToUInt16(commandBlock.value(regs::kScanPathId), 0));
    // 0=未指定：沿用当前活跃路径（含 IPC 自动切路结果）。
    if (requested <= 0) {
        return false;
    }

    if (onlyOnChange && !previousCommandBlock.isEmpty() &&
        regs::kScanPathId < previousCommandBlock.size()) {
        const int previousRequested = static_cast<int>(
            regs::plcAnalogToUInt16(previousCommandBlock.value(regs::kScanPathId), 0));
        if (previousRequested == requested) {
            return false;
        }
    }

    auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return false;
    }

    const int fromPathId = cfgMgr->activePathId();
    if (fromPathId == requested) {
        return false;
    }

    if (!cfgMgr->setActivePathId(requested)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("PLC ScanPathId(40047)=") << requested
            << QStringLiteral(" 无效，保持当前 pathId=") << fromPathId;
        return false;
    }

    // pathId 变化：清段缓存与扫描 Done，避免旧路径段混入新路径检测。
    resetScanSegmentCache();
    if (isModbusConnected()) {
        clearScanSegmentDoneRegisters();
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已按 PLC ScanPathId(40047) 切换扫描路径：")
        << fromPathId << QStringLiteral(" -> ") << requested
        << QStringLiteral("(") << cfgMgr->activePathName() << QStringLiteral(")")
        << QStringLiteral(" algorithm=") << cfgMgr->activePathAlgorithm()
        << QStringLiteral(" 配额 arm=") << cfgMgr->enabledArmPointCount()
        << QStringLiteral(" telescopic=") << cfgMgr->enabledTelescopicPointCount();
    return true;
}

bool StateMachine::isActivePathQuotaComplete() const
{
    const auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return false;
    }
    return m_scanSegmentCache.meetsDeviceQuotas(
        cfgMgr->enabledArmPointCount(), cfgMgr->enabledTelescopicPointCount());
}

void StateMachine::maybeAdvancePathOnNewCycleStart(int localIndex)
{
    if (localIndex != 1) {
        return;
    }
    if (!isActivePathQuotaComplete()) {
        return;
    }

    const int fromPathId =
        common::ConfigManager::instance() != nullptr
            ? common::ConfigManager::instance()->activePathId()
            : 0;
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("检测到 pathId=") << fromPathId
        << QStringLiteral(" 已齐套且 PLC 再次下发段号 1：按新路径周期自动切换（无需 Inspection/ResultReset）。");
    prepareNextScanPathAfterSuccess();
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

    const bool scanStage =
        m_activeTask.definition != nullptr &&
        isScanCaptureStage(m_activeTask.definition->stage);
    QString algorithm;
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        algorithm = cfgMgr->activePathAlgorithm();
    }
    const bool inspectionCodeRead =
        m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::Inspection &&
        algorithm == QLatin1String("code_read");
    // 段扫/检测上的编号：发完 start 即放行。须晚于 Ack=Running 一拍，
    // 否则 PLC 可能抽不到 Ack=1→2 边沿，Trig 一直拉高导致整线卡死。
    const bool fireAndForget = scanStage || inspectionCodeRead;

    m_codeReadPending = !fireAndForget;
    m_codeReadSoftPending = fireAndForget;
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
                : QStringLiteral("-"))
        << (fireAndForget ? QStringLiteral(" mode=fire_and_forget")
                          : QStringLiteral(" mode=wait_ocr"));

    if (!hik->requestCapture(vision::CaptureType::NumberRecognition, cameraIp)) {
        m_codeReadSoftPending = false;
        finishCodeRead(
            2,
            QString(),
            QStringLiteral("向海康 C 发送编号拍照失败：%1").arg(cameraIp));
        return;
    }

    if (fireAndForget) {
        QPointer<StateMachine> self(this);
        QTimer::singleShot(150, this, [self]() {
            if (self == nullptr) {
                return;
            }
            // OCR 可能已先到并清掉 softPending；只要任务未收尾仍需写 Ack=Completed。
            if (self->m_activeTask.definition == nullptr ||
                self->m_activeTask.completionAnnounced) {
                return;
            }
            self->finishCodeRead(
                1,
                QString(),
                QStringLiteral("编号采集已触发（不等待 OCR；回包稍后写寄存器）"));
        });
    }
}

void StateMachine::onHikOcrTextReceived(QString cameraIp, QString text)
{
    const QString code = text.trimmed();
    if (code.isEmpty()) {
        if (m_codeReadPending) {
            finishCodeRead(2, QString(), QStringLiteral("OCR 回包为空。"));
        }
        return;
    }

    if (m_codeReadPending) {
        if (!m_codeReadCameraIp.isEmpty() && cameraIp.trimmed() != m_codeReadCameraIp) {
            return;
        }
        finishCodeRead(1, code, QStringLiteral("编号识别成功：%1").arg(code));
        return;
    }

    if (m_codeReadSoftPending) {
        applyLateCodeReadOcr(cameraIp, code);
    }
}

void StateMachine::applyLateCodeReadOcr(const QString& cameraIp, const QString& codeValue)
{
    if (!m_codeReadCameraIp.isEmpty() && cameraIp.trimmed() != m_codeReadCameraIp) {
        return;
    }

    m_codeReadSoftPending = false;
    m_codeReadCameraIp.clear();

    if (isModbusConnected()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            codeValue);
    }
    notifyCodeReadFinished(1, codeValue);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("编号 OCR 迟到回包已写入寄存器 code=") << codeValue
        << QStringLiteral(" ip=") << cameraIp;
}

void StateMachine::finishCodeRead(quint16 resultCode, const QString& codeValue, const QString& message)
{
    if (m_activeTask.definition == nullptr || m_activeTask.completionAnnounced) {
        m_codeReadPending = false;
        if (!m_codeReadSoftPending) {
            m_codeReadCameraIp.clear();
        }
        return;
    }

    const bool scanStage = isScanCaptureStage(m_activeTask.definition->stage);
    // fire-and-forget 成功收尾后保留 softPending，供迟到 OCR 写寄存器。
    const bool keepSoftPending = resultCode == 1 && m_codeReadSoftPending;

    m_codeReadPending = false;
    if (!keepSoftPending) {
        m_codeReadSoftPending = false;
        m_codeReadCameraIp.clear();
    }

    const QString effectiveMessage = message.isEmpty()
        ? (resultCode == 1
               ? QStringLiteral("编号识别成功")
               : QStringLiteral("编号识别失败"))
        : message;

    if (isModbusConnected() && !codeValue.isEmpty()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            resultCode == 1 ? codeValue : QString());
    }

    // Trig_ScanSegment / Trig_TelescopicScan 兼跑编号：回写段扫寄存器，并占位齐套缓存。
    if (scanStage) {
        const QString triggerLabel = protocol::triggerName(*m_activeTask.definition);
        const bool ok = resultCode == 1;
        bool shouldAdvancePath = false;
        if (ok) {
            const auto device =
                m_activeTask.definition->stage == protocol::Stage::TelescopicScan
                    ? common::ScanDeviceKind::Telescopic
                    : common::ScanDeviceKind::Arm;
            vision::MultiCameraCaptureBundle placeholder;
            placeholder.request.segmentIndex = m_activeTask.scanSegmentIndex;
            placeholder.request.taskId = m_activeTask.taskId;
            m_scanSegmentCache.storeSegment(
                device,
                m_activeTask.scanSegmentIndex,
                m_activeTask.taskId,
                placeholder);

            const auto* configMgr = common::ConfigManager::instance();
            const int armExpected =
                configMgr != nullptr ? configMgr->enabledArmPointCount() : 0;
            const int telescopicExpected =
                configMgr != nullptr ? configMgr->enabledTelescopicPointCount() : 0;
            shouldAdvancePath =
                m_scanSegmentCache.meetsDeviceQuotas(armExpected, telescopicExpected);
            if (shouldAdvancePath) {
                qInfo(LOG_FLOW).noquote()
                    << QStringLiteral("pathId=")
                    << (configMgr != nullptr ? configMgr->activePathId() : 0)
                    << QStringLiteral(" 编号段扫齐套（Done/Ack/Res 保持至 PLC 拉低 Trig）。");
            }
        }

        notifyCodeReadFinished(resultCode, codeValue);
        // PLC 常同时看 DoneIndex + ImageCount/CloudFrameCount；cloud=0 可能被当成无有效应答。
        completeScanSegmentCapture(
            ok ? 1 : 7,
            ok ? 1 : 0,
            ok ? 1 : 0,
            ok ? protocol::AckState::Completed : protocol::AckState::Failed,
            ok);

        qInfo(LOG_FLOW).noquote()
            << triggerLabel << QStringLiteral("（编号）：已完成 Res=") << (ok ? 1 : 7)
            << QStringLiteral(" code=") << codeValue
            << QStringLiteral(" message=") << effectiveMessage
            << (keepSoftPending ? QStringLiteral(" softOcr=1") : QString());

        // 勿在此处立刻 prepareNextScanPath：会清掉 DoneIndex，PLC 来不及读到扫描应答。
        // 等 Trig 拉低 finalize 后再切路径。
        if (shouldAdvancePath) {
            m_advancePathAfterTriggerRelease = true;
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("编号段扫齐套：保留 Done/Ack/Res 供 PLC 读取，"
                                  "待 Trig 释放后再自动切下一路。");
        }
        return;
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

    if (resultCode == 1) {
        prepareNextScanPathAfterSuccess();
    }
}

void StateMachine::startSelfCheckCapture()
{
    auto* vision = m_visionPipeline;
    if (vision == nullptr || !vision->isStarted()) {
        constexpr quint16 kFailCxpOrVision = 1u << 2;
        writeSelfCheckFailWords({kFailCxpOrVision});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCxpOrVision);
        return;
    }

    vision::VisionPipelineService::BundleCaptureOptions options;
    options.useMechEye = true;
    options.useHikCxp = true;
    options.useHikSmartC = false;

    const int segmentIndex =
        m_activeTask.scanSegmentIndex > 0 ? m_activeTask.scanSegmentIndex : 1;
    const quint64 requestId = vision->requestCaptureBundle(
        segmentIndex,
        m_activeTask.taskId,
        mech_eye::CaptureMode::Capture2DAnd3D,
        false,
        options);
    if (requestId == 0) {
        constexpr quint16 kFailCapture = 1u << 4;
        writeSelfCheckFailWords({kFailCapture});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCapture);
        return;
    }

    m_activeTask.captureRequestId = requestId;
    setTaskProgress(40);
    publishIpcStatus();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("自检：已发起组合采集 requestId=") << requestId
        << QStringLiteral(" 段号=") << segmentIndex
        << QStringLiteral(" channels=梅卡+CXP");
}

void StateMachine::finishSelfCheckCapture(const vision::MultiCameraCaptureBundle& bundle)
{
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::SelfCheck ||
        m_activeTask.completionAnnounced) {
        return;
    }
    if (bundle.request.requestId != m_activeTask.captureRequestId) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("自检采集完成忽略：requestId 不匹配 active=")
            << m_activeTask.captureRequestId
            << QStringLiteral(" bundle=") << bundle.request.requestId;
        return;
    }

    constexpr quint16 kFailCapture = 1u << 4;
    const bool cxpOk = bundle.cxpParticipated() &&
                       bundle.hikCameraAResult.success() &&
                       bundle.hikCameraBResult.success();
    const bool ok = bundle.mechEyeResult.success() && cxpOk;

    if (!ok) {
        writeSelfCheckFailWords({kFailCapture});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCapture);
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("自检采集失败：") << bundle.summary()
            << QStringLiteral(" cxpParticipated=") << bundle.cxpParticipated();
        return;
    }

    writeSelfCheckFailWords({0});
    completeActiveTask(1, protocol::AckState::Completed, true);
    notifySelfCheckFinished(1, 0);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("自检采集通过：") << bundle.summary()
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex;
}

}  // namespace scan_tracking::flow_control
