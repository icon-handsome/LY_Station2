#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

namespace {

/// 忙碌期间锁存的 Trig 上升沿（bit=trigOffset）。放文件静态，避免改 StateMachine 布局。
quint64 g_latchedTrigRisingMask = 0;

}  // namespace

void StateMachine::onModbusConnected()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    qInfo(LOG_FLOW) << QStringLiteral("Modbus 已连接，流程控制就绪。");

    if (m_activeTask.definition != nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Modbus 重连后清除残留活动任务：")
            << protocol::triggerName(*m_activeTask.definition);
        clearActiveTask();
    }

    // 重连后丢弃旧命令块，首帧只建快照；避免把 PLC 仍拉高的 Trig 当成新触发。
    m_lastCommandBlock.clear();
    m_blockTrigUntilIdleOffset = -1;
    g_latchedTrigRisingMask = 0;
    m_advancePathAfterTriggerRelease = false;
    m_codeReadPending = false;
    m_codeReadSoftPending = false;
    m_codeReadCameraIp.clear();

    m_isPollingPlc = false;
    resetModbusFailureCounter();
    m_consecutiveModbusFailures = 0;
    m_ipcState = protocol::IpcState::Ready;
    m_currentStage = protocol::Stage::Idle;
    m_alarmLevel = 0;
    m_alarmCode = 0;
    m_warnCode = 0;
    m_progress = 0;
    m_dataValid = false;
    setState(AppState::Ready);
    publishIpcStatus();
    publishHeartbeat();
    m_pollTimer->start();
    m_heartbeatTimer->start();

    qInfo(LOG_FLOW) << QStringLiteral("Modbus 重连恢复完成，系统已回到就绪状态。");
}

void StateMachine::onModbusDisconnected()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    qWarning(LOG_FLOW) << QStringLiteral("Modbus 已断开，流程控制暂停。");
    m_pollTimer->stop();
    m_heartbeatTimer->stop();
    m_timeoutTimer->stop();
    m_isPollingPlc = false;
    enterFaultState(900, QStringLiteral("Modbus 已断开连接"), true, false);
}

void StateMachine::onModbusError(const QString& errorString)
{
    qWarning(LOG_FLOW).noquote() << "Modbus 错误传播到流程控制：" << errorString;
    recordModbusFailure(901, errorString);
}

void StateMachine::pollPlcState()
{
}

void StateMachine::handleRegistersRead(int startAddress, const QVector<quint16>& values)
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (startAddress != protocol::registers::kCommandBlockStart ||
        values.size() < protocol::registers::kCommandBlockSize) {
        return;
    }

    const QVector<quint16> previousCommandBlock = m_lastCommandBlock;
    m_lastCommandBlock = values;
    m_robotTcpPose = protocol::registers::readRobotTcpPoseFromCommandBlock(values);
    resetModbusFailureCounter();

    bool commandBlockChanged = previousCommandBlock.isEmpty();
    if (!commandBlockChanged) {
        const int compareCount = qMin(previousCommandBlock.size(), values.size());
        for (int index = 1; index < compareCount; ++index) {
            if (previousCommandBlock.value(index) != values.value(index)) {
                commandBlockChanged = true;
                break;
            }
        }
    }

    if (m_activePollRequestSequence == 1 || (m_activePollRequestSequence % kPollLogEveryN) == 0) {
        qDebug(LOG_FLOW).noquote()
            << QStringLiteral("PLC 轮询完成")
            << QStringLiteral(" 请求序号=") << m_activePollRequestSequence
            << QStringLiteral(" 耗时ms=") << (m_pollRequestTimer.isValid() ? m_pollRequestTimer.elapsed() : -1);
    }
    m_activePollRequestSequence = 0;

    if (commandBlockChanged) {
        namespace regs = protocol::registers;
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("命令块快照：")
            << "Flow_Enable=" << values.value(regs::kFlowEnable)
            << "ArmScanSegmentIndex(AO47)="
            << regs::plcAnalogToUInt16(values.value(regs::kArmScanSegmentIndex), 0)
            << "TelescopicScanSegmentIndex(AO48)="
            << regs::plcAnalogToUInt16(values.value(regs::kTelescopicScanSegmentIndex), 0)
            << "Trig_ScanSegment=" << values.value(regs::modbusIndexFromPlcAddress(40023))
            << "Trig_TelescopicScan=" << values.value(regs::kTrigTelescopicScan)
            << "Trig_Inspection=" << values.value(regs::modbusIndexFromPlcAddress(40024))
            << "ScanPathId="
            << regs::plcAnalogToUInt16(values.value(regs::kScanPathId), 0)
            << "TaskIdHigh=" << values.value(regs::kTaskIdHigh)
            << "TaskIdLow=" << values.value(regs::kTaskIdLow);
    }

    if (!previousCommandBlock.isEmpty()) {
        static const char* const kRegisterNames[] = {
            "Reserved_0", "PLC_Heartbeat", "PLC_SystemState", "Station_WorkMode", "Flow_Enable",
            "Safety_Status_Word", "Cmd_StartAuto", "Cmd_Pause", "Cmd_Stop", "Cmd_Reset",
            "Cmd_ClearAlarms", "TaskId_H", "TaskId_L", "ProductType", "RecipeId",
            "ArmScanSegmentIndex_AO47", "TelescopicScanSegmentIndex_AO48", "RequestTimeout_s", "Robot_Status_Word",
            "Reserved_CmdExt_19", "Trig_LoadGrasp", "Trig_StationMaterialCheck", "Trig_PoseCheck",
            "Trig_ScanSegment", "Trig_Inspection", "Trig_UnloadCalc", "Trig_SelfCheck",
            "Trig_CodeRead", "Trig_ResultReset",
            "RobotTcpX_w0", "RobotTcpX_w1", "RobotTcpY_w0", "RobotTcpY_w1", "RobotTcpZ_w0",
            "RobotTcpZ_w1", "RobotTcpRx_w0", "RobotTcpRx_w1", "RobotTcpRy_w0", "RobotTcpRy_w1",
            "RobotTcpRz_w0", "RobotTcpRz_w1",
            "TelescopicRodStatus", "RollerSetFreqHz", "RollerRunFreqHz", "ElectromagnetStatus",
            "EstopButtonStatus", "Trig_TelescopicScan", "ScanPathId", "Reserved_CmdExt_48",
            "Reserved_CmdExt_49", "Reserved_CmdExt_50",
        };
        constexpr int kNameCount = sizeof(kRegisterNames) / sizeof(kRegisterNames[0]);
        const int compareCount = qMin(previousCommandBlock.size(),
                                      qMin(values.size(), protocol::registers::kCommandBlockSize));

        QStringList changedFields;
        for (int index = 0; index < compareCount; ++index) {
            const quint16 oldValue = previousCommandBlock.value(index);
            const quint16 newValue = values.value(index);
            if (oldValue == newValue) {
                continue;
            }
            const char* name = (index < kNameCount) ? kRegisterNames[index] : "?";
            changedFields << QStringLiteral("  [%1] %2: %3")
                                 .arg(protocol::registers::holdingRegisterAddress(index))
                                 .arg(QString::fromLatin1(name))
                                 .arg(formatPlcRegisterChangeForLog(index, oldValue, newValue));
        }
        if (!changedFields.isEmpty()) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("=== PLC 寄存器变化 ===") << "\n"
                << changedFields.join(QStringLiteral("\n"));
        }
    }

    if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
        finalizeCompletedTaskIfTriggerReleased(values);

        // 完成时 Trig 已是 0，但中间无 PLC 写事件；本次又看到同触发 0→1，
        // 视为「已释放 + 新触发」，先收尾旧任务再往下接受。
        if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
            const int trigOffset = m_activeTask.definition->trigOffset;
            if (trigOffset >= 0 &&
                trigOffset < previousCommandBlock.size() &&
                trigOffset < values.size() &&
                previousCommandBlock.value(trigOffset) == 0 &&
                values.value(trigOffset) == 1) {
                qInfo(LOG_FLOW).noquote()
                    << QStringLiteral("任务已完成且 Trig 重新置位，强制释放旧任务：")
                    << protocol::triggerName(*m_activeTask.definition);
                finalizeCompletedTaskIfTriggerReleased(values, true);
            }
        }
    }

    // 任意 Trig 的 0→1：忙碌时也锁存，避免臂扫进行中错过伸缩杆上升沿后永久丢触发。
    if (!previousCommandBlock.isEmpty()) {
        for (const auto& trigger : protocol::triggerDefinitions()) {
            if (trigger.trigOffset < 0 ||
                trigger.trigOffset >= 64 ||
                trigger.trigOffset >= values.size() ||
                trigger.trigOffset >= previousCommandBlock.size()) {
                continue;
            }
            const quint16 prev = previousCommandBlock.value(trigger.trigOffset);
            const quint16 curr = values.value(trigger.trigOffset);
            const quint64 bit = 1ull << static_cast<unsigned>(trigger.trigOffset);
            if (curr == 0) {
                g_latchedTrigRisingMask &= ~bit;
            } else if (prev == 0 && curr == 1) {
                g_latchedTrigRisingMask |= bit;
            }
        }
    }

    // 强制收尾后的闭锁：该 Trig 必须先回到 0 才允许再次接受。
    // offset 必须是真实 Trig 下标（>0）；0 为保留字，避免误判「解除闭锁」。
    if (m_blockTrigUntilIdleOffset > 0 &&
        m_blockTrigUntilIdleOffset < values.size() &&
        values.value(m_blockTrigUntilIdleOffset) == 0) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("Trig 已回 0，解除重复触发闭锁 offset=")
            << m_blockTrigUntilIdleOffset;
        m_blockTrigUntilIdleOffset = -1;
    }

    if (m_activeTask.definition != nullptr) {
        return;
    }

    // 空闲时：仅在 40047 相对上次轮询变化时切路，避免覆盖 IPC 自动切路结果。
    applyPlcScanPathId(values, previousCommandBlock, true);

    // 首帧（previous 为空）只建快照，不消费已为 1 的 Trig（IPC 重启/重连常见残留）。
    if (previousCommandBlock.isEmpty()) {
        for (const auto& trigger : protocol::triggerDefinitions()) {
            if (trigger.trigOffset >= 0 &&
                trigger.trigOffset < values.size() &&
                values.value(trigger.trigOffset) == 1) {
                qInfo(LOG_FLOW).noquote()
                    << QStringLiteral("忽略启动/重连首帧残留触发 ")
                    << protocol::triggerName(trigger)
                    << QStringLiteral("（需 PLC 先拉低再置 1）");
            }
        }
        return;
    }

    if (const protocol::TriggerDefinition* pendingTrigger =
            selectPendingTrigger(values, previousCommandBlock)) {
        if (pendingTrigger->trigOffset >= 0 && pendingTrigger->trigOffset < 64) {
            g_latchedTrigRisingMask &= ~(1ull << static_cast<unsigned>(pendingTrigger->trigOffset));
        }
        processTrigger(*pendingTrigger, values);
    }
}

void StateMachine::onRegisterReadFailed(int startAddress, const QString& errorString)
{
    if (startAddress != protocol::registers::kCommandBlockStart) {
        return;
    }

    if (m_isPollingPlc) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("PLC 轮询失败：") << errorString
            << QStringLiteral(" 请求序号=") << m_activePollRequestSequence;
    }
    m_isPollingPlc = false;
    m_activePollRequestSequence = 0;
}

void StateMachine::onRegisterWriteFailed(int startAddress, const QString& errorString)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("寄存器写入失败，地址=") << startAddress << errorString;
}

void StateMachine::processTrigger(const protocol::TriggerDefinition& trigger, const QVector<quint16>& commandBlock)
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;
    }

    // Trig 前以 PLC ScanPathId 为准（同一次轮询写 path 再置 Trig 也能生效）。
    applyPlcScanPathId(commandBlock);

    if (const auto* configMgr = common::ConfigManager::instance()) {
        const auto& profile = configMgr->stationProfile();
        if (!isTriggerEnabledForProfile(profile, trigger.trigOffset)) {
            rejectDisabledTrigger(trigger);
            return;
        }
    }

    if (trigger.stage != protocol::Stage::UnloadCalc &&
        trigger.stage != protocol::Stage::ResultReset &&
        commandBlock.value(protocol::registers::kFlowEnable) == 0) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("Flow_Enable=0 时拒绝触发：")
                                     << protocol::triggerName(trigger);
        sendRes(trigger, 9);
        sendAck(trigger, protocol::AckState::Failed);
        return;
    }

    m_activeTask.definition = &trigger;
    m_activeTask.taskId = readTaskId(commandBlock);
    {
        const quint16 timeoutRaw = commandBlock.value(protocol::registers::kRequestTimeoutSeconds);
        const quint16 timeoutDecoded = protocol::registers::plcAnalogToUInt16(timeoutRaw, 0);
        m_activeTask.timeoutSeconds = timeoutDecoded > 0
            ? timeoutDecoded
            : static_cast<quint16>(trigger.defaultTimeoutSeconds);
    }
    m_activeTask.scanSegmentIndex = resolveScanSegmentIndex(commandBlock, trigger.stage);
    m_activeTask.inspectionPathId = 0;

    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        m_activeTask.inspectionPathId = cfgMgr->activePathId();
        const int configuredTotal = cfgMgr->enabledScanPointCount();
        m_activeTask.scanSegmentTotal = configuredTotal > 0
            ? configuredTotal
            : cfgMgr->trackingConfig().scanSegmentTotal;
    } else {
        m_activeTask.scanSegmentTotal = 1;
    }
    m_activeTask.completionAnnounced = false;
    m_activeTask.captureRequestId = 0;

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已接受触发") << protocol::triggerName(trigger)
        << QStringLiteral(" pathId=") << m_activeTask.inspectionPathId
        << QStringLiteral(" 超时s=") << m_activeTask.timeoutSeconds
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex
        << QStringLiteral("/") << m_activeTask.scanSegmentTotal;

    setAlarm(0, 0, QString());
    setState(AppState::Scanning);
    m_ipcState = protocol::IpcState::Busy;
    m_currentStage = trigger.stage;
    m_progress = 5;
    m_dataValid = false;
    publishIpcStatus();

    sendAck(trigger, protocol::AckState::Running);

    if (m_activeTask.taskId != 0) {
        const bool taskIdWritten = m_modbus->writeRegisters(protocol::registers::kTaskIdEchoHigh, {
            static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu),
            static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),
        });
        if (!taskIdWritten) {
            qWarning(LOG_FLOW).noquote() << QStringLiteral("写入任务 ID 回声寄存器失败");
        }
    }

    m_timeoutTimer->start(static_cast<int>(m_activeTask.timeoutSeconds) * 1000);
    executeActiveTask();
}

void StateMachine::rejectDisabledTrigger(const protocol::TriggerDefinition& trigger)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[Station] 触发器")
        << protocol::triggerName(trigger)
        << QStringLiteral("在当前 profile 未启用，已拒绝，Res=8");
    sendAck(trigger, protocol::AckState::Running);
    sendRes(trigger, 8);
    sendAck(trigger, protocol::AckState::Failed);
}

void StateMachine::sendAck(const protocol::TriggerDefinition& definition, protocol::AckState ackState)
{
    if (!m_modbus) {
        return;
    }

    const bool ackWritten = m_modbus->writeRegister(definition.ackOffset, static_cast<quint16>(ackState));
    if (!ackWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 Ack 状态失败");
    }
}

void StateMachine::sendRes(const protocol::TriggerDefinition& definition, quint16 resultCode)
{
    if (!m_modbus) {
        return;
    }

    const bool resWritten = m_modbus->writeRegister(definition.resOffset, resultCode);
    if (!resWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 Res 结果码失败");
    }
}

void StateMachine::resetPlcOutputRegisters()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;
    }

    if (m_modbus->resetIpcResultBlock()) {
        qInfo(LOG_FLOW).noquote() << QStringLiteral("程序退出：IPC 结果区已清零");
    } else {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("程序退出：IPC 结果区清零失败");
    }
}

void StateMachine::publishIpcStatus()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (!m_modbus || !m_modbus->isConnected()) {
        return;
    }

    QVector<quint16> status = {
        m_heartbeatCounter,
        static_cast<quint16>(m_ipcState),
        static_cast<quint16>(m_currentStage),
        m_alarmLevel,
        m_alarmCode,
        m_warnCode,
        static_cast<quint16>(m_state == AppState::Ready ? 1 : 0),
        static_cast<quint16>(m_dataValid ? 1 : 0),
        m_progress,
        kDeviceOnlineWord0,
        0, 0, 0,
        static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu),
        static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),
    };

    if (!m_modbus->writeRegisters(protocol::registers::kIpcHeartbeat, status)) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 IPC 心跳状态失败");
    }
}

void StateMachine::publishHeartbeat()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (!m_modbus || !m_modbus->isConnected()) {
        return;
    }

    ++m_heartbeatCounter;
    publishIpcStatus();
}

void StateMachine::onProcessTimeout()
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (m_activeTask.definition == nullptr) {
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("任务超时：") << protocol::triggerName(*m_activeTask.definition);
    setAlarm(2, 610, QStringLiteral("任务超时"));
    m_activeTask.captureRequestId = 0;

    if (isScanCaptureStage(m_activeTask.definition->stage)) {
        // 段扫兼跑编号时，超时需清 OCR 等待态，避免迟到回包误收尾。
        m_codeReadPending = false;
        m_codeReadSoftPending = false;
        m_codeReadCameraIp.clear();
        m_advancePathAfterTriggerRelease = false;
        completeScanSegmentCapture(6, 0, 0, protocol::AckState::Failed, false);
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::SelfCheck) {
        constexpr quint16 kFailCapture = 1u << 4;
        writeSelfCheckFailWords({kFailCapture});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCapture);
        return;
    }
    if (m_codeReadPending || isActiveCodeReadTrigger()) {
        finishCodeRead(2, QString(), QStringLiteral("编号识别超时。"));
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::Inspection) {
        InspectionResult timeoutResult;
        timeoutResult.resultCode = kInspectionResTimeoutNg;
        timeoutResult.message = QStringLiteral("检测任务超时");
        finishInspection(timeoutResult);
        return;
    }

    completeActiveTask(6, protocol::AckState::Completed, false);
}

bool StateMachine::completeActiveTask(
    quint16 resultCode,
    protocol::AckState finalAckState,
    bool dataValid)
{
    if (m_activeTask.definition == nullptr || !m_modbus) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("无法完成任务：任务定义或 Modbus 为空");
        return false;
    }

    const int ackOffset = m_activeTask.definition->ackOffset;
    const int resOffset = m_activeTask.definition->resOffset;

    auto failCompletionWrite = [this](const QString& reason) -> bool {
        qWarning(LOG_FLOW).noquote() << reason;
        enterFaultState(902, reason, false, false);
        return false;
    };

    if (resOffset == ackOffset + 1) {
        if (!m_modbus->writeRegisters(ackOffset, {
                static_cast<quint16>(finalAckState),
                resultCode,
            })) {
            return failCompletionWrite(QStringLiteral("批量写入 Ack/Res 失败"));
        }
    } else {
        sendRes(*m_activeTask.definition, resultCode);
        sendAck(*m_activeTask.definition, finalAckState);
    }

    m_timeoutTimer->stop();
    m_progress = 100;
    m_dataValid = dataValid;
    m_activeTask.completionAnnounced = true;
    m_activeTask.captureRequestId = 0;
    publishIpcStatus();

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("触发已完成") << protocol::triggerName(*m_activeTask.definition)
        << QStringLiteral(" Res=") << resultCode
        << QStringLiteral(" Ack=") << static_cast<int>(finalAckState);

    // PLC 常在采集完成前就把 Trig 拉低；若等下次写寄存器才 finalize，
    // 下次再置 1 时会因「Trig≠0」无法释放，后续触发全部被忽略。
    finalizeCompletedTaskIfTriggerReleased(m_lastCommandBlock);
    return true;
}

void StateMachine::finalizeCompletedTaskIfTriggerReleased(
    const QVector<quint16>& commandBlock,
    bool force)
{
    if (m_activeTask.definition == nullptr || !m_activeTask.completionAnnounced) {
        return;
    }

    const int trigOffset = m_activeTask.definition->trigOffset;
    if (!force && (trigOffset >= commandBlock.size() || commandBlock[trigOffset] != 0)) {
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << (force ? QStringLiteral("强制释放已完成触发：")
                  : QStringLiteral("PLC 已释放触发："))
        << protocol::triggerName(*m_activeTask.definition);

    const protocol::TriggerDefinition& definition = *m_activeTask.definition;
    if (definition.stage == protocol::Stage::ScanSegment) {
        writeScanSegmentResult(0, 0, 0);
    } else if (definition.stage == protocol::Stage::TelescopicScan) {
        writeTelescopicScanResult(0, 0, 0);
    }

    if (m_modbus) {
        const int ackOffset = definition.ackOffset;
        const int resOffset = definition.resOffset;
        if (resOffset == ackOffset + 1) {
            m_modbus->writeRegisters(ackOffset, {
                static_cast<quint16>(protocol::AckState::Idle),
                0,
            });
        } else {
            sendRes(definition, 0);
            sendAck(definition, protocol::AckState::Idle);
        }
    }

    clearActiveTask();
    m_ipcState = protocol::IpcState::Ready;
    m_currentStage = protocol::Stage::Idle;
    m_progress = 0;
    setState(AppState::Ready);
    publishIpcStatus();

    if (m_advancePathAfterTriggerRelease) {
        m_advancePathAfterTriggerRelease = false;
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("Trig 已释放，执行编号段扫后的自动切路径。");
        prepareNextScanPathAfterSuccess();
    }
}

const protocol::TriggerDefinition* StateMachine::selectPendingTrigger(
    const QVector<quint16>& commandBlock,
    const QVector<quint16>& previousCommandBlock) const
{
    // 无上一拍时无法判断上升沿，不接受任何 Trig。
    if (previousCommandBlock.isEmpty()) {
        return nullptr;
    }

    for (const auto& trigger : protocol::triggerDefinitions()) {
        if (trigger.trigOffset < 0 || trigger.trigOffset >= commandBlock.size()) {
            continue;
        }
        if (trigger.trigOffset >= previousCommandBlock.size()) {
            continue;
        }
        if (commandBlock.value(trigger.trigOffset) != 1) {
            continue;
        }
        // 看门狗强制收尾后：同一 Trig 仍为 1 时忽略，直到其先回 0。
        if (m_blockTrigUntilIdleOffset > 0 &&
            trigger.trigOffset == m_blockTrigUntilIdleOffset) {
            continue;
        }

        const bool risingEdge =
            previousCommandBlock.value(trigger.trigOffset) == 0;
        const bool latchedWhileBusy =
            trigger.trigOffset >= 0 && trigger.trigOffset < 64 &&
            (g_latchedTrigRisingMask & (1ull << static_cast<unsigned>(trigger.trigOffset))) != 0;
        // 本拍 0→1，或忙碌期间已锁存且空闲后 Trig 仍保持为 1。
        if (!risingEdge && !latchedWhileBusy) {
            continue;
        }
        if (latchedWhileBusy && !risingEdge) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("补接受忙碌期间锁存的触发 ")
                << protocol::triggerName(trigger);
        }
        return &trigger;
    }
    return nullptr;
}

void StateMachine::recordModbusFailure(quint16 alarmCode, const QString& message)
{
    ++m_consecutiveModbusFailures;
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("记录 Modbus 失败")
        << m_consecutiveModbusFailures << QStringLiteral("/") << kMaxConsecutiveModbusFailures
        << message;

    if (m_consecutiveModbusFailures >= kMaxConsecutiveModbusFailures) {
        enterFaultState(alarmCode, message, true, true);
    }
}

void StateMachine::resetModbusFailureCounter()
{
    m_consecutiveModbusFailures = 0;
}

void StateMachine::enterFaultState(
    quint16 alarmCode,
    const QString& message,
    bool abortCurrentTask,
    bool notifyPlc)
{
    setAlarm(3, alarmCode, message);
    m_ipcState = protocol::IpcState::Fault;
    setState(AppState::Error);

    if (abortCurrentTask) {
        abortActiveTaskForFault(7);
    } else {
        m_timeoutTimer->stop();
        m_progress = 0;
        m_currentStage = protocol::Stage::Idle;
        publishIpcStatus();
    }

    if (!notifyPlc) {
        clearActiveTask();
        m_currentStage = protocol::Stage::Idle;
    }
}

void StateMachine::abortActiveTaskForFault(quint16 resultCode)
{
    if (m_activeTask.definition == nullptr) {
        m_timeoutTimer->stop();
        m_progress = 0;
        m_dataValid = false;
        m_currentStage = protocol::Stage::Idle;
        publishIpcStatus();
        return;
    }

    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
    } else if (m_activeTask.definition->stage == protocol::Stage::TelescopicScan) {
        writeTelescopicScanResult(m_activeTask.scanSegmentIndex, 0, 0);
    }

    if (m_modbus && m_modbus->isConnected()) {
        completeActiveTask(resultCode, protocol::AckState::Failed, false);
        return;
    }

    m_timeoutTimer->stop();
    m_progress = 0;
    m_dataValid = false;
    m_activeTask.captureRequestId = 0;
    m_activeTask.completionAnnounced = false;
    clearActiveTask();
    m_currentStage = protocol::Stage::Idle;
    publishIpcStatus();
}

quint32 StateMachine::readTaskId(const QVector<quint16>& commandBlock) const
{
    const quint32 high = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdHigh));
    const quint32 low = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdLow));
    return (high << 16) | low;
}

quint16 StateMachine::resolveScanSegmentIndex(const QVector<quint16>& commandBlock,
                                              protocol::Stage stage) const
{
    return protocol::registers::resolveScanSegmentIndexFromBlock(commandBlock, stage);
}

}  // namespace scan_tracking::flow_control
