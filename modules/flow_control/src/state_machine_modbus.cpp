#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

void StateMachine::onModbusConnected()
{
    qInfo(LOG_FLOW) << QStringLiteral("Modbus 已连接，流程控制就绪。");

    if (m_activeTask.definition != nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Modbus 重连后清除残留活动任务：")
            << protocol::triggerName(*m_activeTask.definition);
        clearActiveTask();
    }

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
            << "ScanSegmentIndex=" << protocol::registers::resolveScanSegmentIndexFromBlock(values)
            << "Trig_ScanSegment=" << values.value(regs::modbusIndexFromPlcAddress(40023))
            << "Trig_Inspection=" << values.value(regs::modbusIndexFromPlcAddress(40024))
            << "TaskIdHigh=" << values.value(regs::kTaskIdHigh)
            << "TaskIdLow=" << values.value(regs::kTaskIdLow);
    }

    if (!previousCommandBlock.isEmpty()) {
        static const char* const kRegisterNames[] = {
            "Reserved_0", "PLC_Heartbeat", "PLC_SystemState", "Station_WorkMode", "Flow_Enable",
            "Safety_Status_Word", "Cmd_StartAuto", "Cmd_Pause", "Cmd_Stop", "Cmd_Reset",
            "Cmd_ClearAlarms", "TaskId_H", "TaskId_L", "ProductType", "RecipeId",
            "ScanSegmentIndex", "ScanSegmentIndex_Robot", "RequestTimeout_s", "Robot_Status_Word",
            "Reserved_19", "Trig_LoadGrasp", "Trig_StationMaterialCheck", "Trig_PoseCheck",
            "Trig_ScanSegment", "Trig_Inspection", "Trig_UnloadCalc", "Trig_SelfCheck",
            "Trig_CodeRead", "Trig_ResultReset",
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
                << QStringLiteral("=== PLC 寄存器变化 ===") << "\n" << changedFields.join(QStringLiteral("\n"));
        }
    }

    if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
        finalizeCompletedTaskIfTriggerReleased(values);
    }

    if (m_activeTask.definition != nullptr) {
        return;
    }

    if (const protocol::TriggerDefinition* pendingTrigger = selectPendingTrigger(values)) {
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
    m_activeTask.scanSegmentIndex = resolveScanSegmentIndex(commandBlock);
    m_activeTask.inspectionPathId = 0;

    if (const auto* cfgMgr = common::ConfigManager::instance()) {
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
        << QStringLiteral(" 超时s=") << m_activeTask.timeoutSeconds
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex;

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
    if (!m_modbus || !m_modbus->isConnected()) {
        return;
    }

    ++m_heartbeatCounter;
    publishIpcStatus();
}

void StateMachine::onProcessTimeout()
{
    if (m_activeTask.definition == nullptr) {
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("任务超时：") << protocol::triggerName(*m_activeTask.definition);
    setAlarm(2, 610, QStringLiteral("任务超时"));
    m_activeTask.captureRequestId = 0;

    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        completeScanSegmentCapture(6, 0, 0, protocol::AckState::Failed, false);
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
    return true;
}

void StateMachine::finalizeCompletedTaskIfTriggerReleased(const QVector<quint16>& commandBlock)
{
    if (m_activeTask.definition == nullptr || !m_activeTask.completionAnnounced) {
        return;
    }

    const int trigOffset = m_activeTask.definition->trigOffset;
    if (trigOffset >= commandBlock.size() || commandBlock[trigOffset] != 0) {
        return;
    }

    qInfo(LOG_FLOW).noquote() << QStringLiteral("PLC 已释放触发：")
                              << protocol::triggerName(*m_activeTask.definition);

    const protocol::TriggerDefinition& definition = *m_activeTask.definition;
    if (definition.stage == protocol::Stage::ScanSegment) {
        writeScanSegmentResult(0, 0, 0);
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
}

const protocol::TriggerDefinition* StateMachine::selectPendingTrigger(
    const QVector<quint16>& commandBlock) const
{
    for (const auto& trigger : protocol::triggerDefinitions()) {
        if (trigger.trigOffset < commandBlock.size() && commandBlock[trigger.trigOffset] == 1) {
            return &trigger;
        }
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

quint16 StateMachine::resolveScanSegmentIndex(const QVector<quint16>& commandBlock) const
{
    return protocol::registers::resolveScanSegmentIndexFromBlock(commandBlock);
}

}  // namespace scan_tracking::flow_control
