#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"
#include "scan_tracking/flow_control/task_handler_registry.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QRegExp>
#include <QtCore/QThread>

#include <array>
#include <cstring>

namespace scan_tracking::flow_control {

Q_LOGGING_CATEGORY(LOG_FLOW, "flow_control")

namespace {

constexpr quint16 kDeviceOnlineWord0 =
    (1u << 0) | (1u << 1) | (1u << 2) | (1u << 4) | (1u << 5) | (1u << 6);

constexpr int kPollLogEveryN = 20;
constexpr int kMaxConsecutiveModbusFailures = 3;
constexpr quint16 kInspectionResTimeoutNg = 6;

QString formatPlcRegisterValueForLog(int modbusIndex, quint16 rawValue)
{
    namespace regs = protocol::registers;
    if (modbusIndex == regs::kScanSegmentIndex || modbusIndex == regs::kScanSegmentIndexRobot) {
        const quint16 decoded = regs::plcAnalogToUInt16(rawValue, 0);
        if (rawValue != decoded) {
            return QStringLiteral("%1 (原始PLC字=%2)").arg(decoded).arg(rawValue);
        }
        return QString::number(decoded);
    }
    if (modbusIndex == regs::kRequestTimeoutSeconds) {
        const quint16 decoded = regs::plcAnalogToUInt16(rawValue, 0);
        if (rawValue != decoded) {
            return QStringLiteral("%1s (原始PLC字=%2)").arg(decoded).arg(rawValue);
        }
        return QStringLiteral("%1").arg(decoded);
    }
    return QString::number(rawValue);
}

QString formatPlcRegisterChangeForLog(int modbusIndex, quint16 oldValue, quint16 newValue)
{
    return QStringLiteral("%1 -> %2")
        .arg(formatPlcRegisterValueForLog(modbusIndex, oldValue))
        .arg(formatPlcRegisterValueForLog(modbusIndex, newValue));
}

QVector<quint16> floatToCdabRegisters(float value)
{
    quint32 raw = 0;
    static_assert(sizeof(raw) == sizeof(value), "Unexpected float width");
    std::memcpy(&raw, &value, sizeof(raw));

    const quint16 high = static_cast<quint16>((raw >> 16) & 0xFFFFu);
    const quint16 low = static_cast<quint16>(raw & 0xFFFFu);
    return {low, high};
}

StateMachine::PoseSourceResult parsePoseSource(
    const char* envName,
    const QString& sourceName,
    const std::array<float, 6>& fallback,
    bool treatMissingAsSimulated)
{
    StateMachine::PoseSourceResult result;
    result.available = true;
    result.sourceName = sourceName;

    const QString raw = qEnvironmentVariable(envName).trimmed();
    if (raw.isEmpty()) {
        result.success = true;
        result.message = treatMissingAsSimulated
            ? QStringLiteral("未配置外部位姿源；使用模拟回退。")
            : QStringLiteral("未配置外部位姿源。");
        result.x = fallback[0];
        result.y = fallback[1];
        result.z = fallback[2];
        result.rx = fallback[3];
        result.ry = fallback[4];
        result.rz = fallback[5];
        return result;
    }

    const auto tokens = raw.split(QRegExp(QStringLiteral("[,;\\s]+")), Qt::SkipEmptyParts);
    if (tokens.size() < 6) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 需要 6 个值：x,y,z,rx,ry,rz。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }

    bool ok = false;
    const float values[6] = {
        tokens.value(0).toFloat(&ok),
        ok ? tokens.value(1).toFloat(&ok) : 0.0f,
        ok ? tokens.value(2).toFloat(&ok) : 0.0f,
        ok ? tokens.value(3).toFloat(&ok) : 0.0f,
        ok ? tokens.value(4).toFloat(&ok) : 0.0f,
        ok ? tokens.value(5).toFloat(&ok) : 0.0f,
    };
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }

    result.success = true;
    result.message = QStringLiteral("从外部源 %1 加载位姿。").arg(QString::fromLatin1(envName));
    result.x = values[0];
    result.y = values[1];
    result.z = values[2];
    result.rx = values[3];
    result.ry = values[4];
    result.rz = values[5];
    return result;
}

}  // namespace

StateMachine::StateMachine(
    modbus::ModbusService* modbusService,
    mech_eye::MechEyeService* mechEyeService,
    vision::VisionPipelineService* visionPipelineService,
    QObject* parent)
    : QObject(parent)
    , m_modbus(modbusService)
    , m_mechEye(mechEyeService)
    , m_visionPipeline(visionPipelineService)
    , m_pollTimer(new QTimer(this))
    , m_heartbeatTimer(new QTimer(this))
    , m_timeoutTimer(new QTimer(this))
    , m_handlerRegistry(std::make_unique<TaskHandlerRegistry>())
    , m_state(AppState::Init)
{
    const auto* configMgr = common::ConfigManager::instance();
    const auto flowConfig = configMgr ? configMgr->flowControlConfig()
                                      : common::FlowControlConfig{100, 1000, 300};
    if (configMgr) {
        const auto& profile = configMgr->stationProfile();
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[Station] StateMachine stationId=")
            << common::stationIdToInt(profile.stationId)
            << QStringLiteral(" workMode=")
            << common::workModeIdToString(profile.defaultWorkMode);
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[Station] handlers=")
            << m_handlerRegistry->handlerCount()
            << QStringLiteral(" enabledTriggers=")
            << m_handlerRegistry->enabledTriggerNames(profile).join(QLatin1Char(','));
    }

    m_pollTimer->setInterval(flowConfig.pollIntervalMs);
    m_heartbeatTimer->setInterval(flowConfig.heartbeatIntervalMs);
    m_timeoutTimer->setSingleShot(true);

    connect(m_pollTimer, &QTimer::timeout, this, &StateMachine::pollPlcState);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &StateMachine::publishHeartbeat);
    connect(m_timeoutTimer, &QTimer::timeout, this, &StateMachine::onProcessTimeout);

    if (m_modbus) {
        connect(m_modbus, &modbus::ModbusService::connected, this, &StateMachine::onModbusConnected);
        connect(m_modbus, &modbus::ModbusService::disconnected, this, &StateMachine::onModbusDisconnected);
        connect(m_modbus, &modbus::ModbusService::errorOccurred, this, &StateMachine::onModbusError);
        connect(m_modbus, &modbus::ModbusService::registersRead, this, &StateMachine::handleRegistersRead);
        connect(m_modbus, &modbus::ModbusService::registerReadFailed, this, &StateMachine::onRegisterReadFailed);
        connect(m_modbus, &modbus::ModbusService::registerWriteFailed, this, &StateMachine::onRegisterWriteFailed);
    }

    if (m_mechEye) {
        connect(
            m_mechEye,
            &mech_eye::MechEyeService::stateChanged,
            this,
            [](mech_eye::CameraRuntimeState state, QString desc) {
                qInfo(LOG_FLOW) << "[MechEye] 相机状态变更:" << static_cast<int>(state) << desc;
            });
        connect(
            m_mechEye,
            &mech_eye::MechEyeService::fatalError,
            this,
            &StateMachine::onMechEyeFatalError,
            Qt::QueuedConnection);
    }

    if (m_visionPipeline) {
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::stateChanged,
            this,
            [](vision::VisionPipelineState state, const QString& description) {
                qInfo(LOG_FLOW) << QStringLiteral("[VisionPipeline] 状态=") << static_cast<int>(state) << description;
            });
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::fatalError,
            this,
            [](vision::VisionErrorCode code, const QString& message) {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("[VisionPipeline] 致命错误：")
                    << static_cast<int>(code)
                    << message;
            });
    }
}

StateMachine::~StateMachine()
{
    stop();
}

void StateMachine::start()
{
    qInfo(LOG_FLOW) << QStringLiteral("状态机启动。");
    clearActiveTask();
    m_isPollingPlc = false;
    m_ipcState = protocol::IpcState::Initializing;
    m_currentStage = protocol::Stage::Idle;
    m_alarmLevel = 0;
    m_alarmCode = 0;
    m_warnCode = 0;
    m_progress = 0;
    m_dataValid = false;
    m_consecutiveModbusFailures = 0;
    setState(AppState::Init);
    publishIpcStatus();

    if (m_modbus && m_modbus->isConnected()) {
        onModbusConnected();
    }
}

void StateMachine::stop()
{
    if (m_stopped.exchange(true)) {
        return;
    }

    if (m_modbus != nullptr) {
        disconnect(m_modbus, nullptr, this, nullptr);
    }
    if (m_mechEye != nullptr) {
        disconnect(m_mechEye, nullptr, this, nullptr);
    }
    if (m_visionPipeline != nullptr) {
        disconnect(m_visionPipeline, nullptr, this, nullptr);
    }

    if (m_pollTimer != nullptr) {
        m_pollTimer->stop();
    }
    if (m_heartbeatTimer != nullptr) {
        m_heartbeatTimer->stop();
    }
    if (m_timeoutTimer != nullptr) {
        m_timeoutTimer->stop();
    }

    m_isPollingPlc = false;
    clearActiveTask();

    m_consecutiveModbusFailures = 0;
    m_alarmLevel = 0;
    m_alarmCode = 0;
    m_warnCode = 0;
    m_progress = 0;
    m_dataValid = false;
    m_heartbeatCounter = 0;
    m_ipcState = protocol::IpcState::Uninitialized;
    m_currentStage = protocol::Stage::Idle;
    resetPlcOutputRegisters();
    setState(AppState::Init);
}

void StateMachine::setState(AppState newState)
{
    if (m_state != newState) {
        m_state = newState;
        if (!m_stopped.load(std::memory_order_acquire)) {
            emit stateChanged(newState);
        }
        qInfo(LOG_FLOW) << QStringLiteral("应用状态切换为：") << static_cast<int>(newState);
    }
}

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
        m_activeTask.scanSegmentTotal = cfgMgr->trackingConfig().scanSegmentTotal;
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

void StateMachine::executeActiveTask()
{
    if (m_activeTask.definition == nullptr) {
        return;
    }

    ITaskHandler* handler = m_handlerRegistry
        ? m_handlerRegistry->handlerForOffset(m_activeTask.definition->trigOffset)
        : nullptr;
    if (handler == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("拒绝不支持的触发")
            << protocol::triggerName(*m_activeTask.definition);
        setAlarm(2, 624, QStringLiteral("收到不支持的触发"));
        completeActiveTask(9, protocol::AckState::Failed, false);
        return;
    }

    TaskHandlerContext ctx{*this, m_lastCommandBlock, m_activeTask};
    handler->execute(ctx);
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
        writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
        completeActiveTask(6, protocol::AckState::Failed, false);
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::Inspection) {
        writeInspectionResult({});
        completeActiveTask(kInspectionResTimeoutNg, protocol::AckState::Failed, false);
        return;
    }

    completeActiveTask(6, protocol::AckState::Completed, false);
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

    writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
    setAlarm(3, 723, message);
    completeActiveTask(7, protocol::AckState::Failed, false);
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

void StateMachine::clearActiveTask()
{
    m_activeTask = {};
}

void StateMachine::resetScanSegmentCache()
{
}

void StateMachine::setAlarm(quint16 level, quint16 code, const QString& message)
{
    m_alarmLevel = level;
    m_alarmCode = code;
    m_warnCode = level > 0 && level < 3 ? code : 0;
    if (!message.isEmpty()) {
        emit protocolEvent(message);
    }
}

bool StateMachine::writeIpcSafetyActionWord()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return false;
    }

    const bool written = m_modbus->writeRegisters(
        protocol::registers::kIpcSafetyActionWord,
        {m_ipcSafetyActionWord});
    if (!written) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("写入 IPC_SafetyAction_Word 失败，值=") << m_ipcSafetyActionWord;
    }
    return written;
}

bool StateMachine::reportPersonZoneAlarm(bool alarm)
{
    namespace safety = protocol::safety_bits;

    if (alarm) {
        m_ipcSafetyActionWord |= safety::kAiPersonIntrusion;
        const bool plcWritten = writeIpcSafetyActionWord();
        enterFaultState(601, QStringLiteral("监控区域检测到人员"), true, false);
        m_personZoneAlarmActive = true;
        publishIpcStatus();
        return plcWritten;
    }

    m_ipcSafetyActionWord &= ~static_cast<quint16>(safety::kAiPersonIntrusion);
    const bool plcWritten = writeIpcSafetyActionWord();

    if (m_personZoneAlarmActive) {
        setAlarm(0, 0, QString());
        m_ipcState = protocol::IpcState::Ready;
        m_currentStage = protocol::Stage::Idle;
        m_personZoneAlarmActive = false;
        setState(AppState::Ready);
    }

    publishIpcStatus();
    return plcWritten;
}

void StateMachine::writeFloatPlaceholder(int startOffset, float value)
{
    if (!m_modbus) {
        return;
    }

    if (!m_modbus->writeRegisters(startOffset, floatToCdabRegisters(value))) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入浮点占位符失败，偏移=") << startOffset;
    }
}

StateMachine::PoseSourceResult StateMachine::resolveLoadGraspPoseSource() const
{
    return parsePoseSource(
        "SCAN_TRACKING_LOAD_GRASP_POSE",
        QStringLiteral("load-grasp-provider"),
        {125.0f, 250.0f, 375.0f, 0.0f, 90.0f, 180.0f},
        true);
}

StateMachine::PoseSourceResult StateMachine::resolveUnloadCalcPoseSource() const
{
    return parsePoseSource(
        "SCAN_TRACKING_UNLOAD_CALC_POSE",
        QStringLiteral("unload-calc-provider"),
        {500.0f, 600.0f, 700.0f, 0.0f, 0.0f, 90.0f},
        true);
}

void StateMachine::writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text)
{
    if (!m_modbus) {
        return;
    }

    const QString padded = text.left(registerCount * 2).leftJustified(registerCount * 2, QLatin1Char(' '));
    QVector<quint16> values;
    values.reserve(registerCount);
    for (int i = 0; i < registerCount; ++i) {
        const QChar first = padded.at(i * 2);
        const QChar second = padded.at(i * 2 + 1);
        const quint16 packed = (static_cast<quint16>(first.unicode()) << 8) |
                               static_cast<quint16>(second.unicode() & 0xFF);
        values.push_back(packed);
    }
    if (!m_modbus->writeRegisters(startOffset, values)) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 ASCII 占位符失败，偏移=") << startOffset;
    }
}

void StateMachine::writeLoadGraspResult()
{
    const auto poseSource = resolveLoadGraspPoseSource();
    writeFloatPlaceholder(protocol::registers::kLoadX, poseSource.x);
    writeFloatPlaceholder(protocol::registers::kLoadY, poseSource.y);
    writeFloatPlaceholder(protocol::registers::kLoadZ, poseSource.z);
    writeFloatPlaceholder(protocol::registers::kLoadRx, poseSource.rx);
    writeFloatPlaceholder(protocol::registers::kLoadRy, poseSource.ry);
    writeFloatPlaceholder(protocol::registers::kLoadRz, poseSource.rz);
}

void StateMachine::writeUnloadCalcResult()
{
    const auto poseSource = resolveUnloadCalcPoseSource();
    writeFloatPlaceholder(protocol::registers::kUnloadX, poseSource.x);
    writeFloatPlaceholder(protocol::registers::kUnloadY, poseSource.y);
    writeFloatPlaceholder(protocol::registers::kUnloadZ, poseSource.z);
    writeFloatPlaceholder(protocol::registers::kUnloadRx, poseSource.rx);
    writeFloatPlaceholder(protocol::registers::kUnloadRy, poseSource.ry);
    writeFloatPlaceholder(protocol::registers::kUnloadRz, poseSource.rz);
}

void StateMachine::writeScanSegmentResult(int segmentIndex, int imageCount, int cloudFrameCount)
{
    if (!m_modbus) {
        return;
    }

    if (!m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {
            static_cast<quint16>(segmentIndex),
            static_cast<quint16>(imageCount),
            static_cast<quint16>(cloudFrameCount),
        })) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入扫描分段进度失败");
    }
}

void StateMachine::writeInspectionResult(const InspectionSummary& summary)
{
    if (!m_modbus) {
        return;
    }

    if (!m_modbus->writeRegisters(protocol::registers::kNgReasonWord0, {
            summary.ngReasonWord0,
            summary.ngReasonWord1,
            summary.measureItemCount,
        })) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入检测结果失败");
    }
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
