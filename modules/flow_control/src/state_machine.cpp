#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"
#include "scan_tracking/flow_control/task_handler_registry.h"

namespace scan_tracking::flow_control {

StateMachine::StateMachine(
    modbus::ModbusService* modbusService,
    orbbec_gemini::OrbbecGeminiService* orbbecGeminiService,
    vision::VisionPipelineService* visionPipelineService,
    QObject* parent)
    : QObject(parent)
    , m_modbus(modbusService)
    , m_orbbecGemini(orbbecGeminiService)
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

    if (m_orbbecGemini) {
        connect(
            m_orbbecGemini,
            &orbbec_gemini::OrbbecGeminiService::stateChanged,
            this,
            [](orbbec_gemini::OrbbecGeminiRuntimeState state, QString desc) {
                qInfo(LOG_FLOW) << "[Orbbec] 相机状态变更:" << static_cast<int>(state) << desc;
            });
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
            &StateMachine::onVisionPipelineFatalError,
            Qt::QueuedConnection);
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::bundleCaptureFinished,
            this,
            &StateMachine::onBundleCaptureFinished,
            Qt::QueuedConnection);
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
    if (m_orbbecGemini != nullptr) {
        disconnect(m_orbbecGemini, nullptr, this, nullptr);
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

void StateMachine::clearActiveTask()
{
    m_activeTask = {};
}

modbus::ModbusService* StateMachine::modbusService() const
{
    return m_modbus;
}

bool StateMachine::isModbusConnected() const
{
    return m_modbus != nullptr && m_modbus->isConnected();
}

orbbec_gemini::OrbbecGeminiService* StateMachine::orbbecGeminiService() const
{
    return m_orbbecGemini;
}

vision::VisionPipelineService* StateMachine::visionPipelineService() const
{
    return m_visionPipeline;
}

void StateMachine::setTaskProgress(quint16 progress)
{
    m_progress = progress;
}

void StateMachine::resetSafetyInterlockState()
{
    m_ipcSafetyActionWord = 0;
    m_personZoneAlarmActive = false;
}

void StateMachine::notifyLoadGraspFinished(quint16 resultCode, const PoseSourceResult& pose)
{
    emit loadGraspFinished(resultCode, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
}

void StateMachine::notifyUnloadCalcFinished(quint16 resultCode, const PoseSourceResult& pose)
{
    emit unloadCalcFinished(resultCode, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
}

void StateMachine::notifyPoseCheckFinished(
    bool success,
    quint16 resultCode,
    double poseDeviationMm,
    const QVector<double>& rt,
    const QString& message)
{
    emit poseCheckFinished(success, resultCode, poseDeviationMm, rt, message);
}

void StateMachine::notifySelfCheckFinished(quint16 resultCode, quint16 failWord0)
{
    emit selfCheckFinished(resultCode, failWord0);
}

void StateMachine::notifyCodeReadFinished(quint16 resultCode, const QString& codeValue)
{
    emit codeReadFinished(resultCode, codeValue);
}

void StateMachine::notifyResultResetFinished(quint16 resultCode)
{
    emit resultResetFinished(resultCode);
}

}  // namespace scan_tracking::flow_control
