/**
 * @file hmi_tcp_server.cpp
 * @brief HMI TCP 服务端实现
 */

#include "scan_tracking/hmi_server/hmi_tcp_server.h"
#include "scan_tracking/hmi_server/hmi_session.h"
#include "scan_tracking/hmi_server/hmi_protocol.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/tracking/tracking_service.h"  // InspectionMeasurement, appendInspectionMeasurementFields
#include "scan_tracking/common/config_manager.h"

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QtCore/QLoggingCategory>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>
#include <QtCore/QVector>
#include <QtCore/QJsonDocument>
#include <QtCore/QUuid>
#include <qdatetime.h>

namespace scan_tracking {
namespace hmi_server {

Q_LOGGING_CATEGORY(LOG_HMI_SERVER, "hmi.server")

// 静态成员初始化：全局日志拦截器回调所需的单例指针和原始日志处理器
HmiTcpServer* HmiTcpServer::s_instance = nullptr;
QtMessageHandler HmiTcpServer::s_previousHandler = nullptr;
bool HmiTcpServer::s_logForwarderInstalled = false;
namespace {

bool g_inTcpSend = false;
bool g_inLogForward = false;

bool isHighFrequencyTcpType(const QString& type)
{
    using namespace msg_type;
    return type == QLatin1String(kStatusSystem)
        || type == QLatin1String(kStatusPlc)
        || type == QLatin1String(kStatusCamera)
        || type == QLatin1String(kStatusDevice)
        || type == QLatin1String(kHeartbeatPing)
        || type == QLatin1String(kHeartbeatPong)
        || type == QLatin1String(kEventLog);
}

/// status.camera 单行摘要（仅在实际下发时打印，与显控 payload 变更去重一致）
QString summarizeCameraStatusPayload(const QJsonObject& payload)
{
    QStringList parts;

    const QJsonObject mechEye = payload.value(QLatin1String("mechEye")).toObject();
    if (!mechEye.isEmpty()) {
        parts << QStringLiteral("mechEye(state=%1,connected=%2)")
                     .arg(mechEye.value(QLatin1String("state")).toInt(-1))
                     .arg(mechEye.value(QLatin1String("connected")).toBool() ? 1 : 0);
    }

    const QJsonObject hikA = payload.value(QLatin1String("hikA")).toObject();
    if (!hikA.isEmpty()) {
        parts << QStringLiteral("hikA(connected=%1)")
                     .arg(hikA.value(QLatin1String("connected")).toBool() ? 1 : 0);
    }

    const QJsonObject hikB = payload.value(QLatin1String("hikB")).toObject();
    if (!hikB.isEmpty()) {
        parts << QStringLiteral("hikB(connected=%1)")
                     .arg(hikB.value(QLatin1String("connected")).toBool() ? 1 : 0);
    }

    const QJsonObject hikC = payload.value(QLatin1String("hikC")).toObject();
    if (!hikC.isEmpty()) {
        parts << QStringLiteral("hikC(connected=%1)")
                     .arg(hikC.value(QLatin1String("connected")).toBool() ? 1 : 0);
    }

    const QJsonObject pipeline = payload.value(QLatin1String("pipeline")).toObject();
    if (!pipeline.isEmpty()) {
        parts << QStringLiteral("pipeline(state=%1)")
                     .arg(pipeline.value(QLatin1String("state")).toInt(-1));
    }
    return parts.isEmpty() ? QStringLiteral("(empty)") : parts.join(QLatin1String(" "));
}

// TODO(hmi): 远程 event.log 转发默认关闭，优先保证 TCP 简洁与主业务打通。
// 若显控需要「远程日志页」，将此处改为 true，并确认 install/uninstall 成对调用。
constexpr bool kForwardQtLogsToHmi = false;

bool shouldForwardLogToHmi(QtMsgType type, const QString& category, const QString& msg)
{
    if (g_inTcpSend || g_inLogForward) {
        return false;
    }
    // HMI 模块日志仅本地可见，不包装成 event.log 发到显控
    if (category == QLatin1String("hmi.server") || category == QLatin1String("hmi.session")) {
        return false;
    }
    if (msg.contains(QLatin1String("[TCPIP]"))) {
        return false;
    }
    return type >= QtWarningMsg;
}

}  // namespace

HmiTcpServer::HmiTcpServer(int port, QObject* parent)
    : QObject(parent)
    , m_tcpServer(new QTcpServer(this))
    , m_port(port)
    , m_statusPushTimer(new QTimer(this))
    , m_heartbeatTimer(new QTimer(this))
{
    connect(m_tcpServer, &QTcpServer::newConnection, this, &HmiTcpServer::onNewConnection);

    m_statusPushTimer->setInterval(500); // 500ms 状态推送
    connect(m_statusPushTimer, &QTimer::timeout, this, &HmiTcpServer::onStatusPushTimer);

    m_heartbeatTimer->setInterval(2000); // 2000ms 心跳
    connect(m_heartbeatTimer, &QTimer::timeout, this, &HmiTcpServer::onHeartbeatTimer);
    
    // 初始化消息处理函数映射表
    initializeMessageHandlers();
}

HmiTcpServer::~HmiTcpServer()
{
    stop();
}

bool HmiTcpServer::start()
{
    if (m_tcpServer->isListening()) {
        return true;
    }

    if (!m_tcpServer->listen(QHostAddress::Any, m_port)) {
        qCritical(LOG_HMI_SERVER) << "HMI TCP 服务端启动失败，端口:" << m_port << "错误:" << m_tcpServer->errorString();
        return false;
    }

    qInfo(LOG_HMI_SERVER) << "HMI TCP 服务端启动成功，监听端口:" << m_port;

    // TODO(hmi): 需要把 IPC 侧 qWarning+ 推到显控 event.log 时，设 kForwardQtLogsToHmi=true
    if (kForwardQtLogsToHmi) {
        installLogForwarder();
    }

    return true;
}

void HmiTcpServer::stop()
{
    if (kForwardQtLogsToHmi) {
        uninstallLogForwarder(); // 卸载日志转发器
    }

    disconnectServiceSignals(); // 断开业务模块信号连接

    m_statusPushTimer->stop(); // 停止状态推送定时器
    m_heartbeatTimer->stop(); // 停止心跳定时器

    if (m_session) {
        m_session->disconnect(); // 断开客户端连接
        m_session->deleteLater();
        m_session = nullptr; // 释放客户端会话指针
    }

    if (m_tcpServer->isListening()) {
        m_tcpServer->close(); // 关闭 TCP 服务器
        qInfo(LOG_HMI_SERVER) << "HMI TCP 服务端已停止";
    }
}

bool HmiTcpServer::isListening() const
{
    return m_tcpServer->isListening(); // 检查 TCP 服务器是否正在监听
}

bool HmiTcpServer::hasClient() const
{
    return m_session != nullptr && m_session->isConnected();
}

void HmiTcpServer::setStateMachine(flow_control::StateMachine* sm) { m_stateMachine = sm; }
void HmiTcpServer::setModbusService(modbus::ModbusService* svc) { m_modbusService = svc; }

void HmiTcpServer::bindServiceSignals()
{
    if (m_serviceSignalsBound) {
        return;
    }
    connectStateMachineSignals();
    connectModbusSignals();
    connectMechEyeSignals();
    connectVisionPipelineSignals();
    connectStatusRefreshSignals();
    m_serviceSignalsBound = true;
}

void HmiTcpServer::disconnectServiceSignals()
{
    if (!m_serviceSignalsBound) {
        return;
    }
    if (m_stateMachine) {
        disconnect(m_stateMachine, nullptr, this, nullptr);
    }
    if (m_modbusService) {
        disconnect(m_modbusService, nullptr, this, nullptr);
    }
    if (m_mechEyeService) {
        disconnect(m_mechEyeService, nullptr, this, nullptr);
    }
    if (m_visionPipeline) {
        disconnect(m_visionPipeline, nullptr, this, nullptr);
    }
    m_serviceSignalsBound = false;
}
void HmiTcpServer::setMechEyeService(mech_eye::MechEyeService* svc) { m_mechEyeService = svc; }
void HmiTcpServer::setVisionPipelineService(vision::VisionPipelineService* svc) { m_visionPipeline = svc; }
void HmiTcpServer::setTrackingService(tracking::TrackingService* svc) { m_trackingService = svc; }
void HmiTcpServer::setHikCameraServices(vision::HikCameraService* hikA, vision::HikCameraService* hikB,
                                        vision::HikCameraService* hikC) {
    m_hikCameraA = hikA;
    m_hikCameraB = hikB;
    m_hikCameraC = hikC;
}

void HmiTcpServer::setHikCameraCController(vision::HikCameraCController* controller) {
    m_hikCameraCController = controller;
}

void HmiTcpServer::onNewConnection()
{
    while (m_tcpServer->hasPendingConnections()) {
        QTcpSocket* socket = m_tcpServer->nextPendingConnection();
        
        // 单客户端模式：如果已有连接，踢掉旧的或拒绝新的。这里策略是踢掉旧的。
        if (m_session) {
            qWarning(LOG_HMI_SERVER) << "新客户端接入，断开旧客户端会话";
            m_session->disconnect();
            m_session->deleteLater();
            m_session = nullptr;
        }

        m_session = new HmiSession(socket, this);
        qInfo(LOG_HMI_SERVER) << "[TCPIP] 新客户端已接入，IP:" << socket->peerAddress().toString() << "端口:" << socket->peerPort();
        
        connect(m_session, &HmiSession::messageReceived, this, &HmiTcpServer::onMessageReceived);
        connect(m_session, &HmiSession::disconnected, this, &HmiTcpServer::onSessionDisconnected);
        connect(m_session, &HmiSession::heartbeatTimeout, this, &HmiTcpServer::onSessionHeartbeatTimeout);

        invalidateStatusPushCache();
        m_statusPushTimer->start();
        m_heartbeatTimer->start();

        // 发送 core.hello 欢迎消息
        sendToClient(buildEnvelope(QStringLiteral("core.hello"), nextEventId(), QJsonObject()));
        pushAllStatusToClient();
    }
}

void HmiTcpServer::onSessionDisconnected()
{
    qInfo(LOG_HMI_SERVER) << "[TCPIP] 客户端连接已断开";
    if (m_session) {
        m_session->deleteLater();
        m_session = nullptr;
    }
    invalidateStatusPushCache();
    m_statusPushTimer->stop();
    m_heartbeatTimer->stop();
}

void HmiTcpServer::onSessionHeartbeatTimeout()
{
    if (m_session) {
        m_session->disconnect();
    }
}

// 初始化消息处理函数映射表
void HmiTcpServer::initializeMessageHandlers()
{
    // 连接管理消息
    m_messageHandlers[QString::fromLatin1(msg_type::kHmiHello)]       = &HmiTcpServer::handleHmiHello;  // 欢迎消息
    m_messageHandlers[QString::fromLatin1(msg_type::kHeartbeatPing)]  = &HmiTcpServer::handleHeartbeatPing;  // 心跳请求
    m_messageHandlers[QString::fromLatin1(msg_type::kHeartbeatPong)]  = &HmiTcpServer::handleHeartbeatPong;  // 心跳响应
    
    // 基础控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdStart)]       = &HmiTcpServer::handleCmdStart;  // 启动
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdStop)]        = &HmiTcpServer::handleCmdStop;  // 停止
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdReset)]       = &HmiTcpServer::handleCmdReset;  // 复位
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdClearAlarm)]  = &HmiTcpServer::handleCmdClearAlarm;  // 清除报警
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdGetStatus)]   = &HmiTcpServer::handleCmdGetStatus;  // 获取状态
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdGetConfig)]   = &HmiTcpServer::handleCmdGetConfig;  // 获取配置
    
    // Modbus 控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdModbusConnect)]    = &HmiTcpServer::handleCmdModbusConnect;  // 连接 Modbus
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdModbusDisconnect)] = &HmiTcpServer::handleCmdModbusDisconnect;  // 断开 Modbus
    
    // 相机控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdRefreshCamera)]    = &HmiTcpServer::handleCmdRefreshCamera;  // 刷新相机
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdCaptureMechEye)]   = &HmiTcpServer::handleCmdCaptureMechEye;  // 捕获 MechEye
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdCaptureBundle)]    = &HmiTcpServer::handleCmdCaptureBundle;  // 捕获 Bundle
    
    // 直接触发命令（占位实现）
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerScan)]         = &HmiTcpServer::handleCmdTriggerScan;  // 触发扫描
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerInspection)]   = &HmiTcpServer::handleCmdTriggerInspection;  // 触发综合检测
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerSelfCheck)]    = &HmiTcpServer::handleCmdTriggerSelfCheck;  // 触发自检
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerPoseCheck)]    = &HmiTcpServer::handleCmdTriggerPoseCheck;  // 触发位姿校验
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerCodeRead)]     = &HmiTcpServer::handleCmdTriggerCodeRead;  // 触发条码读取
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerResultReset)]  = &HmiTcpServer::handleCmdTriggerResultReset;  // 触发结果复位
}

// 处理接收到的客户端消息，根据 type 字段分发到不同的处理函数
void HmiTcpServer::onMessageReceived(const QJsonObject& message)
{
    // 1. 从 JSON 信封中解析出关键字段
    const QString type = message.value(QLatin1String("type")).toString();
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    // 本地单行摘要；完整 JSON 不再打印/转发，避免套娃与刷屏
    if (!isHighFrequencyTcpType(type)) {
        qDebug(LOG_HMI_SERVER).noquote()
            << "[TCPIP] RX" << type << msgId;
    }
    
    // ------------------------------------------------------------------
    // 2. 通信协议命令分发路由 (Dispatcher)
    // 使用 QHash 映射表进行 O(1) 时间复杂度的快速查找和分发。
    // 说明：所有 handleXxx 函数内部都会在执行完具体业务后，通过 sendResponse() 
    // 回传与该请求 msgId 完全一致的响应结果给显控界面，形成请求-响应的闭环。
    // ------------------------------------------------------------------
    auto it = m_messageHandlers.find(type);
    if (it != m_messageHandlers.end()) {
        // 找到对应的处理函数，调用成员函数指针
        (this->*it.value())(message);
    } else {
        // 未知消息类型，返回错误响应
        qWarning(LOG_HMI_SERVER) << "收到未知类型的消息:" << type;
        sendResponse(type, msgId, false, QStringLiteral("未知命令类型"));
    }
}

void HmiTcpServer::onStatusPushTimer()
{
    if (!hasClient()) return;
    pushSystemStatus();
    pushPlcStatus();
    pushCameraStatus(); 
    pushDeviceStatus();  // 周期性推送设备在线状态字和故障状态字
}

void HmiTcpServer::onHeartbeatTimer()
{
    if (!hasClient()) return;
    sendToClient(buildEnvelope(QLatin1String(msg_type::kHeartbeatPing), nextEventId(), QJsonObject()));
}

// --- 命令处理实现示例 ---

void HmiTcpServer::handleHmiHello(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kHmiHello), msgId, true, QStringLiteral("欢迎"));
}

void HmiTcpServer::handleHeartbeatPing(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    // 响应 pong
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kHeartbeatPong);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();    
    envelope[QStringLiteral("payload")]   = QJsonObject();
    sendToClient(envelope);
}

void HmiTcpServer::handleHeartbeatPong(const QJsonObject& message)
{
    Q_UNUSED(message)
}

void HmiTcpServer::handleCmdStart(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->start();
        sendResponse(QLatin1String(msg_type::kCmdStart), msgId, true, QStringLiteral("状态机已启动"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdStart), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdStop(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->stop();
        sendResponse(QLatin1String(msg_type::kCmdStop), msgId, true, QStringLiteral("状态机已停止"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdStop), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdGetStatus(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    QJsonObject payload = buildResponsePayload(true, QStringLiteral("状态已返回"));
    payload[QLatin1String("system")] = buildSystemStatusPayload();
    payload[QLatin1String("plc")] = buildPlcStatusPayload();
    payload[QLatin1String("camera")] = buildCameraStatusPayload();
    payload[QLatin1String("device")] = buildDeviceStatusPayload();
    sendToClient(buildEnvelope(QLatin1String(msg_type::kCmdGetStatus), msgId, payload));
    syncStatusPushCacheFromCurrent();
}

void HmiTcpServer::handleCmdReset(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->start(); // 目前系统使用 start 作为重启/复位的入口
        sendResponse(QLatin1String(msg_type::kCmdReset), msgId, true, QStringLiteral("状态机已复位"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdReset), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdClearAlarm(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->setAlarm(0, 0, QString());
        sendResponse(QLatin1String(msg_type::kCmdClearAlarm), msgId, true, QStringLiteral("报警已清除"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdClearAlarm), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdGetConfig(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    // 从 ConfigManager 单例读取所有配置并序列化为 JSON
    auto* cfgMgr = common::ConfigManager::instance();
    if (!cfgMgr) {
        sendResponse(QLatin1String(msg_type::kCmdGetConfig), msgId, false, QStringLiteral("配置管理器未初始化"));
        return;
    }
    
    QJsonObject configPayload;
    
    // 1. App 配置
    QJsonObject appObj;
    appObj[QLatin1String("version")] = cfgMgr->appConfig().version; // 版本号
    appObj[QLatin1String("environment")] = cfgMgr->appConfig().environment; // 环境
    configPayload[QLatin1String("app")] = appObj; 
    
    // 2. Logger 配置
    QJsonObject loggerObj;
    loggerObj[QLatin1String("level")] = cfgMgr->loggerConfig().level; // 日志级别
    loggerObj[QLatin1String("rotateDays")] = cfgMgr->loggerConfig().rotateDays; // 日志滚动天数
    configPayload[QLatin1String("logger")] = loggerObj;
    
    // 3. Modbus 配置
    QJsonObject modbusObj;
    modbusObj[QLatin1String("host")] = cfgMgr->modbusConfig().host; // 主机
    modbusObj[QLatin1String("port")] = cfgMgr->modbusConfig().port; // 端口
    modbusObj[QLatin1String("unitId")] = cfgMgr->modbusConfig().unitId; // 单元 ID
    modbusObj[QLatin1String("timeoutMs")] = cfgMgr->modbusConfig().timeoutMs; // 超时时间
    modbusObj[QLatin1String("reconnectIntervalMs")] = cfgMgr->modbusConfig().reconnectIntervalMs; // 重连间隔
    configPayload[QLatin1String("modbus")] = modbusObj;
    
    // 4. Camera 配置
    QJsonObject cameraObj;
    cameraObj[QLatin1String("defaultCamera")] = cfgMgr->cameraConfig().defaultCamera; // 默认相机
    cameraObj[QLatin1String("scanTimeoutMs")] = cfgMgr->cameraConfig().scanTimeoutMs; // 扫描超时时间
    configPayload[QLatin1String("camera")] = cameraObj;
    
    // 5. Vision 配置
    QJsonObject visionObj;
    visionObj[QLatin1String("mechEyeCameraKey")] = cfgMgr->visionConfig().mechEyeCameraKey; // MechEye 相机键
    visionObj[QLatin1String("mechCaptureTimeoutMs")] = cfgMgr->visionConfig().mechCaptureTimeoutMs; // MechEye 捕获超时时间
    visionObj[QLatin1String("hikConnectTimeoutMs")] = cfgMgr->visionConfig().hikConnectTimeoutMs; // Hik 连接超时时间
    visionObj[QLatin1String("hikCaptureTimeoutMs")] = cfgMgr->visionConfig().hikCaptureTimeoutMs; // Hik 捕获超时时间
    visionObj[QLatin1String("hikSdkRoot")] = cfgMgr->visionConfig().hikSdkRoot; // Hik SDK 根目录

    QJsonObject hikAObj;
    hikAObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCameraA.logicalName; // 逻辑名称
    hikAObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCameraA.cameraKey; // 相机键
    hikAObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCameraA.ipAddress; // IP 地址
    hikAObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCameraA.serialNumber; // 序列号
    visionObj[QLatin1String("hikCameraA")] = hikAObj;
    
    QJsonObject hikBObj;
    hikBObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCameraB.logicalName; // 逻辑名称
    hikBObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCameraB.cameraKey; // 相机键
    hikBObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCameraB.ipAddress; // IP 地址
    hikBObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCameraB.serialNumber; // 序列号
    visionObj[QLatin1String("hikCameraB")] = hikBObj;

    QJsonObject hikCObj;
    hikCObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCameraC.logicalName;
    hikCObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCameraC.cameraKey;
    hikCObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCameraC.ipAddress;
    hikCObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCameraC.serialNumber;
    hikCObj[QLatin1String("accessMode")] = cfgMgr->visionConfig().hikCameraC.accessMode;
    visionObj[QLatin1String("hikCameraC")] = hikCObj;
    
    configPayload[QLatin1String("vision")] = visionObj;
    
    // 6. FlowControl 配置
    QJsonObject flowControlObj;
    flowControlObj[QLatin1String("pollIntervalMs")] = cfgMgr->flowControlConfig().pollIntervalMs; // 轮询间隔
    flowControlObj[QLatin1String("heartbeatIntervalMs")] = cfgMgr->flowControlConfig().heartbeatIntervalMs; // 心跳间隔
    flowControlObj[QLatin1String("simulatedProcessingMs")] = cfgMgr->flowControlConfig().simulatedProcessingMs; // 模拟处理间隔
    configPayload[QLatin1String("flowControl")] = flowControlObj;
    
    // 7. Tracking 配置
    QJsonObject trackingObj;
    trackingObj[QLatin1String("firstStationOuterSegmentIndex")] = cfgMgr->trackingConfig().firstStationOuterSegmentIndex; // 外表面段号
    trackingObj[QLatin1String("firstStationInnerSegmentIndex")] = cfgMgr->trackingConfig().firstStationInnerSegmentIndex; // 内表面段号
    trackingObj[QLatin1String("firstStationHoleSegmentIndex")] = cfgMgr->trackingConfig().firstStationHoleSegmentIndex; // 开孔段号
    configPayload[QLatin1String("tracking")] = trackingObj;

    QJsonObject hmiObj;
    hmiObj[QLatin1String("enabled")] = cfgMgr->hmiConfig().enabled;
    hmiObj[QLatin1String("tcpPort")] = cfgMgr->hmiConfig().tcpPort;
    configPayload[QLatin1String("hmi")] = hmiObj;
    
    // 8. LbPose 配置
    QJsonObject lbPoseObj;
    lbPoseObj[QLatin1String("dataRoot")] = cfgMgr->lbPoseConfig().dataRoot; // 数据根目录
    lbPoseObj[QLatin1String("leftPattern")] = cfgMgr->lbPoseConfig().leftPattern; // 左模板文件
    lbPoseObj[QLatin1String("rightPattern")] = cfgMgr->lbPoseConfig().rightPattern; // 右模板文件
    lbPoseObj[QLatin1String("templateFile")] = cfgMgr->lbPoseConfig().templateFile; // 模板文件
    lbPoseObj[QLatin1String("minDistance")] = static_cast<double>(cfgMgr->lbPoseConfig().minDistance); // 最小距离
    lbPoseObj[QLatin1String("maxDistance")] = static_cast<double>(cfgMgr->lbPoseConfig().maxDistance); // 最大距离
    lbPoseObj[QLatin1String("cosTolerance")] = static_cast<double>(cfgMgr->lbPoseConfig().cosTolerance); // 余弦误差
    lbPoseObj[QLatin1String("minPercent")] = static_cast<double>(cfgMgr->lbPoseConfig().minPercent); // 最小百分比
    configPayload[QLatin1String("lbPose")] = lbPoseObj;
    
    // 构建响应
    QJsonObject payload = buildResponsePayload(true, QStringLiteral("配置已获取")); // 构建响应Payload
    payload[QLatin1String("config")] = configPayload;
    
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdGetConfig);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = payload;
    sendToClient(envelope);
}

void HmiTcpServer::handleCmdModbusConnect(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_modbusService) {
        m_modbusService->connectDevice();
        sendResponse(QLatin1String(msg_type::kCmdModbusConnect), msgId, true, QStringLiteral("正在连接 Modbus"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdModbusConnect), msgId, false, QStringLiteral("Modbus 服务不可用"));
    }
}

void HmiTcpServer::handleCmdModbusDisconnect(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_modbusService) {
        m_modbusService->disconnectDevice();
        sendResponse(QLatin1String(msg_type::kCmdModbusDisconnect), msgId, true, QStringLiteral("已断开 Modbus 连接"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdModbusDisconnect), msgId, false, QStringLiteral("Modbus 服务不可用"));
    }
}

void HmiTcpServer::handleCmdRefreshCamera(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    // 刷新 MechEye 3D 相机连接状态
    if (m_mechEyeService) {
        qInfo(LOG_HMI_SERVER) << "[TCPIP] 收到相机刷新请求，正在刷新 MechEye 相机状态...";
        m_mechEyeService->requestRefreshStatus();
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, true, QStringLiteral("相机刷新请求已发送"));
    } else {
        qWarning(LOG_HMI_SERVER) << "[TCPIP] 相机刷新失败：MechEye 服务不可用";
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, false, QStringLiteral("相机服务不可用"));
    }
}

void HmiTcpServer::handleCmdTriggerScan(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerScan), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC 或状态机"));
}

void HmiTcpServer::handleCmdTriggerInspection(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerInspection), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerSelfCheck(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerSelfCheck), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerPoseCheck(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerPoseCheck), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerCodeRead(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerCodeRead), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerResultReset(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerResultReset), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdCaptureMechEye(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    const QString cameraKey = message.value(QLatin1String("payload")).toObject().value(QLatin1String("cameraKey")).toString();
    if (m_mechEyeService) {
        quint64 reqId = m_mechEyeService->requestCapture(cameraKey, mech_eye::CaptureMode::Capture3DOnly);
        QJsonObject payload = buildResponsePayload(true, QStringLiteral("采集请求已发送"));
        payload[QLatin1String("requestId")] = static_cast<qint64>(reqId);
        
        QJsonObject envelope;
        envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
        envelope[QStringLiteral("msgId")]     = msgId;
        envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdCaptureMechEye);
        envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        envelope[QStringLiteral("payload")]   = payload;
        sendToClient(envelope);
    } else {
        sendResponse(QLatin1String(msg_type::kCmdCaptureMechEye), msgId, false, QStringLiteral("相机服务不可用"));
    }
}

void HmiTcpServer::handleCmdCaptureBundle(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_visionPipeline) {
        const QJsonObject payloadObj = message.value(QLatin1String("payload")).toObject();
        const int segmentIndex = payloadObj.value(QLatin1String("segmentIndex")).toInt(1);
        const quint32 taskId = static_cast<quint32>(payloadObj.value(QLatin1String("taskId")).toInt(0));
        bool needMechEye2D = payloadObj.value(QLatin1String("needMechEye2D")).toBool(false);
        if (!payloadObj.contains(QLatin1String("needMechEye2D"))) {
            const auto* configMgr = scan_tracking::common::ConfigManager::instance();
            if (configMgr != nullptr) {
                for (const auto& path : configMgr->scanPathsConfig().scanPaths) {
                    if (!path.enabled) {
                        continue;
                    }
                    for (const auto& point : path.points) {
                        if (point.pointIndex == segmentIndex) {
                            needMechEye2D = point.needRotation;
                            break;
                        }
                    }
                    if (needMechEye2D) {
                        break;
                    }
                }
            }
        }
        quint64 reqId = m_visionPipeline->requestCaptureBundle(segmentIndex, taskId, needMechEye2D);
        
        QJsonObject payload = buildResponsePayload(true, QStringLiteral("组合采集请求已发送"));
        payload[QLatin1String("requestId")] = static_cast<qint64>(reqId);
        
        QJsonObject envelope;
        envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
        envelope[QStringLiteral("msgId")]     = msgId;
        envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdCaptureBundle);
        envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        envelope[QStringLiteral("payload")]   = payload;
        sendToClient(envelope);
    } else {
        sendResponse(QLatin1String(msg_type::kCmdCaptureBundle), msgId, false, QStringLiteral("视觉流水线不可用"));
    }
}

// --- 状态推送实现 ---

void HmiTcpServer::invalidateStatusPushCache()
{
    m_systemStatusCache = {};
    m_plcStatusCache = {};
    m_cameraStatusCache = {};
    m_deviceStatusCache = {};
}

bool HmiTcpServer::pushStatusIfChanged(const QString& type, const QJsonObject& payload,
                                       HmiStatusPushCache& slot, bool forcePush)
{
    if (!forcePush && slot.isValid && payload == slot.payload) {
        return false;
    }
    slot.payload = payload;
    slot.isValid = true;
    sendToClient(buildEnvelope(type, nextEventId(), payload));
    if (hasClient() && type == QLatin1String(msg_type::kStatusCamera)) {
        qInfo(LOG_HMI_SERVER).noquote()
            << "[TCPIP] status.camera TX |" << summarizeCameraStatusPayload(payload);
    }
    return true;
}

void HmiTcpServer::pushAllStatusToClient()
{
    if (!hasClient()) {
        return;
    }
    pushStatusIfChanged(QLatin1String(msg_type::kStatusSystem), buildSystemStatusPayload(),
                        m_systemStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusPlc), buildPlcStatusPayload(),
                        m_plcStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusCamera), buildCameraStatusPayload(),
                        m_cameraStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusDevice), buildDeviceStatusPayload(),
                        m_deviceStatusCache, true);
}

void HmiTcpServer::syncStatusPushCacheFromCurrent()
{
    if (m_stateMachine) {
        m_systemStatusCache.payload = buildSystemStatusPayload();
        m_systemStatusCache.isValid = true;
    }
    if (m_modbusService) {
        m_plcStatusCache.payload = buildPlcStatusPayload();
        m_plcStatusCache.isValid = true;
    }
    m_cameraStatusCache.payload = buildCameraStatusPayload();
    m_cameraStatusCache.isValid = true;
    m_deviceStatusCache.payload = buildDeviceStatusPayload();
    m_deviceStatusCache.isValid = true;
}

void HmiTcpServer::pushSystemStatus()
{
    if (!m_stateMachine) return;
    const QJsonObject payload = buildSystemStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusSystem), payload, m_systemStatusCache);
}

QJsonObject HmiTcpServer::buildSystemStatusPayload() const
{
    QJsonObject payload;
    if (!m_stateMachine) return payload;

    QString appStateStr;
    switch (m_stateMachine->currentState()) {
    case flow_control::AppState::Init:     appStateStr = QStringLiteral("Init"); break;
    case flow_control::AppState::Ready:    appStateStr = QStringLiteral("Ready"); break;
    case flow_control::AppState::Scanning: appStateStr = QStringLiteral("Scanning"); break;
    case flow_control::AppState::Error:    appStateStr = QStringLiteral("Error"); break;
    default:                               appStateStr = QStringLiteral("Unknown"); break;
    }

    payload[QLatin1String("ipcState")] = static_cast<int>(m_stateMachine->ipcState());
    payload[QLatin1String("appState")] = appStateStr;
    payload[QLatin1String("stage")] = static_cast<int>(m_stateMachine->currentStage());
    payload[QLatin1String("alarmLevel")] = m_stateMachine->alarmLevel();
    payload[QLatin1String("alarmCode")] = m_stateMachine->alarmCode();
    payload[QLatin1String("warnCode")] = m_stateMachine->warnCode();
    payload[QLatin1String("ipcReady")] = (m_stateMachine->currentState() == flow_control::AppState::Ready) ? 1 : 0;
    payload[QLatin1String("progress")] = m_stateMachine->progress();
    return payload;
}

void HmiTcpServer::pushPlcStatus()
{
    const QJsonObject payload = buildPlcStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusPlc), payload, m_plcStatusCache);
}

QJsonObject HmiTcpServer::buildPlcStatusPayload() const
{
    QJsonObject payload;
    payload[QLatin1String("modbusConnected")] = m_modbusService && m_modbusService->isConnected();
    if (!m_modbusService) {
        return payload;
    }

    if (m_stateMachine) {
        namespace regs = flow_control::protocol::registers;
        const auto& cb = m_stateMachine->lastCommandBlock();
        if (cb.size() > regs::kScanSegmentIndexLow) {
            payload[QLatin1String("plcHeartbeat")]    = cb.value(regs::kPlcHeartbeat);
            payload[QLatin1String("plcSystemState")]  = cb.value(regs::kPlcSystemState);
            payload[QLatin1String("workMode")]        = cb.value(regs::kStationWorkMode);
            payload[QLatin1String("flowEnable")]      = cb.value(regs::kFlowEnable);
            payload[QLatin1String("safetyWord")]      = cb.value(regs::kSafetyStatusWord);
            payload[QLatin1String("taskId")]          = static_cast<int>(
                (static_cast<quint32>(cb.value(regs::kTaskIdHigh)) << 16) |
                static_cast<quint32>(cb.value(regs::kTaskIdLow)));
            payload[QLatin1String("productType")]     = cb.value(regs::kProductType);
            payload[QLatin1String("recipeId")]        = cb.value(regs::kRecipeId);
            // ScanSegmentIndex 是 32 位（offset 14-15 合并）
            const quint16 segHigh = cb.value(regs::kScanSegmentIndex);
            const quint16 segLow = cb.value(regs::kScanSegmentIndexLow);
            payload[QLatin1String("scanSegmentIndex")] = static_cast<int>((static_cast<quint32>(segHigh) << 16) | segLow);
            // scanSegmentTotal 从配置获取，不再从 PLC 读取
            const auto* cfgMgr = scan_tracking::common::ConfigManager::instance();
            payload[QLatin1String("scanSegmentTotal")] = cfgMgr ? cfgMgr->trackingConfig().scanSegmentTotal : 0;
        }
    }
    return payload;
}

void HmiTcpServer::pushCameraStatus()
{
    const QJsonObject payload = buildCameraStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusCamera), payload, m_cameraStatusCache);
}

QJsonObject HmiTcpServer::buildCameraStatusPayload() const
{
    QJsonObject payload;
    
    if (m_mechEyeService) {
        QJsonObject mechEyeObj;
        mechEyeObj[QLatin1String("state")] = static_cast<int>(m_mechEyeService->state());
        mechEyeObj[QLatin1String("connected")] = (m_mechEyeService->state() != mech_eye::CameraRuntimeState::Idle
                                                   && m_mechEyeService->state() != mech_eye::CameraRuntimeState::Error);
        payload[QLatin1String("mechEye")] = mechEyeObj;
    }
    
    if (m_hikCameraA) {
        QJsonObject hikAObj;
        hikAObj[QLatin1String("roleName")] = m_hikCameraA->roleName();
        hikAObj[QLatin1String("connected")] = m_hikCameraA->isConnected();
        payload[QLatin1String("hikA")] = hikAObj;
    }
    
    if (m_hikCameraB) {
        QJsonObject hikBObj;
        hikBObj[QLatin1String("roleName")] = m_hikCameraB->roleName();
        hikBObj[QLatin1String("connected")] = m_hikCameraB->isConnected();
        payload[QLatin1String("hikB")] = hikBObj;
    }

    if (m_hikCameraC) {
        QJsonObject hikCObj;
        hikCObj[QLatin1String("roleName")] = m_hikCameraC->roleName();
        hikCObj[QLatin1String("connected")] = hikCameraCConnected();
        payload[QLatin1String("hikC")] = hikCObj;
    }
    
    if (m_visionPipeline) {
        QJsonObject pipelineObj;
        pipelineObj[QLatin1String("state")] = static_cast<int>(m_visionPipeline->state());
        payload[QLatin1String("pipeline")] = pipelineObj;
    }
    return payload;
}

bool HmiTcpServer::hikCameraCConnected() const
{
    if (m_hikCameraCController && m_hikCameraCController->isStarted()) {
        if (m_hikCameraCController->isCameraConnectedToTcp()) {
            return true;
        }
    }
    return m_hikCameraC && m_hikCameraC->isConnected();
}

void HmiTcpServer::pushDeviceStatus()
{
    const QJsonObject payload = buildDeviceStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusDevice), payload, m_deviceStatusCache);
}

QJsonObject HmiTcpServer::buildDeviceStatusPayload() const
{
    // onlineWord0 / faultWord0 位定义与 docs/封头检测工位_TCP_IP显控通信协议_v1.0.md §2.4 一致
    constexpr int kBitIpcCore = 0;
    constexpr int kBitHmiClient = 1;
    constexpr int kBitMechEye = 2;
    constexpr int kBitVisionPipeline = 3;
    constexpr int kBitHik2d = 4;
    constexpr int kBitTracking = 5;
    constexpr int kBitModbus = 6;
    constexpr int kBitAlarmWarn = 7;

    quint16 onlineWord0 = 0;
    quint16 faultWord0 = 0;

    onlineWord0 |= (1u << kBitIpcCore);

    if (hasClient()) {
        onlineWord0 |= (1u << kBitHmiClient);
    }

    const bool mechEyeOnline = m_mechEyeService
        && m_mechEyeService->state() != mech_eye::CameraRuntimeState::Idle
        && m_mechEyeService->state() != mech_eye::CameraRuntimeState::Error;
    if (mechEyeOnline) {
        onlineWord0 |= (1u << kBitMechEye);
    }
    if (m_mechEyeService && m_mechEyeService->state() == mech_eye::CameraRuntimeState::Error) {
        faultWord0 |= (1u << kBitMechEye);
    }

    if (m_visionPipeline && m_visionPipeline->state() == vision::VisionPipelineState::Ready) {
        onlineWord0 |= (1u << kBitVisionPipeline);
    }
    if (m_visionPipeline && m_visionPipeline->state() == vision::VisionPipelineState::Error) {
        faultWord0 |= (1u << kBitVisionPipeline);
    }

    const bool hikAOnline = m_hikCameraA && m_hikCameraA->isConnected();
    const bool hikBOnline = m_hikCameraB && m_hikCameraB->isConnected();
    const bool hikCOnline = hikCameraCConnected();
    const bool hasAnyHikService = m_hikCameraA || m_hikCameraB || m_hikCameraC;
    if (hikAOnline || hikBOnline || hikCOnline) {
        onlineWord0 |= (1u << kBitHik2d);
    }
    if (hasAnyHikService && !hikAOnline && !hikBOnline && !hikCOnline) {
        faultWord0 |= (1u << kBitHik2d);
    }

    if (m_trackingService) {
        onlineWord0 |= (1u << kBitTracking);
    }

    const bool modbusOnline = m_modbusService && m_modbusService->isConnected();
    if (modbusOnline) {
        onlineWord0 |= (1u << kBitModbus);
    }
    if (m_modbusService && !modbusOnline) {
        faultWord0 |= (1u << kBitModbus);
    }

    if (m_stateMachine) {
        const auto ipcState = m_stateMachine->ipcState();
        const auto appState = m_stateMachine->currentState();
        const quint16 alarmLevel = m_stateMachine->alarmLevel();

        if (ipcState == flow_control::protocol::IpcState::Fault
            || appState == flow_control::AppState::Error) {
            faultWord0 |= (1u << kBitIpcCore);
        }
        if (alarmLevel >= 3) {
            faultWord0 |= (1u << kBitIpcCore);
        }
        if (alarmLevel >= 2) {
            faultWord0 |= (1u << kBitAlarmWarn);
        }
    }

    QJsonObject payload;
    payload[QLatin1String("onlineWord0")] = onlineWord0;
    payload[QLatin1String("faultWord0")] = faultWord0;
    return payload;
}

// --- 事件连接 ---

void HmiTcpServer::connectStateMachineSignals()
{
    if (!m_stateMachine) {
        return;
    }

    qRegisterMetaType<QVector<double>>("QVector<double>");

    // 绑定核心业务报警事件：将状态机发出的协议级错误拦截并封装为 event.alarm 向远端推送
    const bool okProtocol = connect(m_stateMachine, &flow_control::StateMachine::protocolEvent, this,
        [this](const QString& message) {
        QJsonObject payload;
        payload[QLatin1String("message")] = message;
        payload[QLatin1String("level")] = m_stateMachine->alarmLevel();
        payload[QLatin1String("code")] = m_stateMachine->alarmCode();
        payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    }, Qt::UniqueConnection);
    if (!okProtocol) {
        qCritical(LOG_HMI_SERVER) << "connect protocolEvent 失败，请完整重编 flow_control 与 hmi_server";
    }

    // 绑定扫描分段启动事件：告知显控界面哪一段扫描正在拍摄
    connect(m_stateMachine, &flow_control::StateMachine::scanStarted, this,
        [this](int segmentIndex, quint32 taskId) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = segmentIndex;
        payload[QLatin1String("taskId")] = static_cast<int>(taskId);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventScanStarted), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 绑定扫描分段完成事件：推送当前段是否采图成功及有效帧数
    connect(m_stateMachine, &flow_control::StateMachine::scanFinished, this,
        [this](int segmentIndex, quint16 resultCode, int imageCount, int cloudFrameCount) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = segmentIndex;
        payload[QLatin1String("resultCode")] = resultCode; // 1 表示成功
        payload[QLatin1String("imageCount")] = imageCount;
        payload[QLatin1String("cloudFrameCount")] = cloudFrameCount;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventScanFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 绑定综合检测算法结束事件：推送所有工艺偏移量计算结果及缺陷诊断信息（如划痕、裂纹等特征字）
    connect(m_stateMachine, &flow_control::StateMachine::inspectionFinished, this,
        [this](quint16 resultCode, quint16 ngReasonWord0, quint16 ngReasonWord1,
               quint16 measureItemCount, float offsetXmm, float offsetYmm, float offsetZmm,
               float stableOffsetXmm, float stableOffsetYmm, float stableOffsetZmm,
               const tracking::InspectionMeasurement& measurement,
               const QString& outlinerErrorLog, const QString& inlinerErrorLog,
               const QString& message) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("ngReasonWord0")] = ngReasonWord0;    // 主要 NG 原因位图
        payload[QLatin1String("ngReasonWord1")] = ngReasonWord1;    // 辅助 NG 原因位图
        payload[QLatin1String("measureItemCount")] = measureItemCount;
        payload[QLatin1String("offsetXmm")] = static_cast<double>(offsetXmm); // 相对基础模板的绝对偏移量
        payload[QLatin1String("offsetYmm")] = static_cast<double>(offsetYmm);
        payload[QLatin1String("offsetZmm")] = static_cast<double>(offsetZmm);
        payload[QLatin1String("stableOffsetXmm")] = static_cast<double>(stableOffsetXmm); // 平滑滤波后的偏移量
        payload[QLatin1String("stableOffsetYmm")] = static_cast<double>(stableOffsetYmm);
        payload[QLatin1String("stableOffsetZmm")] = static_cast<double>(stableOffsetZmm);
        tracking::appendInspectionMeasurementFields(payload, measurement);
        payload[QLatin1String("outlinerErrorLog")] = outlinerErrorLog; // 外部轮廓瑕疵诊断日志
        payload[QLatin1String("inlinerErrorLog")] = inlinerErrorLog;   // 内部孔径瑕疵诊断日志
        payload[QLatin1String("message")] = message;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventInspectionFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 位姿校验完成
    connect(m_stateMachine, &flow_control::StateMachine::poseCheckFinished, this,
        [this](bool success, quint16 resultCode, double poseDeviationMm, const QVector<double>& rt, const QString& message) {
        QJsonObject payload;
        QJsonArray rtArray;
        for (double value : rt) {
            rtArray.append(value);
        }
        payload[QLatin1String("success")] = success;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("poseDeviationMm")] = poseDeviationMm;
        payload[QLatin1String("rt")] = rtArray;
        payload[QLatin1String("message")] = message;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventPoseCheckFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 上料抓取完成
    connect(m_stateMachine, &flow_control::StateMachine::loadGraspFinished, this,
        [this](quint16 resultCode, float x, float y, float z, float rx, float ry, float rz) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("x")] = static_cast<double>(x);
        payload[QLatin1String("y")] = static_cast<double>(y);
        payload[QLatin1String("z")] = static_cast<double>(z);
        payload[QLatin1String("rx")] = static_cast<double>(rx);
        payload[QLatin1String("ry")] = static_cast<double>(ry);
        payload[QLatin1String("rz")] = static_cast<double>(rz);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventLoadGraspFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 卸料计算完成
    connect(m_stateMachine, &flow_control::StateMachine::unloadCalcFinished, this,
        [this](quint16 resultCode, float x, float y, float z, float rx, float ry, float rz) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("x")] = static_cast<double>(x);
        payload[QLatin1String("y")] = static_cast<double>(y);
        payload[QLatin1String("z")] = static_cast<double>(z);
        payload[QLatin1String("rx")] = static_cast<double>(rx);
        payload[QLatin1String("ry")] = static_cast<double>(ry);
        payload[QLatin1String("rz")] = static_cast<double>(rz);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventUnloadCalcFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 自检完成
    connect(m_stateMachine, &flow_control::StateMachine::selfCheckFinished, this,
        [this](quint16 resultCode, quint16 failWord0) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("failWord0")] = failWord0;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventSelfCheckFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 条码读取完成
    connect(m_stateMachine, &flow_control::StateMachine::codeReadFinished, this,
        [this](quint16 resultCode, const QString& codeValue) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("codeValue")] = codeValue;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventCodeReadFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 结果复位完成
    connect(m_stateMachine, &flow_control::StateMachine::resultResetFinished, this,
        [this](quint16 resultCode) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventResultResetFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);
}

void HmiTcpServer::connectModbusSignals()
{
    if (!m_modbusService) return;

    connect(m_modbusService, &modbus::ModbusService::connected, this, [this]() {
        QJsonObject payload;
        payload[QLatin1String("message")] = QStringLiteral("Modbus connected");
        payload[QLatin1String("level")] = 0;
        payload[QLatin1String("code")] = 0;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });

    connect(m_modbusService, &modbus::ModbusService::disconnected, this, [this]() {
        QJsonObject payload;
        payload[QLatin1String("message")] = QStringLiteral("Modbus disconnected");
        payload[QLatin1String("level")] = 3;
        payload[QLatin1String("code")] = 900;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });

    connect(m_modbusService, &modbus::ModbusService::errorOccurred, this, [this](const QString& errorString) {
        QJsonObject payload;
        payload[QLatin1String("message")] = errorString;
        payload[QLatin1String("level")] = 2;
        payload[QLatin1String("code")] = 901;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });
}

void HmiTcpServer::connectMechEyeSignals()
{
    if (!m_mechEyeService) return;

    connect(m_mechEyeService, &mech_eye::MechEyeService::captureFinished, this,
        [this](scan_tracking::mech_eye::CaptureResult result) {
        QJsonObject payload;
        payload[QLatin1String("requestId")] = static_cast<qint64>(result.requestId);
        payload[QLatin1String("cameraKey")] = result.cameraKey;
        payload[QLatin1String("pointCount")] = static_cast<int>(result.pointCloud.pointCount);
        payload[QLatin1String("width")] = result.pointCloud.width;
        payload[QLatin1String("height")] = result.pointCloud.height;
        payload[QLatin1String("elapsedMs")] = static_cast<int>(result.elapsedMs);
        payload[QLatin1String("errorCode")] = static_cast<int>(result.errorCode);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventImageCaptured), nextEventId(), payload));
    });

    connect(m_mechEyeService, &mech_eye::MechEyeService::fatalError, this,
        [this](scan_tracking::mech_eye::CaptureErrorCode code, QString message) {
        QJsonObject payload;
        payload[QLatin1String("message")] = message;
        payload[QLatin1String("level")] = 3;
        payload[QLatin1String("code")] = static_cast<int>(code);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
        if (hasClient()) {
            pushDeviceStatus();
        }
    });
}

void HmiTcpServer::connectStatusRefreshSignals()
{
    if (m_stateMachine) {
        connect(m_stateMachine, &flow_control::StateMachine::stateChanged, this, [this]() {
            if (hasClient()) {
                pushSystemStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    if (m_modbusService) {
        connect(m_modbusService, &modbus::ModbusService::connected, this, [this]() {
            if (hasClient()) {
                pushPlcStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
        connect(m_modbusService, &modbus::ModbusService::disconnected, this, [this]() {
            if (hasClient()) {
                pushPlcStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    if (m_mechEyeService) {
        connect(m_mechEyeService, &mech_eye::MechEyeService::stateChanged, this,
                [this](mech_eye::CameraRuntimeState, QString) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    if (m_visionPipeline) {
        connect(m_visionPipeline, &vision::VisionPipelineService::stateChanged, this,
                [this](vision::VisionPipelineState, const QString&) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    auto refreshCameraOnHik = [this](QString, QString, QString) {
        if (hasClient()) {
            pushCameraStatus();
            pushDeviceStatus();
        }
    };
    if (m_hikCameraA) {
        connect(m_hikCameraA, &vision::HikCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }
    if (m_hikCameraB) {
        connect(m_hikCameraB, &vision::HikCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }
    if (m_hikCameraC) {
        connect(m_hikCameraC, &vision::HikCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }

    if (m_hikCameraCController) {
        connect(m_hikCameraCController, &vision::HikCameraCController::stateChanged, this,
                [this](vision::HikCameraCState, const QString&) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }
}

void HmiTcpServer::connectVisionPipelineSignals()
{
    if (!m_visionPipeline) return;

    connect(m_visionPipeline, &vision::VisionPipelineService::bundleCaptureFinished, this,
        [this](scan_tracking::vision::MultiCameraCaptureBundle bundle) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = bundle.request.segmentIndex;
        payload[QLatin1String("taskId")] = static_cast<int>(bundle.request.taskId);
        payload[QLatin1String("mechOk")] = bundle.mechEyeResult.success();
        payload[QLatin1String("hikAOk")] = bundle.hikCameraAResult.success();
        payload[QLatin1String("hikBOk")] = bundle.hikCameraBResult.success();
        payload[QLatin1String("lbOk")] = bundle.lbPoseResult.success;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventBundleCaptured), nextEventId(), payload));
    });
}

// --- 日志转发实现（默认关闭，见 kForwardQtLogsToHmi）---

void HmiTcpServer::installLogForwarder()
{
    if (s_logForwarderInstalled) {
        return;
    }

    // 保存当前实例指针，供全局回调函数使用
    s_instance = this;
    
    // 拦截全局 Qt 日志输出，并保存原有处理器以支持链式调用
    s_previousHandler = qInstallMessageHandler([](QtMsgType type, const QMessageLogContext& context, const QString& msg) {
        const QString category = QString::fromLatin1(context.category ? context.category : "default");

        if (s_instance && s_instance->hasClient() && shouldForwardLogToHmi(type, category, msg)) {
            QJsonObject payload;
            int severity = 2;
            switch (type) {
            case QtWarningMsg: severity = 2; break;
            case QtCriticalMsg: severity = 3; break;
            case QtFatalMsg: severity = 4; break;
            default: break;
            }
            payload[QLatin1String("severity")] = severity;
            payload[QLatin1String("category")] = category;
            payload[QLatin1String("message")] = msg;
            payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();

            g_inLogForward = true;
            s_instance->sendToClient(buildEnvelope(QLatin1String(msg_type::kEventLog), s_instance->nextEventId(), payload));
            g_inLogForward = false;
        }

        if (s_previousHandler) {
            s_previousHandler(type, context, msg);
        }
    });
    s_logForwarderInstalled = true;
}

void HmiTcpServer::uninstallLogForwarder()
{
    if (!s_logForwarderInstalled) {
        return;
    }
    if (s_previousHandler) {
        qInstallMessageHandler(s_previousHandler);
        s_previousHandler = nullptr;
    }
    s_instance = nullptr;
    s_logForwarderInstalled = false;
}

// --- 辅助发送 ---

void HmiTcpServer::sendToClient(const QJsonObject& envelope)
{
    if (!m_session) {
        return;
    }

    const QString type = envelope.value(QLatin1String("type")).toString();
    const QString msgId = envelope.value(QLatin1String("msgId")).toString();

    g_inTcpSend = true;
    if (!isHighFrequencyTcpType(type)) {
        qDebug(LOG_HMI_SERVER).noquote() << "[TCPIP] TX" << type << msgId;
    }
    m_session->sendMessage(envelope);
    g_inTcpSend = false;
}

void HmiTcpServer::sendResponse(const QString& type, const QString& msgId, bool success, const QString& message)
{
    QJsonObject payload = buildResponsePayload(success, message);
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = type;
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = payload;
    sendToClient(envelope);
}

QString HmiTcpServer::nextEventId()
{
    return QStringLiteral("evt-%1").arg(m_nextEventId++);
}

}  // namespace hmi_server
}  // namespace scan_tracking
