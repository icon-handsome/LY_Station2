#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"
#include "scan_tracking/vision/hik_mono_io.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QEventLoop>
#include <QtCore/QPointer>
#include <QtCore/QStringList>
#include <QtCore/QLoggingCategory>
#include <QtCore/QElapsedTimer>
#include <QtCore/QThread>
#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QTextStream>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
#include <qdebug.h>
namespace scan_tracking::flow_control {

// 定义流程控制模块的日志分类
Q_LOGGING_CATEGORY(LOG_FLOW, "flow_control")

namespace {

/**
 * @brief 设备在线状态字位掩码
 * 
 * 用于标识 IPC 设备在线状态的各个比特位，包括：
 * - Bit 0: 系统就绪
 * - Bit 1: 通信正常
 * - Bit 2: 传感器在线
 * - Bit 4: 相机连接
 * - Bit 5: 跟踪服务
 * - Bit 6: Modbus 连接
 */
constexpr quint16 kDeviceOnlineWord0 =
    (1u << 0) |
    (1u << 1) |
    (1u << 2) |
    (1u << 4) |
    (1u << 5) |
    (1u << 6);

/// 最大扫描分段索引（从1开始计数）
constexpr int kMaxScanSegmentIndex = 16;

/// 默认扫描分段采集超时时间（毫秒）
constexpr int kDefaultScanSegmentCaptureTimeoutMs = 30000;

/// 允许的最大连续 Modbus 通信失败次数，超过此值将进入故障状态
constexpr int kMaxConsecutiveModbusFailures = 3;
constexpr int kPollLogEveryN = 20;
constexpr int kBackgroundRefinementJoinTimeoutMs = 300000;

/// PLC `Res_Inspection`：1=OK，6=超时 NG（与 Modbus 协议一致）
constexpr quint16 kInspectionResOk = 1;
constexpr quint16 kInspectionResTimeoutNg = 6;

// TODO(field-commissioning): 现场联调临时策略——仅超时向 PLC 报 NG（Res_Inspection=6）；
// 算法 NG、缺段、Tracking 不可用等其它情况一律回 Res=1(OK)。日志/HMI 仍保留真实结果码。
// 联调结束、恢复按蓝友/业务判定写 Res 后，删除本函数并在 executeInspectionTask 中直传 actualResultCode。
quint16 inspectionResForPlcHandshake(quint16 actualResultCode)
{
    if (actualResultCode == kInspectionResTimeoutNg) {
        return kInspectionResTimeoutNg;
    }
    return kInspectionResOk;
}

int countHikImagesInBundle(const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    int imageCount = 0;
    if (bundle.hikCameraAResult.success() && bundle.hikCameraAResult.frame.isValid()) {
        ++imageCount;
    }
    if (bundle.hikCameraBResult.success() && bundle.hikCameraBResult.frame.isValid()) {
        ++imageCount;
    }
    return imageCount;
}

scan_tracking::mech_eye::PointCloudFrame clonePointCloudFrame(
    const scan_tracking::mech_eye::PointCloudFrame& src)
{
    scan_tracking::mech_eye::PointCloudFrame dst = src;
    if (src.pointsXYZ) {
        dst.pointsXYZ = std::make_shared<std::vector<float>>(*src.pointsXYZ);
    }
    if (src.normalsXYZ) {
        dst.normalsXYZ = std::make_shared<std::vector<float>>(*src.normalsXYZ);
    }
    return dst;
}

scan_tracking::mech_eye::CaptureResult cloneCaptureResult(
    const scan_tracking::mech_eye::CaptureResult& src)
{
    scan_tracking::mech_eye::CaptureResult dst = src;
    dst.pointCloud = clonePointCloudFrame(src.pointCloud);
    if (src.texture2D.isValid()) {
        dst.texture2D.pixels =
            std::make_shared<std::vector<uint8_t>>(*src.texture2D.pixels);
    }
    return dst;
}

scan_tracking::vision::HikMonoFrame cloneHikMonoFrame(
    const scan_tracking::vision::HikMonoFrame& src)
{
    scan_tracking::vision::HikMonoFrame dst = src;
    if (src.pixels) {
        dst.pixels = std::make_shared<std::vector<std::uint8_t>>(*src.pixels);
    }
    return dst;
}

scan_tracking::vision::MultiCameraCaptureBundle cloneCaptureBundle(
    const scan_tracking::vision::MultiCameraCaptureBundle& src)
{
    scan_tracking::vision::MultiCameraCaptureBundle dst = src;
    dst.mechEyeResult = cloneCaptureResult(src.mechEyeResult);
    dst.hikCameraAResult.frame = cloneHikMonoFrame(src.hikCameraAResult.frame);
    dst.hikCameraBResult.frame = cloneHikMonoFrame(src.hikCameraBResult.frame);
    return dst;
}

std::array<float, 16> identityMatrix4x4()
{
    std::array<float, 16> m{};
    m[0] = m[5] = m[10] = m[15] = 1.0f;
    return m;
}

/** 行优先 4×4：out = left × right */
std::array<float, 16> multiplyRowMajor4x4(
    const std::array<float, 16>& left,
    const std::array<float, 16>& right)
{
    std::array<float, 16> out{};
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += left[static_cast<std::size_t>(row * 4 + k)] *
                       right[static_cast<std::size_t>(k * 4 + col)];
            }
            out[static_cast<std::size_t>(row * 4 + col)] = sum;
        }
    }
    return out;
}

std::array<float, 16> poseMatrixToArray(const scan_tracking::vision::PoseMatrix4x4& pose)
{
    return pose.isValid() ? pose.values : identityMatrix4x4();
}

bool lookupNeedRotationForSegment(int segmentIndex)
{
    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr == nullptr || segmentIndex <= 0) {
        return false;
    }

    const auto& pathsConfig = configMgr->scanPathsConfig();
    for (const auto& path : pathsConfig.scanPaths) {
        if (!path.enabled) {
            continue;
        }
        for (const auto& point : path.points) {
            if (point.pointIndex == segmentIndex) {
                return point.needRotation;
            }
        }
    }
    return false;
}

QString selectedSegmentTextForInspection(const scan_tracking::common::TrackingConfig& tracking)
{
    return QStringLiteral("[%1,%2,%3]")
        .arg(tracking.firstStationOuterSegmentIndex)
        .arg(tracking.firstStationInnerSegmentIndex)
        .arg(tracking.firstStationHoleSegmentIndex);
}

/// 日志中展示 PLC 原始字；若与下游解码值不同则注明（如 16256→段号1）
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

/**
 * @brief 将浮点数转换为 CDAB 字节序的两个寄存器值
 * 
 * Modbus 协议中 float 类型通常占用两个 16 位寄存器，采用 CDAB 字节序：
 * - 低16位在前（Little Endian 的低位字）
 * - 高16位在后（Little Endian 的高位字）
 * 
 * @param value 要转换的浮点数值
 * @return 包含两个寄存器值的向量 [low_word, high_word]
 */
QVector<quint16> floatToCdabRegisters(float value)
{
    quint32 raw = 0;
    static_assert(sizeof(raw) == sizeof(value), "Unexpected float width");
    std::memcpy(&raw, &value, sizeof(raw));

    const quint16 high = static_cast<quint16>((raw >> 16) & 0xFFFFu);
    const quint16 low = static_cast<quint16>(raw & 0xFFFFu);
    return {low, high};
}

scan_tracking::flow_control::StateMachine::PoseSourceResult parsePoseSource(
    const char* envName,
    const QString& sourceName,
    const std::array<float, 6>& fallback,
    bool treatMissingAsSimulated)
{
    scan_tracking::flow_control::StateMachine::PoseSourceResult result;
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

    const auto tokens = raw.split(QRegExp(QStringLiteral("[,;\\s]+")), QString::SkipEmptyParts);
    if (tokens.size() < 6) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 需要 6 个值：x,y,z,rx,ry,rz。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }

    bool ok = false;
    const float x = tokens.value(0).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float y = tokens.value(1).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float z = tokens.value(2).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float rx = tokens.value(3).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float ry = tokens.value(4).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float rz = tokens.value(5).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }

    result.success = true;
    result.message = QStringLiteral("从外部源 %1 加载位姿。").arg(QString::fromLatin1(envName));
    result.x = x;
    result.y = y;
    result.z = z;
    result.rx = rx;
    result.ry = ry;
    result.rz = rz;
    return result;
}

}  // namespace

/**
 * @brief 状态机构造函数
 * 
 * 初始化流程控制状态机，配置定时器、连接信号槽，并设置初始状态。
 * 
 * @param modbusService Modbus 通信服务指针
 * @param mechEyeService Mech-Eye 相机服务指针
 * @param trackingService 跟踪检测服务指针
 * @param parent Qt 父对象指针
 */
StateMachine::StateMachine(
    modbus::ModbusService* modbusService,
    mech_eye::MechEyeService* mechEyeService,
    vision::VisionPipelineService* visionPipelineService,
    tracking::TrackingService* trackingService,
    QObject* parent)
    : QObject(parent)
    , m_modbus(modbusService)
    , m_mechEye(mechEyeService)
    , m_visionPipeline(visionPipelineService)
    , m_tracking(trackingService)
    , m_pollTimer(new QTimer(this))
    , m_heartbeatTimer(new QTimer(this))
    , m_timeoutTimer(new QTimer(this))
    , m_state(AppState::Init)
{
    // 从配置管理器获取流程控制配置，如果配置不存在则使用默认值
    const auto* configMgr = common::ConfigManager::instance();
    const auto flowConfig = configMgr ? configMgr->flowControlConfig()
                                      : common::FlowControlConfig{100, 1000, 300};

    // 配置定时器间隔
    m_pollTimer->setInterval(flowConfig.pollIntervalMs);      // PLC 轮询间隔
    m_heartbeatTimer->setInterval(flowConfig.heartbeatIntervalMs);  // 心跳发送间隔
    m_timeoutTimer->setSingleShot(true);  // 超时定时器为单次触发

    // 连接定时器信号到对应的槽函数
    connect(m_pollTimer, &QTimer::timeout, this, &StateMachine::pollPlcState);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &StateMachine::publishHeartbeat);
    connect(m_timeoutTimer, &QTimer::timeout, this, &StateMachine::onProcessTimeout);

    reloadCalibrationMatricesFromConfig();

    // 如果 Modbus 服务可用，连接其信号
    if (m_modbus) {
        connect(m_modbus, &modbus::ModbusService::connected, this, &StateMachine::onModbusConnected);
        connect(m_modbus, &modbus::ModbusService::disconnected, this, &StateMachine::onModbusDisconnected);
        connect(m_modbus, &modbus::ModbusService::errorOccurred, this, &StateMachine::onModbusError);
        connect(m_modbus, &modbus::ModbusService::registersRead, this, &StateMachine::handleRegistersRead);
        connect(m_modbus, &modbus::ModbusService::registerReadFailed, this, &StateMachine::onRegisterReadFailed);
        // P1修复：连接写失败信号，便于追踪具体哪个寄存器写入失败
        connect(m_modbus, &modbus::ModbusService::registerWriteFailed, this, &StateMachine::onRegisterWriteFailed);
    }

    // 如果 Mech-Eye 相机服务可用，连接其信号
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
            &vision::VisionPipelineService::bundleCaptureFinished,
            this,
            &StateMachine::onVisionBundleCaptureFinished,
            Qt::QueuedConnection);
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

/**
 * @brief 析构函数，停止所有定时器并清理资源
 */
StateMachine::~StateMachine()
{
    stop();
}

/**
 * @brief 启动状态机
 * 
 * 重置所有状态变量，发布初始 IPC 状态，如果 Modbus 已连接则直接进入就绪状态。
 * 此方法通常在系统初始化或从错误恢复后调用。
 */
void StateMachine::start()
{
    qInfo(LOG_FLOW) << QStringLiteral("状态机启动。");
    clearActiveTask();           // 清除当前活动任务
    resetScanSegmentCache();     // 清空扫描缓存
    m_isPollingPlc = false;      // 重置 PLC 轮询标志
    m_ipcState = protocol::IpcState::Initializing;  // 设置 IPC 状态为初始化中
    m_currentStage = protocol::Stage::Idle;         // 设置当前阶段为空闲
    m_alarmLevel = 0;            // 清除报警级别
    m_alarmCode = 0;             // 清除报警代码
    m_warnCode = 0;              // 清除警告代码
    m_progress = 0;              // 重置进度
    m_dataValid = false;         // 标记数据无效
    m_consecutiveModbusFailures = 0;  // 重置 Modbus 失败计数器
    const int stalePending = m_pendingRefinementJobs.exchange(0, std::memory_order_acq_rel);
    if (stalePending != 0) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("状态机 start：复位后台 refinement 在途计数 stale=") << stalePending;
    }
    setState(AppState::Init);    // 设置应用状态为初始化
    publishIpcStatus();          // 发布 IPC 状态到 PLC

    // 如果 Modbus 已经连接，直接触发连接成功处理
    if (m_modbus && m_modbus->isConnected()) {
        onModbusConnected();
    }
}

/**
 * @brief 停止状态机
 * 
 * 停止所有定时器，清除活动任务，重置所有状态变量。
 * 此方法通常在系统关闭或需要完全重置时调用。
 */
void StateMachine::stop()
{
    joinAllBackgroundRefinementJobs();
    if (pendingRefinementJobCount() != 0) {
        reconcilePendingRefinementJobCounter("stop");
    }

    m_pollTimer->stop();         // 停止 PLC 轮询定时器
    m_heartbeatTimer->stop();    // 停止心跳定时器
    m_timeoutTimer->stop();      // 停止超时定时器
    m_isPollingPlc = false;      // 重置 PLC 轮询标志
    clearActiveTask();           // 清除当前活动任务
    resetScanSegmentCache();     // 清空扫描缓存
    m_consecutiveModbusFailures = 0;  // 重置 Modbus 失败计数器
    m_alarmLevel = 0;
    m_alarmCode = 0;
    m_warnCode = 0;
    m_progress = 0;
    m_dataValid = false;
    m_heartbeatCounter = 0;
    m_ipcState = protocol::IpcState::Uninitialized;  // 设置 IPC 状态为未初始化
    m_currentStage = protocol::Stage::Idle;         // 设置当前阶段为空闲
    resetPlcOutputRegisters();   // 退出时将 IPC→PLC 结果区全部清零
    setState(AppState::Init);    // 设置应用状态为初始化
    m_inspectionResultPublisher = nullptr;  // 避免析构已 move 走的 std::function
}

/**
 * @brief 设置应用状态并发出状态变更信号
 * 
 * @param newState 新的应用状态
 */
void StateMachine::setState(AppState newState)
{
    if (m_state != newState) {
        m_state = newState;
        emit stateChanged(newState);  // 发出状态变更信号，通知外部监听者
        qInfo(LOG_FLOW) << QStringLiteral("应用状态切换为：") << static_cast<int>(newState);
    }
}

/**
 * @brief Modbus 连接成功时的回调处理
 * 
 * 重置失败计数器，设置 IPC 状态为就绪，启动定时器和心跳。
 */
void StateMachine::onModbusConnected()
{
    qInfo(LOG_FLOW) << QStringLiteral("Modbus 已连接，流程控制就绪。");
    
    // P2改进：重连后清理可能的残留状态，确保系统处于干净的初始状态
    if (m_activeTask.definition != nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Modbus 重连后清除残留活动任务：")
            << protocol::triggerName(*m_activeTask.definition);
        clearActiveTask();
        resetScanSegmentCache();
    }
    
    m_isPollingPlc = false;           // 重置 PLC 轮询标志
    resetModbusFailureCounter();      // 重置 Modbus 失败计数器
    m_consecutiveModbusFailures = 0;  // 显式清零失败计数器
    m_ipcState = protocol::IpcState::Ready;   // 设置 IPC 状态为就绪
    m_currentStage = protocol::Stage::Idle;   // 设置当前阶段为空闲
    m_alarmLevel = 0;                 // 清除报警级别
    m_alarmCode = 0;                  // 清除报警代码
    m_warnCode = 0;                   // 清除警告代码
    m_progress = 0;                   // 重置进度
    m_dataValid = false;              // 标记数据无效
    setState(AppState::Ready);        // 设置应用状态为就绪
    publishIpcStatus();               // 发布 IPC 状态到 PLC
    publishHeartbeat();               // 立即发送一次心跳
    m_pollTimer->start();             // 启动 PLC 轮询定时器
    m_heartbeatTimer->start();        // 启动心跳定时器
    
    qInfo(LOG_FLOW) << QStringLiteral("Modbus 重连恢复完成，系统已回到就绪状态。");
}

/**
 * @brief Modbus 断开连接时的回调处理
 * 
 * 停止所有定时器，进入故障状态，等待重新连接。
 */
void StateMachine::onModbusDisconnected()
{
    qWarning(LOG_FLOW) << QStringLiteral("Modbus 已断开，流程控制暂停。");
    m_pollTimer->stop();       // 停止 PLC 轮询
    m_heartbeatTimer->stop();  // 停止心跳
    m_timeoutTimer->stop();    // 停止超时定时器
    m_isPollingPlc = false;    // 重置 PLC 轮询标志
    // 进入故障状态，报警代码 900，不中止当前任务（因为没有活跃任务），不通知 PLC
    enterFaultState(900, QStringLiteral("Modbus 已断开连接"), true, false);
}

/**
 * @brief Modbus 发生错误时的回调处理
 * 
 * 记录错误并增加失败计数器，连续失败达到阈值时将进入故障状态。
 * 
 * @param errorString 错误描述信息
 */
void StateMachine::onModbusError(const QString& errorString)
{
    qWarning(LOG_FLOW).noquote() << "Modbus 错误传播到流程控制：" << errorString;
    recordModbusFailure(901, errorString);  // 记录 Modbus 失败，报警代码 901
}

/**
 * @brief 轮询 PLC 状态（Server 模式下为空操作）
 * 
 * 在 Server 模式下，PLC 主动写入命令区时会触发 onDataWritten → registersRead 信号，
 * 不需要 IPC 主动轮询。保留此函数是为了兼容定时器连接，避免改动过大。
 */
void StateMachine::pollPlcState()
{
    // Server 模式：PLC 主动写入命令区，通过 registersRead 信号触发 handleRegistersRead
    // 此函数保留为空操作，轮询定时器可用于其他周期性检查（如心跳超时检测）
}

/**
 * @brief 处理从 PLC 读取的寄存器数据
 *
 * 解析命令块，检测触发信号，处理任务完成确认等。
 *
 * @param startAddress 起始寄存器地址
 * @param values 读取到的寄存器值向量
 */
void StateMachine::handleRegistersRead(int startAddress, const QVector<quint16>& values)
{
    // 验证是否是预期的命令块读取，且数据长度足够
    if (startAddress != protocol::registers::kCommandBlockStart ||
        values.size() < protocol::registers::kCommandBlockSize) {
        return;
    }

    const QVector<quint16> previousCommandBlock = m_lastCommandBlock;
    m_lastCommandBlock = values;       // 保存最新的命令块数据
    resetModbusFailureCounter();       // 通信成功，重置失败计数器

    // 判断命令块是否发生业务变化：忽略 PLC_Heartbeat，避免每 100ms 因心跳刷屏
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

    // 轮询完成日志：节流输出，避免 100ms 轮询刷屏
    if (m_activePollRequestSequence == 1 || (m_activePollRequestSequence % kPollLogEveryN) == 0) {
        qDebug(LOG_FLOW).noquote()
            << QStringLiteral("PLC 轮询完成")
            << QStringLiteral(" 请求序号=") << m_activePollRequestSequence
            << QStringLiteral(" 耗时ms=") << (m_pollRequestTimer.isValid() ? m_pollRequestTimer.elapsed() : -1);
    }

    m_activePollRequestSequence = 0;

    // 命令块快照：只在首次读取或内容变化时打印
    if (commandBlockChanged) {
        namespace regs = protocol::registers;
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("命令块快照：")
            << "PLC_Start=" << regs::holdingRegisterAddress(regs::kCommandBlockStart)
            << "Flow_Enable=" << values.value(regs::kFlowEnable)
            << "Reg04=" << values.value(regs::kSafetyStatusWord)
            << "ScanSegmentIndex=" << protocol::registers::resolveScanSegmentIndexFromBlock(values)
            << "RequestTimeout_s=" << formatPlcRegisterValueForLog(regs::kRequestTimeoutSeconds,
                                                                   values.value(regs::kRequestTimeoutSeconds))
            << "Trig_LoadGrasp=" << values.value(regs::modbusIndexFromPlcAddress(40020))
            << "Trig_StationMaterialCheck=" << values.value(regs::modbusIndexFromPlcAddress(40021))
            << "Trig_PoseCheck=" << values.value(regs::modbusIndexFromPlcAddress(40022))
            << "Trig_ScanSegment=" << values.value(regs::modbusIndexFromPlcAddress(40023))
            << "Trig_Inspection=" << values.value(regs::modbusIndexFromPlcAddress(40024))
            << "Trig_UnloadCalc=" << values.value(regs::modbusIndexFromPlcAddress(40025))
            << "Trig_SelfCheck=" << values.value(regs::modbusIndexFromPlcAddress(40026))
            << "Trig_CodeRead=" << values.value(regs::modbusIndexFromPlcAddress(40027))
            << "Trig_ResultReset=" << values.value(regs::modbusIndexFromPlcAddress(40028))
            << "TaskIdHigh=" << values.value(regs::kTaskIdHigh)
            << "TaskIdLow=" << values.value(regs::kTaskIdLow);
    }

    // 命令块原始寄存器值：只在首次读取或内容变化时打印
    if (commandBlockChanged) {
        QStringList rawRegisters;
        rawRegisters.reserve(values.size());

        for (int index = 0; index < values.size(); ++index) {
            rawRegisters << QStringLiteral("%1=%2")
                .arg(index)
                .arg(values.value(index));
        }

        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("命令块原始寄存器：")
            << rawRegisters.join(QStringLiteral(" "));
    }

    // 打印变化字段：只在非首次读取且有字段变化时打印（带 PLC 地址和寄存器名）
    if (!previousCommandBlock.isEmpty()) {
        static const char* const kRegisterNames[] = {
            "Reserved_0",              // 0
            "PLC_Heartbeat",           // 1   40001
            "PLC_SystemState",         // 2
            "Station_WorkMode",        // 3
            "Flow_Enable",             // 4
            "Safety_Status_Word",      // 5
            "Cmd_StartAuto",           // 6
            "Cmd_Pause",               // 7
            "Cmd_Stop",                // 8
            "Cmd_Reset",               // 9
            "Cmd_ClearAlarms",         // 10
            "TaskId_H",                // 11
            "TaskId_L",                // 12
            "ProductType",             // 13
            "RecipeId",                // 14
            "ScanSegmentIndex",        // 15  40015
            "ScanSegmentIndex_Robot",  // 16  40016 机械臂/PLC 实际段号
            "RequestTimeout_s",        // 17  40017
            "Reserved_18",             // 18
            "Reserved_19",             // 19
            "Trig_LoadGrasp",          // 20  40020
            "Trig_StationMaterialCheck", // 21
            "Trig_PoseCheck",          // 22
            "Trig_ScanSegment",        // 23
            "Trig_Inspection",         // 24
            "Trig_UnloadCalc",         // 25
            "Trig_SelfCheck",          // 26
            "Trig_CodeRead",           // 27
            "Trig_ResultReset",        // 28
            "Reserved_29",             // 29
        };
        constexpr int kNameCount = sizeof(kRegisterNames) / sizeof(kRegisterNames[0]);
        const int compareCount = qMin(previousCommandBlock.size(), qMin(values.size(), 30));

        QStringList changedFields;
        for (int index = 0; index < compareCount; ++index) {
            const quint16 oldValue = previousCommandBlock.value(index);
            const quint16 newValue = values.value(index);

            if (oldValue == newValue) {
                continue;
            }

            const char* name = (index < kNameCount) ? kRegisterNames[index] : "?";
            changedFields << QStringLiteral("  [%1] %2 (plcOffset=%3, modbusIndex=%4): %5")
                .arg(protocol::registers::holdingRegisterAddress(index))
                .arg(QString::fromLatin1(name))
                .arg(protocol::registers::plcTableOffset(index))
                .arg(index)
                .arg(formatPlcRegisterChangeForLog(index, oldValue, newValue));
        }

        if (!changedFields.isEmpty()) {
            qInfo(LOG_FLOW).noquote()
                << "=== PLC 寄存器变化 ===" << "\n" << changedFields.join(QStringLiteral("\n"));
        }
    }

    // 如果当前有活动任务且已完成宣告，检查 PLC 是否已释放触发信号
    if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
        finalizeCompletedTaskIfTriggerReleased(values);
        // PLC 联调临时容错：勿 return；释放扫描触发的同轮询内可能已置位 Trig_Inspection。
    }

    // 如果有活动任务但未完成宣告，等待任务执行完毕
    if (m_activeTask.definition != nullptr) {
        return;
    }

    if (const protocol::TriggerDefinition* pendingTrigger = selectPendingTrigger(values)) {
        processTrigger(*pendingTrigger, values);
    }
}
/**
 * @brief 处理寄存器读取失败的回调
 * 
 * 记录错误并重置轮询标志，允许下一次轮询继续进行。
 * 
 * @param startAddress 读取失败的起始地址
 * @param errorString 错误描述信息
 */
void StateMachine::onRegisterReadFailed(int startAddress, const QString& errorString)
{
    // 只关心命令块的读取失败
    if (startAddress != protocol::registers::kCommandBlockStart) {
        return;
    }

    if (m_isPollingPlc) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("PLC 轮询失败：")
            << errorString
            << QStringLiteral(" 请求序号=") << m_activePollRequestSequence
            << QStringLiteral(" 耗时ms=") << (m_pollRequestTimer.isValid() ? m_pollRequestTimer.elapsed() : -1);
    }
    m_isPollingPlc = false;  // 重置轮询标志，允许下次轮询
    m_activePollRequestSequence = 0;
}

/**
 * @brief 处理寄存器写入失败事件
 * 
 * P1修复：当 Modbus 写操作失败时记录详细日志，便于追踪具体哪个寄存器写入失败。
 * 此槽函数连接到 ModbusService 的 registerWriteFailed 信号。
 * 
 * @param startAddress 写入失败的起始地址
 * @param errorString 错误描述信息
 */
void StateMachine::onRegisterWriteFailed(int startAddress, const QString& errorString)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("寄存器写入失败，地址=") << startAddress
        << QStringLiteral(" (0x") << QString::number(startAddress, 16) << QStringLiteral(")：")
        << errorString;
    
    // 可以根据地址范围判断是哪个业务的写入失败，采取不同的恢复策略
    // 例如：如果是结果区写入失败，可能需要重新发送结果
}

/**
 * @brief 处理 PLC 触发信号
 * 
 * 当检测到某个触发位被置为 1 时，验证触发条件，初始化任务状态，
 * 发送 ACK 响应，启动超时定时器，并执行对应的任务逻辑。
 * 
 * @param trigger 触发定义结构，包含触发偏移、阶段、超时等信息
 * @param commandBlock 完整的命令块寄存器数据
 */
void StateMachine::processTrigger(const protocol::TriggerDefinition& trigger, const QVector<quint16>& commandBlock)
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法处理触发
    }

    // 除了卸载计算和结果复位外，其他触发都需要 Flow_Enable=1 才能执行
    if (trigger.stage != protocol::Stage::UnloadCalc &&
        trigger.stage != protocol::Stage::ResultReset &&
        commandBlock.value(protocol::registers::kFlowEnable) == 0) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("Flow_Enable=0 时拒绝触发：")
                                     << protocol::triggerName(trigger);
        sendRes(trigger, 9);                          // 返回错误码 9（参数错误）
        sendAck(trigger, protocol::AckState::Failed); // 发送失败 ACK
        return;
    }

    // 初始化活动任务状态
    m_activeTask.definition = &trigger;                                    // 保存触发定义指针
    m_activeTask.taskId = readTaskId(commandBlock);                        // 读取任务 ID
    // 如果 PLC 指定了超时时间则使用，否则使用触发定义的默认超时
    {
        const quint16 timeoutRaw = commandBlock.value(protocol::registers::kRequestTimeoutSeconds);
        const quint16 timeoutDecoded = protocol::registers::plcAnalogToUInt16(timeoutRaw, 0);
        m_activeTask.timeoutSeconds = timeoutDecoded > 0
            ? timeoutDecoded
            : static_cast<quint16>(trigger.defaultTimeoutSeconds);
    }
    m_activeTask.scanSegmentIndex = resolveScanSegmentIndex(commandBlock); // 解析扫描分段索引

    if (trigger.stage == protocol::Stage::ScanSegment) {
        QString validationError;
        if (!validateScanSegmentRequest(commandBlock, &validationError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("拒绝触发") << protocol::triggerName(trigger)
                << validationError;
            m_activeTask.definition = nullptr;
            sendRes(trigger, 9);
            sendAck(trigger, protocol::AckState::Failed);
            return;
        }
    }

    {
        const auto* cfgMgr = scan_tracking::common::ConfigManager::instance();
        m_activeTask.scanSegmentTotal = cfgMgr ? cfgMgr->trackingConfig().scanSegmentTotal : 1;
    }
    m_activeTask.completionAnnounced = false;  // 重置完成宣告标志
    m_activeTask.captureRequestId = 0;         // 重置采集请求 ID

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已接受触发") << protocol::triggerName(trigger)
        << QStringLiteral(" 超时s=") << m_activeTask.timeoutSeconds
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex;

    // 清除之前的报警信息
    setAlarm(0, 0, QString());
    // 切换到扫描状态
    setState(AppState::Scanning);
    m_ipcState = protocol::IpcState::Busy;   // IPC 状态设为忙碌
    m_currentStage = trigger.stage;          // 设置当前阶段
    m_progress = 5;                          // 进度设为 5%（刚开始）
    m_dataValid = false;                     // 数据无效
    publishIpcStatus();                      // 发布 IPC 状态到 PLC

    sendAck(trigger, protocol::AckState::Running);  // 发送运行中 ACK

    // 如果任务 ID 不为 0，回写到 PLC 的任务 ID 回声寄存器
    if (m_activeTask.taskId != 0) {
        const bool taskIdWritten = m_modbus->writeRegisters(protocol::registers::kTaskIdEchoHigh, {
            static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu),
            static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),
        });
        if (!taskIdWritten) {
            qWarning(LOG_FLOW).noquote() << QStringLiteral("写入任务 ID 回声寄存器失败");
        }
    }

    // 启动超时定时器（秒转毫秒）
    m_timeoutTimer->start(static_cast<int>(m_activeTask.timeoutSeconds) * 1000);
    // 执行具体的任务逻辑
    executeActiveTask();
}

/**
 * @brief 执行当前活动任务
 * 
 * 根据触发定义的 trigOffset 分发到具体的任务执行函数。
 * 每个 trigOffset 对应一个特定的 PLC 触发信号。
 */
void StateMachine::executeActiveTask()
{
    if (m_activeTask.definition == nullptr) {
        return;  // 没有活动任务，直接返回
    }

    switch (m_activeTask.definition->trigOffset) {
    case 20:  // Trig_LoadGrasp @40020
        executeLoadGraspTask();
        return;
    case 21:  // Trig_StationMaterialCheck @40021
        executeStationMaterialCheckTask();
        return;
    case 22:  // Trig_PoseCheck @40022
        executePoseCheckTask();
        return;
    case 23:  // Trig_ScanSegment @40023
        executeScanSegmentTask();
        return;
    case 24:  // Trig_Inspection @40024
        executeInspectionTask();
        return;
    case 25:  // Trig_UnloadCalc @40025
        executeUnloadCalcTask();
        return;
    case 26:  // Trig_SelfCheck @40026
        executeSelfCheckTask();
        return;
    case 27:  // Trig_CodeRead @40027
        executeCodeReadTask();
        return;
    case 28:  // Trig_ResultReset @40028
        executeResultResetTask();
        return;
    default:  // 未知触发类型，使用默认响应码完成任务
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("拒绝不支持的触发")
            << protocol::triggerName(*m_activeTask.definition)
            << QStringLiteral(" trigOffset=") << m_activeTask.definition->trigOffset;
        setAlarm(2, 624, QStringLiteral("收到不支持的触发"));
        completeActiveTask(9, protocol::AckState::Failed, false);
        return;
    }
}

/**
 * @brief 执行加载抓取任务（Trig_LoadGrasp）
 * 
 * 向 PLC 写入模拟的加载位姿数据，然后立即完成任务。
 * 这是一个占位实现，实际应用中可能需要调用视觉定位或机器人接口。
 */
void StateMachine::executeLoadGraspTask()
{
    const auto poseSource = resolveLoadGraspPoseSource();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("LoadGrasp 位姿来源：")
        << poseSource.sourceName
        << QStringLiteral(" 可用=") << poseSource.available
        << QStringLiteral(" 成功=") << poseSource.success
        << QStringLiteral(" 消息=") << poseSource.message;
    writeLoadGraspResult();   // 写入加载位姿结果到 PLC 寄存器
    const quint16 resultCode = poseSource.success ? 1 : 7;
    completeActiveTask(resultCode, poseSource.success ? protocol::AckState::Completed
                                                       : protocol::AckState::Failed,
                       poseSource.success);
    emit loadGraspFinished(resultCode, poseSource.x, poseSource.y, poseSource.z,
                           poseSource.rx, poseSource.ry, poseSource.rz);
}

void StateMachine::executeStationMaterialCheckTask()
{
    const bool hasModbus = m_modbus != nullptr && m_modbus->isConnected();
    const bool hasTracking = m_tracking != nullptr;
    const bool hasMechEye = m_mechEye != nullptr;

    if (!hasModbus || !hasTracking || !hasMechEye) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("工位检材不可用：")
            << QStringLiteral(" modbus=") << hasModbus
            << QStringLiteral(" tracking=") << hasTracking
            << QStringLiteral(" mechEye=") << hasMechEye;
        writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("NO"));
        completeActiveTask(5, protocol::AckState::Failed, false);
        return;
    }

    writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("OK"));
    completeActiveTask(1, protocol::AckState::Completed, true);
}

/**
 * @brief 执行卸载计算任务（Trig_UnloadCalc）
 * 
 * 向 PLC 写入模拟的卸料位姿数据，然后立即完成任务。
 * 这是一个占位实现，实际应用中可能需要进行路径规划或碰撞检测。
 */
void StateMachine::executeUnloadCalcTask()
{
    const auto poseSource = resolveUnloadCalcPoseSource();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("UnloadCalc 位姿来源：")
        << poseSource.sourceName
        << QStringLiteral(" 可用=") << poseSource.available
        << QStringLiteral(" 成功=") << poseSource.success
        << QStringLiteral(" 消息=") << poseSource.message;
    writeUnloadCalcResult();  // 写入卸料位姿结果到 PLC 寄存器
    const quint16 resultCode = poseSource.success ? 1 : 7;
    completeActiveTask(resultCode, poseSource.success ? protocol::AckState::Completed
                                                       : protocol::AckState::Failed,
                       poseSource.success);
    emit unloadCalcFinished(resultCode, poseSource.x, poseSource.y, poseSource.z,
                            poseSource.rx, poseSource.ry, poseSource.rz);
}

/**
 * @brief 执行扫描分段任务（Trig_ScanSegment）
 * 
 * 这是整个流程中最复杂的任务，负责控制 Mech-Eye 相机进行 3D 点云采集。
 * 包括参数验证、相机状态检查、异步采集请求发起等步骤。
 * 
 * 关键流程：
 * 1. 验证扫描分段请求的合法性（段号范围、重复检测）
 * 2. 检查相机是否就绪且空闲
 * 3. 发起异步采集请求
 * 4. 等待 onCaptureFinished 回调处理采集结果
 */
void StateMachine::executeScanSegmentTask()
{
    // 优先走视觉编排层：梅卡点云 + 海康双目黑白图同时采集。
    if (m_visionPipeline == nullptr) {
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            3,                    // 报警级别：3 = 严重错误
            720,                  // 报警代码：720 = 视觉编排服务不可用
            QStringLiteral("视觉流水线服务不可用"),
            QStringLiteral("视觉流水线服务不可用"));
        return;
    }

    // 验证扫描分段请求的参数合法性
    QString validationError;
    if (!validateScanSegmentRequest(m_lastCommandBlock, &validationError)) {
        // 段号错误或重复触发会污染分段缓存，因此在拍照前就拒绝本次业务。
        finishScanSegmentFailure(9, 2, 724, validationError, validationError);
        return;
    }

    // 检查相机是否处于就绪状态且当前没有正在进行的采集
    if (m_visionPipeline->state() != vision::VisionPipelineState::Ready || m_visionPipeline->isStarted() == false) {
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            2,                    // 报警级别：2 = 警告
            721,                  // 报警代码：721 = 视觉编排忙或未就绪
            QStringLiteral("视觉流水线忙或未就绪"),
            QStringLiteral("视觉流水线忙或未就绪"));
        return;
    }

    // 计算采集超时时间：优先使用任务指定的超时，否则使用默认值
    const int captureTimeoutMs = m_activeTask.timeoutSeconds > 0
        ? static_cast<int>(m_activeTask.timeoutSeconds) * 1000
        : kDefaultScanSegmentCaptureTimeoutMs;

    const bool needMechEye2D = resolveNeedRotationForSegment(m_activeTask.scanSegmentIndex);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[ScanSync] 触发") << QDateTime::currentMSecsSinceEpoch();
    const auto mechCaptureMode = needMechEye2D
        ? scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D
        : scan_tracking::mech_eye::CaptureMode::Capture3DOnly;
    const quint64 requestId = m_visionPipeline->requestCaptureBundle(
        m_activeTask.scanSegmentIndex,
        m_activeTask.taskId,
        mechCaptureMode);

    if (requestId == 0) {
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            2,                    // 报警级别：2 = 警告
            721,                  // 报警代码：721 = 视觉编排忙或未就绪
            QStringLiteral("视觉流水线拒绝采集请求"),
            QStringLiteral("视觉流水线忙或未就绪"));
        return;
    }

    // 保存采集请求 ID，用于在回调中匹配响应
    m_activeTask.captureRequestId = requestId;
    m_progress = 30;              // 更新进度为 30%（采集中）
    publishIpcStatus();           // 发布更新的 IPC 状态

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ScanSegment 已启动组合采集")
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex
        << QStringLiteral(" 段总数=") << m_activeTask.scanSegmentTotal
        << QStringLiteral(" 需梅卡2D=") << needMechEye2D
        << QStringLiteral(" 超时ms=") << captureTimeoutMs;
    emit scanStarted(m_activeTask.scanSegmentIndex, m_activeTask.taskId);
}

void StateMachine::onVisionBundleCaptureFinished(scan_tracking::vision::MultiCameraCaptureBundle bundle)
{
    if (!m_activeTask.definition
        || m_activeTask.definition->stage != protocol::Stage::ScanSegment) {
        return;
    }
    if (m_activeTask.completionAnnounced) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("忽略过期视觉 bundle（Trig_ScanSegment 已握手完成）")
            << QStringLiteral(" 段号=") << bundle.request.segmentIndex
            << QStringLiteral(" requestId=") << bundle.request.requestId;
        return;
    }
    if (bundle.request.taskId != m_activeTask.taskId) {
        return;
    }
    if (m_activeTask.captureRequestId != 0
        && bundle.request.requestId != m_activeTask.captureRequestId) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("忽略 requestId 不匹配的视觉 bundle：期望=")
            << m_activeTask.captureRequestId
            << QStringLiteral(" 实际=") << bundle.request.requestId;
        return;
    }

    if (!bundle.mechEyeResult.success()) {
        const quint16 resultCode = mapCaptureErrorToResCode(bundle.mechEyeResult.errorCode);
        finishScanSegmentFailure(
            resultCode,
            3,
            722,
            QStringLiteral("视觉组合中 MechEye 采集失败"),
            QStringLiteral("视觉组合中 MechEye 采集失败"));
        return;
    }

    // TODO: MechEye 暂时屏蔽（已验证通过），当前只测试海康 A/B
    // 当 MechEye 恢复后，需检查 bundle.mechEyeResult.success()

    // 海康 A/B 采集失败则报错终止
    if (!bundle.hikCameraAResult.success() || !bundle.hikCameraBResult.success()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("海康 A/B 采集失败")
            << QStringLiteral(" hikA=") << bundle.hikCameraAResult.errorMessage
            << QStringLiteral(" hikB=") << bundle.hikCameraBResult.errorMessage;
        // 不阻断流程但记录失败
    }

    const auto& result = bundle.mechEyeResult;
    const bool needRotation = resolveNeedRotationForSegment(m_activeTask.scanSegmentIndex);
    applyLbnCalibrationUpdate(m_activeTask.scanSegmentIndex, needRotation, bundle);

    if (!result.pointCloud.isValid()) {
        finishScanSegmentFailure(
            7,
            3,
            722,
            QStringLiteral("Mech-Eye 点云无效，无法缓存"),
            QStringLiteral("Mech-Eye 点云无效，无法缓存"));
        return;
    }

    const int segmentIndex = m_activeTask.scanSegmentIndex;
    const quint32 taskId = m_activeTask.taskId;
    const scan_tracking::common::PointCloudProcessingConfig processingConfig =
        scan_tracking::common::ConfigManager::instance()
            ? scan_tracking::common::ConfigManager::instance()->pointCloudProcessingConfig()
            : scan_tracking::common::PointCloudProcessingConfig{};

    commitScanSegmentCaptureImmediate(segmentIndex, result, bundle);
    startSegmentBackgroundRefinement(segmentIndex, taskId, processingConfig);
}

void StateMachine::commitScanSegmentCaptureImmediate(
    int segmentIndex,
    const scan_tracking::mech_eye::CaptureResult& result,
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        m_segmentCaptureResults.insert(segmentIndex, cloneCaptureResult(result));
        m_segmentCaptureBundles.insert(segmentIndex, cloneCaptureBundle(bundle));
    }

    int imageCount = countHikImagesInBundle(bundle);
    if (imageCount == 0) {
        imageCount = 1;
    }
    const int cloudFrameCount = result.pointCloud.pointCount > 0 ? 1 : 0;

    writeScanSegmentResult(segmentIndex, imageCount, cloudFrameCount);
    m_progress = 100;
    publishIpcStatus();

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SegmentCache] 原始点云已入内存，立即回写 PLC")
        << QStringLiteral(" 段号=") << segmentIndex
        << QStringLiteral(" 任务ID=") << m_activeTask.taskId
        << QStringLiteral(" 点数=") << result.pointCloud.pointCount
        << QStringLiteral(" 缓存段数=") << m_segmentCaptureResults.size();

    completeActiveTask(1);
    emit scanFinished(segmentIndex, 1, imageCount, cloudFrameCount);
}

void StateMachine::registerRefinementJob()
{
    const int previous = m_pendingRefinementJobs.fetch_add(1, std::memory_order_acq_rel);
    if (previous + 1 > kMaxReasonableRefinementJobs) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台 refinement 在途数偏高 pending=") << (previous + 1);
    }
}

void StateMachine::completeRefinementJob()
{
    const int previous = m_pendingRefinementJobs.fetch_sub(1, std::memory_order_acq_rel);
    if (previous <= 0) {
        m_pendingRefinementJobs.fetch_add(1, std::memory_order_relaxed);
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台 refinement 完成时计数已为 0（重复收尾）");
    }
}

int StateMachine::pendingRefinementJobCount() const
{
    const int count = m_pendingRefinementJobs.load(std::memory_order_acquire);
    if (count < 0 || count > kMaxReasonableRefinementJobs) {
        return -1;
    }
    return count;
}

int StateMachine::reconcilePendingRefinementJobCounter(const char* reason)
{
    const int pending = m_pendingRefinementJobs.exchange(0, std::memory_order_acq_rel);
    if (pending == 0) {
        return 0;
    }

    qCritical(LOG_FLOW).noquote()
        << QStringLiteral("后台 refinement 在途计数已强制复位 pending=") << pending
        << QStringLiteral(" 原因=") << (reason != nullptr ? QString::fromUtf8(reason) : QString())
        << QStringLiteral("（常见根因：旧版 dispatch 对已 move 的 outcome 二次 invoke；"
                           "或 register/complete 未成对；将用原始点云继续综合检测）");
    return pending;
}

void StateMachine::joinAllBackgroundRefinementJobs(int maxWaitMs)
{
    const int waitLimitMs = maxWaitMs >= 0
        ? maxWaitMs
        : kBackgroundRefinementJoinTimeoutMs;

    int remaining = pendingRefinementJobCount();
    if (remaining < 0) {
        reconcilePendingRefinementJobCounter("join 入口计数异常");
        return;
    }
    if (remaining <= 0) {
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("等待后台点云 refinement 结束，剩余任务数=") << remaining
        << QStringLiteral(" 最长等待ms=") << waitLimitMs;

    int waitedMs = 0;
    while (remaining > 0 && waitedMs < waitLimitMs) {
        // 先 pump 事件，确保 QueuedConnection 的 refinement 收尾能执行 completeRefinementJob
        if (QCoreApplication::instance() != nullptr) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        }
        QThread::msleep(50);
        waitedMs += 50;
        remaining = pendingRefinementJobCount();
        if (remaining < 0) {
            reconcilePendingRefinementJobCounter("join 轮询计数异常");
            return;
        }
    }

    if (remaining > 0) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台点云 refinement 未在时限内结束，剩余=") << remaining
            << QStringLiteral("；综合检测将使用当前缓存（可能含未 refine 的原始点云）");
        reconcilePendingRefinementJobCounter("join 超时仍有在途任务");
    }
}

void StateMachine::dispatchSegmentRefinementFinished(SegmentProcessOutcome outcome)
{
    // 使用 shared_ptr 避免 invokeMethod 失败时对已 move 的 outcome 二次 move（UB，可破坏 m_pendingRefinementJobs）
    const auto outcomeHolder = std::make_shared<SegmentProcessOutcome>(std::move(outcome));
    QPointer<StateMachine> self(this);

    const auto deliverOnMainThread = [self, outcomeHolder]() {
        if (!self) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("后台 refinement 回调时 StateMachine 已销毁，在途计数将在 stop/reset 时复位");
            return;
        }
        self->onSegmentBackgroundRefinementFinished(std::move(*outcomeHolder));
    };

    if (QThread::currentThread() == thread()) {
        deliverOnMainThread();
        return;
    }

    if (QCoreApplication::instance() == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台 refinement 无法投递到主线程（无 QCoreApplication），仅平衡在途计数");
        completeRefinementJob();
        return;
    }

    if (QMetaObject::invokeMethod(this, deliverOnMainThread, Qt::QueuedConnection)) {
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("后台 refinement QueuedConnection 失败，改 BlockingQueuedConnection 段号=")
        << outcomeHolder->segmentIndex;
    QMetaObject::invokeMethod(this, deliverOnMainThread, Qt::BlockingQueuedConnection);
}

void StateMachine::startSegmentBackgroundRefinement(
    int segmentIndex,
    quint32 taskId,
    const scan_tracking::common::PointCloudProcessingConfig& config)
{
    if (!config.enabled) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[ScanSync] 点云后处理已禁用，跳过后台 refinement，段号=") << segmentIndex;
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[ScanSync] 后台 refinement 已排队，段号=") << segmentIndex
        << QStringLiteral(" 任务ID=") << taskId;

    QPointer<StateMachine> self(this);
    const qint64 refinementQueuedAtMs = QDateTime::currentMSecsSinceEpoch();
    std::thread([self, segmentIndex, taskId, config, refinementQueuedAtMs]() {
        QElapsedTimer refinementWallTimer;
        refinementWallTimer.start();

        // 多段 400 万级点云并行 refinement 会占满内存；与 PCL 全局锁一致，进程内串行执行。
        static std::mutex kSegmentRefinementSerialMutex;
        std::lock_guard<std::mutex> serialGuard(kSegmentRefinementSerialMutex);
        const qint64 queueWaitMs = refinementWallTimer.elapsed();

        if (!self) {
            return;
        }

        self->registerRefinementJob();

        struct RefinementPendingGuard {
            QPointer<StateMachine> owner;
            bool handedOff = false;
            void handOff() { handedOff = true; }
            ~RefinementPendingGuard()
            {
                if (!handedOff && owner) {
                    owner->completeRefinementJob();
                }
            }
        } pendingGuard{self};

        SegmentProcessOutcome outcome;
        outcome.segmentIndex = segmentIndex;
        outcome.taskId = taskId;

        const auto logRefinementTiming = [&](const SegmentProcessOutcome& finished) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("[RefinementTimer] Mech点云后台refinement")
                << QStringLiteral(" 段号=") << finished.segmentIndex
                << QStringLiteral(" 任务ID=") << finished.taskId
                << QStringLiteral(" 成功=") << (finished.success ? QStringLiteral("是") : QStringLiteral("否"))
                << QStringLiteral(" 总耗时ms=") << finished.processElapsedMs
                << QStringLiteral(" (排队等待ms=") << queueWaitMs
                << QStringLiteral(" PCL多步处理ms=") << finished.pclProcessElapsedMs
                << QStringLiteral(" 点数=") << finished.rawPointCount
                << QStringLiteral("->") << finished.processedPointCount
                << QStringLiteral(" 入队时刻ms=") << refinementQueuedAtMs;
        };

        const auto finishJob = [&](SegmentProcessOutcome finishedOutcome) {
            finishedOutcome.processElapsedMs = refinementWallTimer.elapsed();
            logRefinementTiming(finishedOutcome);
            if (self) {
                pendingGuard.handOff();
                self->dispatchSegmentRefinementFinished(std::move(finishedOutcome));
            } else {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("后台 refinement 完成时 StateMachine 已销毁，段号=") << segmentIndex;
            }
        };

        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[ScanSync] 后台 refinement 开始，段号=") << segmentIndex
            << QStringLiteral(" 任务ID=") << taskId
            << QStringLiteral(" 排队等待ms=") << queueWaitMs;

        try {
            if (!self) {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("后台 refinement 启动时 StateMachine 已销毁，段号=") << segmentIndex;
                return;
            }

            scan_tracking::mech_eye::PointCloudFrame inputCloud;
            {
                std::lock_guard<std::mutex> lock(self->m_segmentCacheMutex);
                const auto it = self->m_segmentCaptureResults.constFind(segmentIndex);
                if (it == self->m_segmentCaptureResults.cend() || !it->pointCloud.isValid()) {
                    outcome.success = false;
                    outcome.errorMessage =
                        QStringLiteral("后台 refinement 时缓存段不存在或点云无效。");
                } else {
                    inputCloud = clonePointCloudFrame(it->pointCloud);
                    outcome.rawPointCount = inputCloud.pointCount;
                }
            }

            if (!outcome.errorMessage.isEmpty()) {
                finishJob(std::move(outcome));
                return;
            }

            QElapsedTimer pclTimer;
            pclTimer.start();

            scan_tracking::mech_eye::PointCloudProcessReport report;
            scan_tracking::mech_eye::PointCloudFrame processedCloud;
            scan_tracking::mech_eye::CaptureResult captureInput;
            captureInput.pointCloud = std::move(inputCloud);

            if (scan_tracking::mech_eye::processPointCloudFrame(
                    captureInput.pointCloud,
                    config,
                    &processedCloud,
                    &report)) {
                outcome.success = true;
                outcome.captureResult.pointCloud = std::move(processedCloud);
                outcome.processedPointCount = report.outputPointCount;
                outcome.rawPointCount = report.inputPointCount;
            } else {
                outcome.success = false;
                outcome.errorMessage = report.message.isEmpty()
                    ? QStringLiteral("后台点云 refinement 失败。")
                    : report.message;
                outcome.rawPointCount = report.inputPointCount;
            }
            outcome.pclProcessElapsedMs = pclTimer.elapsed();
            finishJob(std::move(outcome));
        } catch (const std::exception& ex) {
            qCritical(LOG_FLOW).noquote()
                << QStringLiteral("后台 refinement 异常 段号=") << segmentIndex
                << QStringLiteral(" 说明=") << QString::fromUtf8(ex.what());
            outcome.success = false;
            outcome.errorMessage = QStringLiteral("后台 refinement 异常：%1")
                                       .arg(QString::fromUtf8(ex.what()));
            finishJob(std::move(outcome));
        } catch (...) {
            qCritical(LOG_FLOW).noquote()
                << QStringLiteral("后台 refinement 未知异常 段号=") << segmentIndex;
            outcome.success = false;
            outcome.errorMessage = QStringLiteral("后台 refinement 未知异常。");
            finishJob(std::move(outcome));
        }
    }).detach();
}

void StateMachine::onSegmentBackgroundRefinementFinished(SegmentProcessOutcome outcome)
{
    completeRefinementJob();

    if (!outcome.success) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanSync] 后台 refinement 失败，保留原始缓存")
            << QStringLiteral(" 段号=") << outcome.segmentIndex
            << QStringLiteral(" 说明=") << outcome.errorMessage;
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[ScanSync] 后台 refinement 完成")
        << QStringLiteral(" 段号=") << outcome.segmentIndex
        << QStringLiteral(" 任务ID=") << outcome.taskId
        << QStringLiteral(" 总耗时ms=") << outcome.processElapsedMs
        << QStringLiteral(" PCL处理ms=") << outcome.pclProcessElapsedMs
        << QStringLiteral(" 点数=") << outcome.rawPointCount << QStringLiteral("->") << outcome.processedPointCount;

    applySegmentRefinementOutcome(outcome);
}

void StateMachine::applySegmentRefinementOutcome(const SegmentProcessOutcome& outcome)
{
    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    auto resultIt = m_segmentCaptureResults.find(outcome.segmentIndex);
    if (resultIt == m_segmentCaptureResults.end()) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("忽略 refinement 结果：缓存段已不存在，段号=") << outcome.segmentIndex;
        return;
    }

    scan_tracking::mech_eye::releasePointCloudFrameBuffers(&resultIt->pointCloud);
    resultIt->pointCloud = clonePointCloudFrame(outcome.captureResult.pointCloud);

    auto bundleIt = m_segmentCaptureBundles.find(outcome.segmentIndex);
    if (bundleIt != m_segmentCaptureBundles.end()) {
        scan_tracking::mech_eye::releasePointCloudFrameBuffers(&bundleIt->mechEyeResult.pointCloud);
        bundleIt->mechEyeResult.pointCloud = clonePointCloudFrame(outcome.captureResult.pointCloud);
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SegmentCache] refinement 已写回内存")
        << QStringLiteral(" 段号=") << outcome.segmentIndex
        << QStringLiteral(" 点数=") << outcome.processedPointCount;
}

/**
 * @brief 执行位姿检查任务（Trig_PoseCheck）
 *
 * 直接调用 LB 位姿算法，输出位姿偏差值并按正式 PLC 结果码回写。
 */
void StateMachine::executePoseCheckTask()
{
    const QVector<double> identityRt = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    if (m_tracking == nullptr) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("位姿检查：Tracking 服务不可用。");
        writeFloatPlaceholder(protocol::registers::kPoseDeviationMm, 0.0f);
        completeActiveTask(7, protocol::AckState::Failed, false);
        emit poseCheckFinished(false, 7, 0.0, identityRt, QStringLiteral("跟踪服务不可用"));
        return;
    }

    const tracking::PoseCheckResult poseResult = m_tracking->checkPose();
    writeFloatPlaceholder(
        protocol::registers::kPoseDeviationMm,
        static_cast<float>(poseResult.poseDeviationMm));

    if (!poseResult.invoked) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("位姿检查未调用 LB 算法：")
            << poseResult.message;
        completeActiveTask(7, protocol::AckState::Failed, false);
        emit poseCheckFinished(false, 7, poseResult.poseDeviationMm, identityRt, poseResult.message);
        return;
    }

    QVector<double> rt;
    rt.reserve(static_cast<int>(poseResult.rt.size()));
    for (double value : poseResult.rt) {
        rt.append(value);
    }

    if (!poseResult.success) {
        const quint16 resultCode = poseResult.resultCode == 0 ? 7 : poseResult.resultCode;
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("位姿检查失败：")
            << poseResult.message
            << QStringLiteral(" resultCode=") << resultCode
            << QStringLiteral(" deviationMm=") << poseResult.poseDeviationMm;
        completeActiveTask(resultCode, protocol::AckState::Failed, false);
        emit poseCheckFinished(false, resultCode, poseResult.poseDeviationMm, rt, poseResult.message);
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("位姿检查成功")
        << QStringLiteral(" inputPoints=") << poseResult.inputPointCount
        << QStringLiteral(" deviationMm=") << poseResult.poseDeviationMm
        << QStringLiteral(" rt00=") << poseResult.rt[0]
        << QStringLiteral(" hasPoseMatrix=") << poseResult.hasPoseMatrix();
    completeActiveTask(1, protocol::AckState::Completed, true);
    emit poseCheckFinished(true, 1, poseResult.poseDeviationMm, rt, poseResult.message);
}

void StateMachine::executeSelfCheckTask()
{
    const bool modbusReady = m_modbus != nullptr && m_modbus->isConnected();
    const bool mechEyeReady = m_mechEye != nullptr && m_mechEye->state() != mech_eye::CameraRuntimeState::Error;
    const bool trackingReady = m_tracking != nullptr;
    const bool visionReady = m_visionPipeline != nullptr && m_visionPipeline->isStarted();

    QVector<quint16> failWords = {
        static_cast<quint16>(modbusReady ? 0 : (1u << 1)),
        static_cast<quint16>(mechEyeReady ? 0 : (1u << 0)),
    };
    if (!modbusReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：Modbus 不可用。");
    }
    if (!mechEyeReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：MechEye 不可用。");
    }
    if (!trackingReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：Tracking 不可用。");
    }
    if (!visionReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：视觉流水线不可用。");
    }

    const quint16 resultCode = (modbusReady && mechEyeReady && trackingReady && visionReady) ? 1 : 0;
    if (m_modbus && m_modbus->isConnected()) {
        m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord0, failWords);
        m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord1, {
            static_cast<quint16>(trackingReady ? 0 : (1u << 0)),
        });
    }
    completeActiveTask(resultCode, resultCode == 1 ? protocol::AckState::Completed : protocol::AckState::Failed, resultCode == 1);
    emit selfCheckFinished(resultCode, failWords.value(0));
}

void StateMachine::executeCodeReadTask()
{
    qInfo(LOG_FLOW).noquote() << QStringLiteral("收到 Trig_CodeRead，当前为占位实现。");
    if (m_modbus && m_modbus->isConnected()) {
        writeAsciiPlaceholder(protocol::registers::kCodeValueAscii, protocol::registers::kCodeValueRegisterCount, QStringLiteral("RD"));
    }
    completeActiveTask(9, protocol::AckState::Failed, false);
    emit codeReadFinished(9, QStringLiteral("RD"));
}

/**
 * @brief 执行综合检测任务（Trig_Inspection）
 * 
 * 调用跟踪检测服务对之前采集的所有分段点云进行综合分析，
 * 计算工件的偏移量和检测结果，并将结果写入 PLC 寄存器。
 * 
 * 关键步骤：
 * 1. 检查跟踪服务是否可用
 * 2. 调用 inspectSegments 进行点云分析
 * 3. 将检测结果写入 PLC
 * 4. 根据检测结果决定任务成功或失败
 * 5. 清空点云缓存（检测完成后不再需要原始点云）
 */
void StateMachine::setInspectionResultPublisher(
    std::function<void(const tracking::InspectionResult&)> publisher)
{
    m_inspectionResultPublisher = std::move(publisher);
}

QVector<int> StateMachine::cachedScanSegmentIndices() const
{
    QVector<int> indices;
    indices.reserve(m_segmentCaptureResults.size());
    for (auto it = m_segmentCaptureResults.cbegin(); it != m_segmentCaptureResults.cend(); ++it) {
        indices.push_back(it.key());
    }
    std::sort(indices.begin(), indices.end());
    return indices;
}

bool StateMachine::hasAllScanSegmentsCached() const
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const int scanSegmentTotal =
        configManager ? configManager->trackingConfig().scanSegmentTotal : 0;
    if (scanSegmentTotal <= 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    for (int segmentIndex = 1; segmentIndex <= scanSegmentTotal; ++segmentIndex) {
        const auto it = m_segmentCaptureResults.constFind(segmentIndex);
        if (it == m_segmentCaptureResults.cend() || !it->pointCloud.isValid()) {
            return false;
        }
    }
    return true;
}

const protocol::TriggerDefinition* StateMachine::selectPendingTrigger(
    const QVector<quint16>& commandBlock) const
{
    namespace regs = protocol::registers;
    const int scanTrigOffset = regs::modbusIndexFromPlcAddress(40023);
    const int inspectionTrigOffset = regs::modbusIndexFromPlcAddress(40024);
    const bool scanPending =
        scanTrigOffset < commandBlock.size() && commandBlock[scanTrigOffset] == 1;
    const bool inspectionPending =
        inspectionTrigOffset < commandBlock.size() && commandBlock[inspectionTrigOffset] == 1;

    // TODO(plc-handshake): PLC 程序应保证 Trig_ScanSegment 与 Trig_Inspection 互斥；全段齐时临时优先检测。
    if (scanPending && inspectionPending && hasAllScanSegmentsCached()) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[PLC容错] Trig_ScanSegment 与 Trig_Inspection 同时为 1，扫描段已齐，优先综合检测");
        return protocol::triggerByOffset(inspectionTrigOffset);
    }

    for (const auto& trigger : protocol::triggerDefinitions()) {
        if (trigger.trigOffset < commandBlock.size() && commandBlock[trigger.trigOffset] == 1) {
            return &trigger;
        }
    }
    return nullptr;
}

QMap<int, scan_tracking::mech_eye::CaptureResult> StateMachine::loadSegmentCaptureResultsForInspection(
    QString* errorMessage)
{
    int maxJoinMs = kBackgroundRefinementJoinTimeoutMs;
    if (m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::Inspection &&
        m_activeTask.timeoutSeconds > 0) {
        // 为综合检测主流程留出时间，避免 join 占满 PLC 超时（常见 60s）
        maxJoinMs = std::min(
            kBackgroundRefinementJoinTimeoutMs,
            std::max(5000, static_cast<int>(m_activeTask.timeoutSeconds) * 1000 - 5000));
    }
    joinAllBackgroundRefinementJobs(maxJoinMs);

    int pendingAfterJoin = pendingRefinementJobCount();
    if (pendingAfterJoin < 0) {
        reconcilePendingRefinementJobCounter("综合检测前计数异常");
        pendingAfterJoin = 0;
    } else if (pendingAfterJoin > 0) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("综合检测 join 超时后仍有 refinement 在途=") << pendingAfterJoin
            << QStringLiteral("，已强制复位；将使用当前缓存点云（可能为未 refine 的原始点云）");
        reconcilePendingRefinementJobCounter("综合检测前 join 未清空");
        pendingAfterJoin = 0;
    }

    QMap<int, scan_tracking::mech_eye::CaptureResult> loaded;
    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        for (auto it = m_segmentCaptureResults.cbegin(); it != m_segmentCaptureResults.cend(); ++it) {
            loaded.insert(it.key(), cloneCaptureResult(*it));
        }
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("综合检测失败：ConfigManager 不可用。");
        }
        return {};
    }

    const auto& tracking = configManager->trackingConfig();
    const QVector<int> requiredSegments = {
        tracking.firstStationOuterSegmentIndex,
        tracking.firstStationInnerSegmentIndex,
        tracking.firstStationHoleSegmentIndex,
    };

    for (int segmentIndex : requiredSegments) {
        const auto metaIt = loaded.constFind(segmentIndex);
        if (metaIt == loaded.cend() || !metaIt->pointCloud.isValid()) {
            if (errorMessage != nullptr) {
                QStringList cachedKeys;
                for (auto it = loaded.cbegin(); it != loaded.cend(); ++it) {
                    cachedKeys << QString::number(it.key());
                }
                *errorMessage = QStringLiteral(
                    "第一工位检测缺少必需分段 %1，配置段位 %2，已缓存段位 [%3]")
                                      .arg(segmentIndex)
                                      .arg(selectedSegmentTextForInspection(tracking))
                                      .arg(cachedKeys.join(QLatin1Char(',')));
            }
            return {};
        }
    }

    return loaded;
}

tracking::InspectionResult StateMachine::runDebugInspectionOnCachedSegments() const
{
    tracking::InspectionResult failure;
    failure.resultCode = 2;

    if (m_tracking == nullptr) {
        failure.ngReasonWord0 = (1u << 4);
        failure.message = QStringLiteral("调试综合检测失败：Tracking 服务不可用。");
        return failure;
    }

    if (m_segmentCaptureResults.isEmpty()) {
        failure.ngReasonWord0 = (1u << 4);
        failure.message = QStringLiteral("调试综合检测失败：点云缓存为空，请先完成扫描分段采集。");
        return failure;
    }

    QString loadError;
    auto* mutableSelf = const_cast<StateMachine*>(this);
    const auto segmentsForInspection = mutableSelf->loadSegmentCaptureResultsForInspection(&loadError);
    if (segmentsForInspection.isEmpty()) {
        failure.ngReasonWord0 = (1u << 4);
        failure.message = loadError.isEmpty()
            ? QStringLiteral("调试综合检测失败：无法加载必需分段点云。")
            : loadError;
        return failure;
    }

    return m_tracking->inspectSegments(segmentsForInspection, false);
}

void StateMachine::executeInspectionTask()
{
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
        resetPointCloudCache();  // 清空点云缓存
        return;
    }

    QString loadError;
    const auto segmentsForInspection = loadSegmentCaptureResultsForInspection(&loadError);
    if (segmentsForInspection.isEmpty()) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("Trig_Inspection 加载内存点云失败：") << loadError;
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
        // TODO(field-commissioning): 真实失败码为 7；PLC 侧临时强制 Res_Inspection=1(OK)
        const quint16 plcRes = inspectionResForPlcHandshake(7);
        completeActiveTask(plcRes, protocol::AckState::Completed, plcRes == kInspectionResOk);
        resetScanSegmentCache();
        return;
    }

    const tracking::InspectionResult trackingResult =
        m_tracking->inspectSegments(segmentsForInspection);

    // 将跟踪服务的检测结果转换为内部摘要结构
    InspectionSummary summary;
    summary.resultCode = trackingResult.resultCode;           // 结果码：1=合格，其他=不合格
    summary.ngReasonWord0 = trackingResult.ngReasonWord0;     // NG 原因字 0
    summary.ngReasonWord1 = trackingResult.ngReasonWord1;     // NG 原因字 1
    summary.measureItemCount = trackingResult.measureItemCount;  // 测量项数量
    summary.offsetXmm = trackingResult.offsetXmm;             // X 方向偏移量（mm）
    summary.offsetYmm = trackingResult.offsetYmm;             // Y 方向偏移量（mm）
    summary.offsetZmm = trackingResult.offsetZmm;             // Z 方向偏移量（mm）

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection 完成")
        << QStringLiteral(" 参与段数=") << trackingResult.segmentCount
        << QStringLiteral(" 总点数=") << trackingResult.totalPointCount
        << QStringLiteral(" 偏移Xmm=") << trackingResult.offsetXmm
        << QStringLiteral(" 偏移Ymm=") << trackingResult.offsetYmm
        << QStringLiteral(" 偏移Zmm=") << trackingResult.offsetZmm
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
        summary.measureItemCount, summary.offsetXmm, summary.offsetYmm, summary.offsetZmm,
        trackingResult.stableOffsetXmm, trackingResult.stableOffsetYmm, trackingResult.stableOffsetZmm,
        trackingResult.measurement,
        trackingResult.outlinerErrorLog, trackingResult.inlinerErrorLog,
        trackingResult.message);
    resetScanSegmentCache();  // 检测完成，清空扫描缓存释放内存
}

/**
 * @brief 执行结果复位任务（Trig_ResultReset）
 * 
 * 清空所有累积的点云缓存和检测结果，将相关寄存器归零，
 * 为下一轮扫描周期做准备。
 */
void StateMachine::executeResultResetTask()
{
    resetScanSegmentCache();  // 清空扫描缓存
    // 将扫描分段完成索引寄存器清零
    const bool segmentIndexCleared = m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {0, 0, 0});
    if (!segmentIndexCleared) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("清除扫描分段完成索引失败");
    }
    // 写入空的检测结果（全零）
    writeInspectionResult({});
    // 清除 IPC 安全动作字
    const bool safetyActionCleared = m_modbus->writeRegisters(protocol::registers::kIpcSafetyActionWord, {0});
    if (!safetyActionCleared) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("清除 IPC 安全动作字失败");
    }
    // 完成任务，返回成功
    completeActiveTask(1);
    emit resultResetFinished(1);
}

/**
 * @brief 向 PLC 发送 ACK（应答）信号
 * 
 * 将当前的应答状态写入触发定义中指定的 ACK 寄存器地址。
 * ACK 状态包括：Idle(0)、Running(1)、Completed(2)、Failed(3)。
 * 
 * @param definition 触发定义，包含 ACK 寄存器的偏移地址
 * @param ackState 要写入的应答状态
 */
void StateMachine::sendAck(const protocol::TriggerDefinition& definition, protocol::AckState ackState)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法发送
    }

    qDebug(LOG_FLOW).noquote() << QStringLiteral("写入 Ack") << static_cast<int>(ackState)
                               << QStringLiteral(" 至") << definition.ackOffset
                               << QStringLiteral(" 触发=") << protocol::triggerName(definition);
    const bool ackWritten = m_modbus->writeRegister(definition.ackOffset, static_cast<quint16>(ackState));
    if (!ackWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 Ack 状态失败");
    }
}

/**
 * @brief 向 PLC 发送 Res（结果）信号
 * 
 * 将任务执行的结果码写入触发定义中指定的 Res 寄存器地址。
 * 常见的结果码：1=成功，5=设备未就绪，6=超时，7=处理失败，9=参数错误。
 * 
 * @param definition 触发定义，包含 Res 寄存器的偏移地址
 * @param resultCode 要写入的结果码
 */
void StateMachine::sendRes(const protocol::TriggerDefinition& definition, quint16 resultCode)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法发送
    }

    qDebug(LOG_FLOW).noquote() << QStringLiteral("写入 Res") << resultCode
                               << QStringLiteral(" 至") << definition.resOffset
                               << QStringLiteral(" 触发=") << protocol::triggerName(definition);
    const bool resWritten = m_modbus->writeRegister(definition.resOffset, resultCode);
    if (!resWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 Res 结果码失败");
    }
}

/**
 * @brief 发布 IPC 状态到 PLC
 * 
 * 将当前的 IPC 运行状态、报警信息、进度等写入一组连续的 Modbus 寄存器，
 * 供 PLC 实时监控 IPC 的运行情况。
 * 
 * 写入的寄存器包括：
 * - 心跳计数器
 * - IPC 状态（Ready/Busy/Fault）
 * - 当前阶段（Idle/ScanSegment/Inspection 等）
 * - 报警级别和代码
 * - 警告代码
 * - 系统就绪标志
 * - 数据有效标志
 * - 任务进度百分比
 * - 设备在线状态字
 * - 当前任务 ID（高16位和低16位）
 */
void StateMachine::resetPlcOutputRegisters()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        qInfo(LOG_FLOW).noquote() << QStringLiteral("跳过程序退出寄存器复位：Modbus 未连接");
        return;
    }

    const bool cleared = m_modbus->resetIpcResultBlock();
    if (cleared) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("程序退出：IPC 结果区已清零（40101-40184，Ack/Res/状态/坐标等全部为 0）");
    } else {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("程序退出：IPC 结果区清零失败");
    }
}

void StateMachine::publishIpcStatus()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法发布状态
    }

    QVector<quint16> status = {
        m_heartbeatCounter,                                          // 0: 心跳计数器
        static_cast<quint16>(m_ipcState),                            // 1: IPC 状态
        static_cast<quint16>(m_currentStage),                        // 2: 当前阶段
        m_alarmLevel,                                                // 3: 报警级别
        m_alarmCode,                                                 // 4: 报警代码
        m_warnCode,                                                  // 5: 警告代码
        static_cast<quint16>(m_state == AppState::Ready ? 1 : 0),   // 6: 系统就绪标志
        static_cast<quint16>(m_dataValid ? 1 : 0),                  // 7: 数据有效标志
        m_progress,                                                  // 8: 任务进度（0-100）
        kDeviceOnlineWord0,                                          // 9: 设备在线状态字
        0,                                                           // 10: 保留
        0,                                                           // 11: 保留
        0,                                                           // 12: 保留
        static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu), // 13: 任务 ID 高16位
        static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),         // 14: 任务 ID 低16位
    };

    // 批量写入状态寄存器（从 kIpcHeartbeat 开始）
    const bool heartbeatWritten = m_modbus->writeRegisters(protocol::registers::kIpcHeartbeat, status);
    if (!heartbeatWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 IPC 心跳状态失败");
    }
}

/**
 * @brief 发布心跳信号
 * 
 * 递增心跳计数器并发布 IPC 状态，用于向 PLC 证明 IPC 仍在正常运行。
 * PLC 可以通过监控心跳计数器的变化来判断 IPC 是否死机或通信中断。
 */
void StateMachine::publishHeartbeat()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法发送心跳
    }

    ++m_heartbeatCounter;  // 递增心跳计数器
    publishIpcStatus();    // 发布包含新心跳计数的状态
}

/**
 * @brief 处理任务超时事件
 * 
 * 当任务执行时间超过设定的超时时间时触发此回调。
 * 设置超时报警，并根据任务类型采取不同的处理策略：
 * - 扫描分段任务：写入失败结果并标记为失败
 * - 其他任务：直接标记为完成但数据无效
 */
void StateMachine::onProcessTimeout()
{
    if (m_activeTask.definition == nullptr) {
        return;  // 没有活动任务，忽略超时
    }

    qWarning(LOG_FLOW).noquote() << QStringLiteral("任务超时：") << protocol::triggerName(*m_activeTask.definition);
    setAlarm(2, 610, QStringLiteral("任务超时"));  // 设置警告级别报警，代码 610
    m_activeTask.captureRequestId = 0;  // 清除采集请求 ID

    // P0修复：超时时清理已缓存的点云数据，防止内存泄漏
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        qWarning(LOG_FLOW) << QStringLiteral("任务超时，清空扫描分段内存缓存");
        resetScanSegmentCache();
    }

    // 根据任务类型采取不同的超时处理策略
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        // 扫描分段超时：写入空结果（0 图像数，0 点云帧数）
        writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
        completeActiveTask(6, protocol::AckState::Failed, false);  // Res=6 表示超时
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::Inspection) {
        // TODO(field-commissioning): 综合检测超时为唯一向 PLC 报 NG 的路径（Res_Inspection=6）
        writeInspectionResult({});
        completeActiveTask(
            kInspectionResTimeoutNg,
            protocol::AckState::Failed,
            false);
        resetScanSegmentCache();
        return;
    }
    // 其他任务超时：直接完成，标记为失败
    completeActiveTask(6, protocol::AckState::Completed, false);
}

/**
 * @brief 处理相机采集完成的回调
 * 
 * 当 Mech-Eye 相机完成一次 3D 点云采集后，通过此回调接收结果。
 * 这是扫描分段任务的核心处理逻辑。
 * 
 * 处理流程：
 * 1. 验证回调是否对应当前的活动任务和请求 ID
 * 2. 检查采集是否成功以及点云数据是否有效
 * 3. 如果成功，将点云结果存入缓存，更新进度，完成任务
 * 4. 如果失败，记录错误信息并完成失败的任务
 * 
 * @param result 采集结果，包含点云数据、状态码、错误信息等
 */
void StateMachine::onCaptureFinished(mech_eye::CaptureResult result)
{
    if (m_visionPipeline != nullptr) {
        return;
    }

    // 只在扫描分段阶段且存在活动任务时才处理
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment) {
        return;
    }

    // 过滤掉过期的回调（任务已完成或请求 ID 不匹配）
    if (m_activeTask.completionAnnounced || result.requestId != m_activeTask.captureRequestId) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("忽略过期梅卡采集回调，段号=") << m_activeTask.scanSegmentIndex;
        return;
    }

    // 检查采集是否成功、点云数据是否有效，以及法向量是否齐全
    if (!result.success() || !result.pointCloud.isValid() || !result.pointCloud.hasNormals()) {
        const QString failureMessage =
            result.errorMessage.isEmpty() ? QStringLiteral("采集失败") : result.errorMessage;
        finishScanSegmentFailure(
            mapCaptureErrorToResCode(result.errorCode),  // 映射错误码到 Res 码
            2,                                           // 报警级别：2 = 警告
            722,                                         // 报警代码：722 = 采集失败
            QStringLiteral("Trig_ScanSegment 采集失败：%1").arg(failureMessage),
            failureMessage);
        return;
    }

    scan_tracking::vision::MultiCameraCaptureBundle legacyBundle;
    legacyBundle.mechEyeResult = result;
    legacyBundle.request.segmentIndex = m_activeTask.scanSegmentIndex;
    legacyBundle.request.taskId = m_activeTask.taskId;

    const int segmentIndex = m_activeTask.scanSegmentIndex;
    const quint32 taskId = m_activeTask.taskId;
    const scan_tracking::common::PointCloudProcessingConfig processingConfig =
        scan_tracking::common::ConfigManager::instance()
            ? scan_tracking::common::ConfigManager::instance()->pointCloudProcessingConfig()
            : scan_tracking::common::PointCloudProcessingConfig{};

    commitScanSegmentCaptureImmediate(segmentIndex, result, legacyBundle);
    startSegmentBackgroundRefinement(segmentIndex, taskId, processingConfig);
}

/**
 * @brief 处理 Mech-Eye 相机致命错误（legacy 单相机路径）
 * 
 * 当相机发生无法恢复的错误（如硬件故障、驱动崩溃等）时触发此回调。
 * 如果当前正在执行扫描分段任务，立即终止任务并进入故障状态。
 * 
 * @param code 错误代码
 * @param message 错误描述信息
 */
void StateMachine::onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message)
{
    qCritical(LOG_FLOW) << "[MechEye] 致命错误:" << message;
    emit protocolEvent(QStringLiteral("Mech-Eye: %1").arg(message));  // 发出协议事件通知

    // 只在扫描分段阶段且任务未完成时才处理
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment ||
        m_activeTask.completionAnnounced) {
        return;
    }

    // P0修复：相机致命错误时清理已缓存的点云数据，防止内存泄漏
    qWarning(LOG_FLOW) << QStringLiteral("Mech-Eye 致命错误，清空扫描分段内存缓存");
    resetScanSegmentCache();

    // 相机在扫描中途发生致命错误时，需要第一时间拉高报警并强制结束当前扫描触发。
    finishScanSegmentFailure(
        mapCaptureErrorToResCode(code),  // 映射错误码到 Res 码
        3,                               // 报警级别：3 = 严重错误
        723,                             // 报警代码：723 = 相机致命错误
        QStringLiteral("扫描中 Mech-Eye 致命错误"),
        message);
}

/**
 * @brief 完成当前活动任务
 * 
 * 这是任务完成的统一出口，负责：
 * 1. 停止超时定时器
 * 2. 更新进度和数据有效性标志
 * 3. 向 PLC 发送 Res（结果码）和 ACK（应答状态）
 * 4. 标记任务已完成宣告
 * 5. 发布更新的 IPC 状态
 * 
 * @param resultCode 任务结果码（1=成功，其他=各种失败原因）
 * @param finalAckState 最终的 ACK 状态（Completed 或 Failed）
 * @param dataValid 数据是否有效（true 表示检测结果可用）
 */
/**
 * @brief 完成活动任务并发送结果和确认
 * 
 * P1修复：使用批量写入 Res 和 Ack，确保原子性，避免 PLC 读到中间状态。
 * 由于所有触发器的 Ack 和 Res 地址都是连续的（Ack 在前，Res 在后），
 * 可以使用一次 writeRegisters 调用同时写入两个寄存器。
 * 
 * @param resultCode 结果代码
 * @param finalAckState 最终 ACK 状态（Completed=2 或 Failed=3）
 * @param dataValid 数据是否有效标志
 */
bool StateMachine::completeActiveTask(
    quint16 resultCode,
    protocol::AckState finalAckState,
    bool dataValid)
{
    if (m_activeTask.definition == nullptr || !m_modbus) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("无法完成任务：任务定义或 Modbus 为空");
        return false;  // 没有活动任务或 Modbus 不可用，无法完成
    }

    // P3改进：封装带重试的 Modbus 写入 lambda
    auto executeWithRetry = [this](auto&& writeOperation, const QString& operationName) -> bool {
        constexpr int kMaxRetries = 3;
        constexpr int kRetryIntervalMs = 100;
        
        for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
            // P3改进：每次重试前检查必要前提条件
            if (!m_modbus) {
                qWarning(LOG_FLOW).noquote()
                    << operationName << QStringLiteral(" 失败：第") << attempt << QStringLiteral(" 次重试时 Modbus 已为空");
                return false;
            }
            
            if (m_activeTask.definition == nullptr) {
                qWarning(LOG_FLOW).noquote()
                    << operationName << QStringLiteral(" 失败：第") << attempt << QStringLiteral(" 次重试时任务定义已为空");
                return false;
            }
            
            // 执行写入操作
            writeOperation();
            
            // P3改进：如果是最后一次尝试，直接返回成功（已发起写入请求）
            if (attempt == kMaxRetries) {
                return true;
            }
            
            // P3改进：非最后一次尝试，等待后继续重试
            QThread::msleep(kRetryIntervalMs);
        }
        
        return true;  // 理论上不会到达这里
    };
    
    // P1修复：批量写入 Res 和 Ack，保证原子性
    // P2改进：虽然批量写入是原子的，但为了确保语义清晰，我们明确标注写入顺序
    // 在 Modbus 批量写入中，所有寄存器在同一事务中原子更新，PLC 会同时看到新值
    // 地址布局：ackOffset (低位) -> resOffset (高位)，两者必须连续
    const int ackOffset = m_activeTask.definition->ackOffset;
    const int resOffset = m_activeTask.definition->resOffset;
    
    bool writeSuccess = false;
    auto failCompletionWrite = [this](const QString& reason) -> bool {
        qWarning(LOG_FLOW).noquote() << reason;
        enterFaultState(902, reason, false, false);
        return false;
    };
    
    // 验证地址连续性（防御性编程）
    if (resOffset == ackOffset + 1) {
        // 地址连续，使用批量写入
        QVector<quint16> batchValues = {
            static_cast<quint16>(finalAckState),  // Ack 值
            resultCode                             // Res 值
        };
        
        // P3改进：带重试的批量写入
        writeSuccess = executeWithRetry(
            [&]() {
                m_modbus->writeRegisters(ackOffset, batchValues);
            },
            QStringLiteral("批量写入寄存器"));
        
        if (!writeSuccess) {
            return failCompletionWrite(QStringLiteral(
                "批量写入重试后仍失败：Ack=%1 Res=%2 地址 %3-%4")
                .arg(static_cast<int>(finalAckState))
                .arg(resultCode)
                .arg(ackOffset)
                .arg(resOffset));
        }
        
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("原子批量写入 Ack=") << static_cast<int>(finalAckState)
            << QStringLiteral(" Res=") << resultCode
            << QStringLiteral(" 地址") << ackOffset << QStringLiteral("-") << resOffset;
    } else {
        // 地址不连续（异常情况），降级为单独写入
        // P2改进：确保先写 Res 再写 Ack，避免 PLC 读到中间状态
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Ack 与 Res 地址不连续：")
            << QStringLiteral(" ackOffset=") << ackOffset << QStringLiteral(" resOffset=") << resOffset
            << QStringLiteral("，降级为单独写入并重试。");
        
        // P3改进：先写 Res（结果数据），带重试
        bool resSuccess = executeWithRetry(
            [&]() {
                sendRes(*m_activeTask.definition, resultCode);
            },
            QStringLiteral("发送 Res"));
        
        if (!resSuccess) {
            return failCompletionWrite(QStringLiteral(
                "Res 重试后仍发送失败：resultCode=%1")
                .arg(resultCode));
        }
        
        // P3改进：再写 Ack（完成标志），带重试
        bool ackSuccess = executeWithRetry(
            [&]() {
                sendAck(*m_activeTask.definition, finalAckState);
            },
            QStringLiteral("发送 Ack"));
        
        if (!ackSuccess) {
            return failCompletionWrite(QStringLiteral(
                "Ack 重试后仍发送失败：ackState=%1")
                .arg(static_cast<int>(finalAckState)));
        }
    }
    
    // P3改进：只有 Modbus 操作成功发起后，才更新状态
    m_timeoutTimer->stop();   // 停止超时定时器
    m_progress = 100;         // 进度设为 100%
    m_dataValid = dataValid;  // 设置数据有效性标志
    m_activeTask.completionAnnounced = true;                    // 标记已完成宣告
    m_activeTask.captureRequestId = 0;                          // 清除采集请求 ID
    publishIpcStatus();                                         // 发布更新的 IPC 状态

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("触发已完成") << protocol::triggerName(*m_activeTask.definition)
        << QStringLiteral(" Res=") << resultCode
        << QStringLiteral(" Ack=") << static_cast<int>(finalAckState);
    
    return true;
}

/**
 * @brief 在 PLC 释放触发信号后 finalize 已完成的任务
 * 
 * 当任务完成后，PLC 需要将对应的 Trig 位清零以确认收到结果。
 * 此方法检测 Trig 位是否已清零，如果是则回零 Ack/Res 并清除活动任务状态，
 * 使系统回到就绪状态等待下一个触发。
 * 
 * @param commandBlock 最新的命令块寄存器数据
 */
void StateMachine::finalizeCompletedTaskIfTriggerReleased(const QVector<quint16>& commandBlock)
{
    if (m_activeTask.definition == nullptr || !m_activeTask.completionAnnounced) {
        return;  // 没有活动任务或任务未完成宣告，无需处理
    }

    const int trigOffset = m_activeTask.definition->trigOffset;
    // 检查 Trig 位是否已清零（PLC 确认收到结果）
    if (trigOffset >= commandBlock.size() || commandBlock[trigOffset] != 0) {
        return;  // Trig 位仍为 1，PLC 尚未释放
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
        bool released = false;
        if (resOffset == ackOffset + 1) {
            released = m_modbus->writeRegisters(ackOffset, {
                static_cast<quint16>(protocol::AckState::Idle),
                0,
            });
        } else {
            sendRes(definition, 0);
            sendAck(definition, protocol::AckState::Idle);
            released = true;
        }
        if (!released) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("释放触发后回零 Ack/Res 失败：") << protocol::triggerName(definition);
        } else {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("握手释放完成") << protocol::triggerName(definition)
                << QStringLiteral(" Ack=0 Res=0");
        }
    }

    clearActiveTask();                                            // 清除活动任务
    m_ipcState = protocol::IpcState::Ready;                       // IPC 状态回到就绪
    m_currentStage = protocol::Stage::Idle;                       // 当前阶段回到空闲
    m_progress = 0;                                               // 进度归零
    setState(AppState::Ready);                                    // 应用状态回到就绪
    publishIpcStatus();                                           // 发布更新的 IPC 状态
}

/**
 * @brief 清除当前活动任务的所有状态
 * 
 * 将活动任务结构体重置为默认值，释放所有相关资源。
 */
void StateMachine::clearActiveTask()
{
    m_activeTask = {};  // 重置为默认构造的空任务
}

/**
 * @brief 设置报警信息
 * 
 * 更新报警级别、报警代码和警告代码，并发出协议事件通知。
 * 
 * @param level 报警级别（0=无报警，1=提示，2=警告，3=严重错误）
 * @param code 报警代码
 * @param message 报警描述信息
 */
void StateMachine::setAlarm(quint16 level, quint16 code, const QString& message)
{
    m_alarmLevel = level;
    m_alarmCode = code;
    // 警告代码仅在报警级别为 1 或 2 时有效
    m_warnCode = level > 0 && level < 3 ? code : 0;
    if (!message.isEmpty()) {
        emit protocolEvent(message);  // 发出协议事件通知外部监听者
    }
}

/**
 * @brief 向 PLC 写入浮点数占位符（CDAB 字节序）
 * 
 * 将单个 float 值转换为两个 16 位寄存器并写入指定的起始地址。
 * 用于向 PLC 传递坐标、角度等浮点数据。
 * 
 * @param startOffset 起始寄存器偏移地址
 * @param value 要写入的浮点数值
 */
void StateMachine::writeFloatPlaceholder(int startOffset, float value)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    const bool floatWritten = m_modbus->writeRegisters(startOffset, floatToCdabRegisters(value));
    if (!floatWritten) {
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

/**
 * @brief 向 PLC 写入 ASCII 字符串占位符
 * 
 * 将字符串按每两个字符打包为一个 16 位寄存器的方式写入 PLC。
 * 如果字符串长度不足，用空格填充；如果超长，则截断。
 * 
 * @param startOffset 起始寄存器偏移地址
 * @param registerCount 要写入的寄存器数量
 * @param text 要写入的文本字符串
 */
void StateMachine::writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    // 限制字符串长度并用空格右对齐填充
    const QString padded = text.left(registerCount * 2).leftJustified(registerCount * 2, QLatin1Char(' '));
    QVector<quint16> values;
    values.reserve(registerCount);
    for (int i = 0; i < registerCount; ++i) {
        // 每两个字符打包为一个 16 位寄存器：高8位为第一个字符，低8位为第二个字符
        const QChar first = padded.at(i * 2);
        const QChar second = padded.at(i * 2 + 1);
        const quint16 packed = (static_cast<quint16>(first.unicode()) << 8) |
                               static_cast<quint16>(second.unicode() & 0xFF);
        values.push_back(packed);
    }
    const bool asciiWritten = m_modbus->writeRegisters(startOffset, values);
    if (!asciiWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 ASCII 占位符失败，偏移=") << startOffset;
    }
}

/**
 * @brief 写入加载抓取任务的模拟结果
 * 
 * 向 PLC 写入预设的加载位姿数据（位置和姿态角）。
 * 这是一个占位实现，实际应用中应替换为真实的视觉定位结果。
 */
void StateMachine::writeLoadGraspResult()
{
    const auto poseSource = resolveLoadGraspPoseSource();
    writeFloatPlaceholder(protocol::registers::kLoadX, poseSource.x);
    writeFloatPlaceholder(protocol::registers::kLoadY, poseSource.y);
    writeFloatPlaceholder(protocol::registers::kLoadZ, poseSource.z);
    writeFloatPlaceholder(protocol::registers::kLoadRx, poseSource.rx);
    writeFloatPlaceholder(protocol::registers::kLoadRy, poseSource.ry);
    writeFloatPlaceholder(protocol::registers::kLoadRz, poseSource.rz);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("LoadGrasp 位姿已写入")
        << QStringLiteral(" source=") << poseSource.sourceName
        << QStringLiteral(" x=") << poseSource.x
        << QStringLiteral(" y=") << poseSource.y
        << QStringLiteral(" z=") << poseSource.z
        << QStringLiteral(" rx=") << poseSource.rx
        << QStringLiteral(" ry=") << poseSource.ry
        << QStringLiteral(" rz=") << poseSource.rz;
}

/**
 * @brief 写入卸载计算任务的模拟结果
 * 
 * 向 PLC 写入预设的卸料位姿数据（位置和姿态角）。
 * 这是一个占位实现，实际应用中应替换为真实的路径规划结果。
 */
void StateMachine::writeUnloadCalcResult()
{
    const auto poseSource = resolveUnloadCalcPoseSource();
    writeFloatPlaceholder(protocol::registers::kUnloadX, poseSource.x);
    writeFloatPlaceholder(protocol::registers::kUnloadY, poseSource.y);
    writeFloatPlaceholder(protocol::registers::kUnloadZ, poseSource.z);
    writeFloatPlaceholder(protocol::registers::kUnloadRx, poseSource.rx);
    writeFloatPlaceholder(protocol::registers::kUnloadRy, poseSource.ry);
    writeFloatPlaceholder(protocol::registers::kUnloadRz, poseSource.rz);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("UnloadCalc 位姿已写入")
        << QStringLiteral(" source=") << poseSource.sourceName
        << QStringLiteral(" x=") << poseSource.x
        << QStringLiteral(" y=") << poseSource.y
        << QStringLiteral(" z=") << poseSource.z
        << QStringLiteral(" rx=") << poseSource.rx
        << QStringLiteral(" ry=") << poseSource.ry
        << QStringLiteral(" rz=") << poseSource.rz;
}

/**
 * @brief 写入扫描分段结果到 PLC
 * 
 * 将当前分段的索引、采集的图像数量和点云帧数量写入 PLC 寄存器，
 * 供 PLC 跟踪扫描进度。
 * 
 * @param segmentIndex 分段索引（从1开始）
 * @param imageCount 采集的图像数量（通常为1）
 * @param cloudFrameCount 采集的点云帧数量（通常为1）
 */
void StateMachine::writeScanSegmentResult(int segmentIndex, int imageCount, int cloudFrameCount)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    const bool progressWritten = m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {
        static_cast<quint16>(segmentIndex),     // 已完成的分段索引
        static_cast<quint16>(imageCount),       // 该分段的图像数量
        static_cast<quint16>(cloudFrameCount),  // 该分段的点云帧数量
    });
    if (!progressWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入扫描分段进度失败");
    }
}

/**
 * @brief 写入综合检测结果到 PLC
 * 
 * 将检测结果的 NG 原因字、测量项数量等写入 PLC 寄存器，
 * 供 PLC 判断工件是否合格以及获取详细的缺陷信息。
 * 
 * @param summary 检测结果摘要结构
 */
void StateMachine::writeInspectionResult(const InspectionSummary& summary)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    // 写入 NG 原因字和测量项数量
    const bool inspectionWritten = m_modbus->writeRegisters(protocol::registers::kNgReasonWord0, {
        summary.ngReasonWord0,      // NG 原因字 0（位掩码表示各种缺陷类型）
        summary.ngReasonWord1,      // NG 原因字 1（扩展缺陷类型）
        summary.measureItemCount,   // 实际测量的项目数量
    });
    if (!inspectionWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入检测结果失败");
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("检测结果寄存器已写入")
        << QStringLiteral(" ngReasonWord0=") << summary.ngReasonWord0
        << QStringLiteral(" ngReasonWord1=") << summary.ngReasonWord1
        << QStringLiteral(" measureItemCount=") << summary.measureItemCount
        << QStringLiteral(" offsetXmm=") << summary.offsetXmm   // X 方向偏移（未在寄存器中写入，仅记录日志）
        << QStringLiteral(" offsetYmm=") << summary.offsetYmm   // Y 方向偏移
        << QStringLiteral(" offsetZmm=") << summary.offsetZmm;  // Z 方向偏移
}

/**
 * @brief 重置点云缓存
 * 
 * 清空所有累积的扫描分段点云数据，释放内存。
 * 在以下情况下调用：
 * - 综合检测完成后（点云已被消费）
 * - 结果复位任务执行时
 * - 系统启动或停止时
 * - 发生故障需要清理状态时
 */
void StateMachine::resetPointCloudCache()
{
    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    const int cacheSize = m_segmentCaptureResults.size();
    for (auto it = m_segmentCaptureResults.begin(); it != m_segmentCaptureResults.end(); ++it) {
        scan_tracking::mech_eye::releasePointCloudFrameBuffers(&it->pointCloud);
    }
    m_segmentCaptureResults.clear();

    if (cacheSize > 0) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("已清空内存点云缓存，条目数=") << cacheSize;
    }
}

void StateMachine::resetScanSegmentCache()
{
    joinAllBackgroundRefinementJobs();
    if (pendingRefinementJobCount() != 0) {
        reconcilePendingRefinementJobCounter("resetScanSegmentCache");
    }

    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    for (auto it = m_segmentCaptureBundles.begin(); it != m_segmentCaptureBundles.end(); ++it) {
        scan_tracking::vision::releaseHikMonoFrameBuffers(&it->hikCameraAResult.frame);
        scan_tracking::vision::releaseHikMonoFrameBuffers(&it->hikCameraBResult.frame);
        scan_tracking::mech_eye::releasePointCloudFrameBuffers(&it->mechEyeResult.pointCloud);
    }
    m_segmentCaptureBundles.clear();

    const int cacheSize = m_segmentCaptureResults.size();
    for (auto it = m_segmentCaptureResults.begin(); it != m_segmentCaptureResults.end(); ++it) {
        scan_tracking::mech_eye::releasePointCloudFrameBuffers(&it->pointCloud);
    }
    m_segmentCaptureResults.clear();

    if (cacheSize > 0) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("已清空内存点云缓存，条目数=") << cacheSize;
    }

    m_segmentCalibrationMatrices.clear();
    m_currentCalibrationMatrix = m_baseCalibrationMatrix;
}

void StateMachine::reloadCalibrationMatricesFromConfig()
{
    m_baseCalibrationMatrix = identityMatrix4x4();
    m_currentCalibrationMatrix = m_baseCalibrationMatrix;

    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr == nullptr) {
        return;
    }

    const auto& t0 = configMgr->scanPathsConfig().calibrationMatrixT0;
    bool hasNonIdentity = false;
    for (std::size_t i = 0; i < t0.size(); ++i) {
        if (i == 0 || i == 5 || i == 10 || i == 15) {
            if (std::abs(t0[i] - 1.0f) > 1e-6f) {
                hasNonIdentity = true;
            }
        } else if (std::abs(t0[i]) > 1e-6f) {
            hasNonIdentity = true;
        }
    }
    if (hasNonIdentity) {
        m_baseCalibrationMatrix = t0;
        m_currentCalibrationMatrix = t0;
    }

    qInfo(LOG_FLOW).noquote() << QStringLiteral("已从 scan_paths_config 加载标定矩阵 T0");
}

bool StateMachine::resolveNeedRotationForSegment(int segmentIndex) const
{
    return lookupNeedRotationForSegment(segmentIndex);
}

void StateMachine::applyLbnCalibrationUpdate(
    int segmentIndex,
    bool needRotation,
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    if (!needRotation) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("LBN 标定：段号=") << segmentIndex
            << QStringLiteral(" 无需转盘，保持当前 T0'");
        return;
    }

    const auto& lbn = bundle.lbnPoseResult;
    if (!lbn.invoked) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("LBN 标定：段号=") << segmentIndex
            << QStringLiteral(" 未调用 LBN，保持当前 T0'");
        return;
    }
    const bool useIdentityBypass = []() {
        const auto* cfg = scan_tracking::common::ConfigManager::instance();
        return cfg && cfg->lbnPoseConfig().useIdentityRtWithoutMarkers;
    }();

    if (!lbn.success || !lbn.poseMatrix.isValid()) {
        if (!useIdentityBypass) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("LBN 标定：段号=") << segmentIndex
                << QStringLiteral(" 失败，保持当前 T0'：") << lbn.message;
            return;
        }
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("LBN 标定：段号=") << segmentIndex
            << QStringLiteral(" 失败，TODO(marker) 回退 Rt=单位阵：") << lbn.message;
    }

    const auto rt = (lbn.success && lbn.poseMatrix.isValid())
        ? poseMatrixToArray(lbn.poseMatrix)
        : identityMatrix4x4();
    m_currentCalibrationMatrix = multiplyRowMajor4x4(rt, m_currentCalibrationMatrix);
    m_segmentCalibrationMatrices.insert(segmentIndex, m_currentCalibrationMatrix);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("LBN 标定已更新 T0'，段号=") << segmentIndex
        << QStringLiteral(" 匹配点数=") << lbn.matchedPointCount;
    for (int row = 0; row < 4; ++row) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("  T0'[%1] %2 %3 %4 %5")
                   .arg(row)
                   .arg(m_currentCalibrationMatrix[static_cast<std::size_t>(row * 4 + 0)], 0, 'g', 6)
                   .arg(m_currentCalibrationMatrix[static_cast<std::size_t>(row * 4 + 1)], 0, 'g', 6)
                   .arg(m_currentCalibrationMatrix[static_cast<std::size_t>(row * 4 + 2)], 0, 'g', 6)
                   .arg(m_currentCalibrationMatrix[static_cast<std::size_t>(row * 4 + 3)], 0, 'g', 6);
    }
}

/**
 * @brief 记录 Modbus 通信失败
 * 
 * 递增连续失败计数器，当达到阈值时自动进入故障状态。
 * 这种机制可以容忍偶发的通信干扰，但对持续故障做出快速响应。
 * 
 * @param alarmCode 报警代码
 * @param message 错误描述信息
 */
void StateMachine::recordModbusFailure(quint16 alarmCode, const QString& message)
{
    ++m_consecutiveModbusFailures;
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("记录 Modbus 失败")
        << m_consecutiveModbusFailures << QStringLiteral("/") << kMaxConsecutiveModbusFailures
        << QStringLiteral(" alarmCode=") << alarmCode
        << QStringLiteral(" reason=") << message;

    // 如果连续失败次数达到阈值，进入故障状态
    if (m_consecutiveModbusFailures >= kMaxConsecutiveModbusFailures) {
        enterFaultState(alarmCode, message, true, true);
    }
}

/**
 * @brief 重置 Modbus 失败计数器
 * 
 * 在成功通信后调用，将连续失败计数器归零。
 */
void StateMachine::resetModbusFailureCounter()
{
    if (m_consecutiveModbusFailures > 0) {
        qInfo(LOG_FLOW) << QStringLiteral("通信成功后重置 Modbus 失败计数器。");
    }
    m_consecutiveModbusFailures = 0;
}

/**
 * @brief 进入故障状态
 * 
 * 设置报警信息，切换应用状态为 Error，并根据参数决定是否中止当前任务。
 * 这是系统处理严重错误的统一入口。
 * 
 * @param alarmCode 报警代码
 * @param message 错误描述信息
 * @param abortCurrentTask 是否中止当前活动任务
 * @param notifyPlc 是否通知 PLC（通过发送 Res/Ack）
 */
void StateMachine::enterFaultState(
    quint16 alarmCode,
    const QString& message,
    bool abortCurrentTask,
    bool notifyPlc)
{
    setAlarm(3, alarmCode, message);       // 设置严重错误级别报警
    m_ipcState = protocol::IpcState::Fault; // IPC 状态设为故障
    setState(AppState::Error);              // 应用状态设为错误

    if (abortCurrentTask) {
        abortActiveTaskForFault(7);  // 中止当前任务，Res=7 表示处理失败
    } else {
        m_timeoutTimer->stop();      // 停止超时定时器
        m_progress = 0;              // 进度归零
        m_currentStage = protocol::Stage::Idle;  // 阶段回到空闲
        publishIpcStatus();          // 发布故障状态
    }

    if (!notifyPlc) {
        clearActiveTask();           // 清除活动任务
        m_currentStage = protocol::Stage::Idle;
    }
}

/**
 * @brief 因故障中止当前活动任务
 * 
 * 根据任务类型采取不同的中止策略：
 * - 扫描分段任务：先写入空结果再完成
 * - 其他任务：直接清理状态
 * 如果 Modbus 可用，通过正常的 completeActiveTask 流程通知 PLC；
 * 否则直接清理本地状态。
 * 
 * @param resultCode 任务结果码（通常为 7 = 处理失败）
 */
void StateMachine::abortActiveTaskForFault(quint16 resultCode)
{
    if (m_activeTask.definition == nullptr) {
        // 没有活动任务，只需清理基本状态
        m_timeoutTimer->stop();
        m_progress = 0;
        m_dataValid = false;
        m_currentStage = protocol::Stage::Idle;
        publishIpcStatus();
        return;
    }

    // 如果是扫描分段任务，先写入空结果
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
    }

    // 如果 Modbus 可用，通过正常流程通知 PLC
    if (m_modbus && m_modbus->isConnected()) {
        completeActiveTask(resultCode, protocol::AckState::Failed, false);
        return;
    }

    // Modbus 不可用时，直接清理本地状态
    m_timeoutTimer->stop();
    m_progress = 0;
    m_dataValid = false;
    m_activeTask.captureRequestId = 0;
    m_activeTask.completionAnnounced = false;
    clearActiveTask();
    m_currentStage = protocol::Stage::Idle;
    publishIpcStatus();
}

/**
 * @brief 将相机采集错误码映射为 PLC 结果码
 * 
 * 将 Mech-Eye 相机的内部错误码转换为 PLC 能理解的结果码：
 * - 1: 成功
 * - 5: 设备未就绪（连接失败、忙等）
 * - 6: 超时
 * - 7: 处理失败（未知错误）
 * - 9: 参数错误（无效请求）
 * 
 * @param errorCode 相机采集错误码
 * @return 对应的 PLC 结果码
 */
quint16 StateMachine::mapCaptureErrorToResCode(mech_eye::CaptureErrorCode errorCode) const
{
    switch (errorCode) {
    case mech_eye::CaptureErrorCode::Success:
        return 1;   // 成功
    case mech_eye::CaptureErrorCode::NotStarted:
    case mech_eye::CaptureErrorCode::NotConnected:
    case mech_eye::CaptureErrorCode::Busy:
    case mech_eye::CaptureErrorCode::DiscoverFailed:
    case mech_eye::CaptureErrorCode::ConnectFailed:
    case mech_eye::CaptureErrorCode::DisconnectFailed:
        return 5;   // 设备未就绪
    case mech_eye::CaptureErrorCode::Timeout:
        return 6;   // 超时
    case mech_eye::CaptureErrorCode::InvalidRequest:
        return 9;   // 参数错误
    default:
        return 7;   // 其他错误归为处理失败
    }
}

/**
 * @brief 从命令块中读取任务 ID
 * 
 * 任务 ID 是一个 32 位整数，存储在两个 16 位寄存器中（高16位和低16位）。
 * 
 * @param commandBlock 命令块寄存器数据
 * @return 32 位任务 ID
 */
quint32 StateMachine::readTaskId(const QVector<quint16>& commandBlock) const
{
    const quint32 high = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdHigh));
    const quint32 low = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdLow));
    return (high << 16) | low;  // 组合高16位和低16位
}

/**
 * @brief 解析扫描分段索引
 * 
 * 从命令块中读取当前请求的扫描分段索引。
 * 40015 为段号；PLC 常把 REAL（如 1.0f）以未转字序的原始字写入（如 16256=0x3F80）。
 * 通过 plcAnalogToUInt16 解码，不再固定按 40015/40016 高低字拼接。
 *
 * @param commandBlock 命令块寄存器数据
 * @return 扫描分段索引（从1开始）
 */
quint16 StateMachine::resolveScanSegmentIndex(const QVector<quint16>& commandBlock) const
{
    return protocol::registers::resolveScanSegmentIndexFromBlock(commandBlock);
}

/**
 * @brief 验证扫描分段请求的合法性
 * 
 * 检查分段索引是否在有效范围内，以及是否已经采集过该分段（重复检测）。
 * 
 * @param commandBlock 命令块寄存器数据
 * @param errorMessage 输出参数，如果验证失败则填充错误描述信息
 * @return true 如果请求合法，false 如果请求非法
 */
bool StateMachine::validateScanSegmentRequest(const QVector<quint16>& commandBlock, QString* errorMessage)
{
    const int segmentIndex = resolveScanSegmentIndex(commandBlock);  // 获取分段索引（32位合并）
    // ScanSegmentTotal 从配置文件获取（PLC 不再下发此字段）
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const int segmentTotal = configManager != nullptr
        ? configManager->trackingConfig().scanSegmentTotal
        : 0;
    // 计算允许的最大段号：不超过总段数和系统上限的最小值
    const int maxSegmentIndex = segmentTotal > 0 ? qMin(segmentTotal, kMaxScanSegmentIndex)
                                                 : kMaxScanSegmentIndex;

    // 检查段号是否在有效范围内 [1, maxSegmentIndex]
    if (segmentIndex < 1 || segmentIndex > maxSegmentIndex) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("扫描段号无效：段号=%1，总段数=%2，允许范围 1-%3")
                .arg(segmentIndex)
                .arg(segmentTotal)
                .arg(maxSegmentIndex);
        }
        qWarning(LOG_FLOW).noquote() << QStringLiteral("拒绝 Trig_ScanSegment：段号无效")
                                     << QStringLiteral(" 段号=") << segmentIndex
                                     << QStringLiteral(" 总段数=") << segmentTotal
                                     << QStringLiteral(" 允许最大=") << maxSegmentIndex;
        return false;
    }

    // 检查是否已经采集过该分段（防止重复触发污染缓存）
    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        if (m_segmentCaptureResults.contains(segmentIndex)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("重复扫描段号：%1").arg(segmentIndex);
            }
            qWarning(LOG_FLOW).noquote() << QStringLiteral("拒绝 Trig_ScanSegment：段号已缓存")
                                         << QStringLiteral(" 段号=") << segmentIndex
                                         << QStringLiteral(" 当前缓存段数=") << m_segmentCaptureResults.size();
            return false;
        }

        // P2改进：检查缓存大小是否达到上限，防止内存无限增长
        if (m_segmentCaptureResults.size() >= kMaxPointCloudCacheSize) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("点云内存缓存已满：当前 %1 段，上限 %2")
                    .arg(m_segmentCaptureResults.size())
                    .arg(kMaxPointCloudCacheSize);
            }
            qWarning(LOG_FLOW).noquote() << QStringLiteral("拒绝 Trig_ScanSegment：内存缓存已满")
                                         << QStringLiteral(" 当前=") << m_segmentCaptureResults.size()
                                         << QStringLiteral(" 上限=") << kMaxPointCloudCacheSize;
            return false;
        }
    }

    return true;  // 验证通过
}

/**
 * @brief 完成扫描分段失败的处理
 * 
 * 统一的扫描分段失败出口，确保按照正确的顺序写入结果：
 * 1. 设置报警信息
 * 2. 写入空结果（0 图像数，0 点云帧数）
 * 3. 清除采集请求 ID
 * 4. 完成任务并标记为失败
 * 
 * 这样可以保证 PLC 不会读到旧数据或 inconsistent 的状态。
 * 
 * @param resultCode 结果码（5=设备未就绪，6=超时，7=处理失败，9=参数错误）
 * @param alarmLevel 报警级别（2=警告，3=严重错误）
 * @param alarmCode 报警代码
 * @param logMessage 日志消息（详细的技术描述）
 * @param alarmMessage 报警消息（显示给用户的简洁描述）
 */
void StateMachine::finishScanSegmentFailure(
    quint16 resultCode,
    quint16 alarmLevel,
    quint16 alarmCode,
    const QString& logMessage,
    const QString& alarmMessage)
{
    // 失败闭环也必须先写结果区，再写 Res，最后写 Ack=3，保证 PLC 不会读到旧数据。
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ScanSegment 失败")
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex
        << QStringLiteral(" Res=") << resultCode
        << QStringLiteral(" 原因=") << logMessage;
    setAlarm(alarmLevel, alarmCode, alarmMessage);              // 设置报警
    writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0); // 写入空结果
    
    // P0修复：严重错误时清理已缓存的扫描数据，防止内存泄漏
    if (resultCode >= 5) {
        qWarning(LOG_FLOW) << QStringLiteral("扫描失败，清空分段内存缓存，Res=") << resultCode;
        resetScanSegmentCache();
    }
    
    m_activeTask.captureRequestId = 0;                          // 清除采集请求 ID
    completeActiveTask(resultCode, protocol::AckState::Failed, false);  // 完成失败任务
}

}  // namespace scan_tracking::flow_control
