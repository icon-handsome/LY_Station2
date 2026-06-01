#include "scan_tracking/vision/hik_camera_c_controller.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QThread>

#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/vision/hik_smart_camera_tcp_server.h"
#include "scan_tracking/vision/hik_smart_camera_ftp_monitor.h"

Q_LOGGING_CATEGORY(hikCControllerLog, "vision.hik_camera_c_controller")

namespace scan_tracking {
namespace vision {

void HikCameraCController::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    qRegisterMetaType<scan_tracking::vision::HikCameraCState>("scan_tracking::vision::HikCameraCState");
    qRegisterMetaType<scan_tracking::vision::CaptureType>("scan_tracking::vision::CaptureType");
    qRegisterMetaType<scan_tracking::vision::ImageFileInfo>("scan_tracking::vision::ImageFileInfo");    registered = true;
}

HikCameraCController::HikCameraCController(
    HikCameraService* hikCameraCService,
    QObject* parent)
    : QObject(parent)
    , m_hikCameraCService(hikCameraCService)
    , m_tcpServer(nullptr)
    , m_testCaptureTimer(nullptr)
{
    registerMetaTypes();
}

HikCameraCController::~HikCameraCController()
{
    cleanupFtpMonitor();
    cleanupTcpServer();

    if (m_testCaptureTimer) {
        m_testCaptureTimer->stop();
        m_testCaptureTimer->setParent(nullptr);
        delete m_testCaptureTimer;
        m_testCaptureTimer = nullptr;
    }
}

void HikCameraCController::start(const scan_tracking::common::VisionConfig& config)
{
    if (m_started) {
        qWarning(hikCControllerLog) << QStringLiteral("HikCameraCController 已启动，无需重复启动。");
        return;
    }

    m_config = config;
    m_smartCameraIp = config.hikCameraC.ipAddress;
    m_tcpListenIp = config.hikCameraCTcpListenIp.isEmpty()
                        ? QStringLiteral("192.168.8.13")
                        : config.hikCameraCTcpListenIp;
    m_tcpListenPort = config.hikCameraCTcpListenPort > 0
                          ? config.hikCameraCTcpListenPort
                          : 8999;
    m_ftpDirectory = config.hikCameraCFtpDirectory.isEmpty()
                         ? QStringLiteral("D:/HikCameraFTP")
                         : config.hikCameraCFtpDirectory;

    if (m_hikCameraCService == nullptr) {
        setState(HikCameraCState::Error, QStringLiteral("海康相机 C 服务未初始化"));
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("海康相机 C 服务未初始化。"));
        return;
    }

    // 连接相机状态变化信号
    connect(
        m_hikCameraCService,
        &HikCameraService::stateChanged,
        this,
        &HikCameraCController::onCameraCStateChanged,
        Qt::QueuedConnection);

    // 连接错误信号
    connect(
        m_hikCameraCService,
        &HikCameraService::fatalError,
        this,
        &HikCameraCController::onCameraError,
        Qt::QueuedConnection);

    m_started = true;
    setState(HikCameraCState::Initializing, QStringLiteral("海康相机 C 控制器正在初始化"));

    qInfo(hikCControllerLog) << QStringLiteral("HikCameraCController 已启动，相机：")
                             << m_config.hikCameraC.logicalName
                             << QStringLiteral(" IP:") << m_config.hikCameraC.ipAddress
                             << QStringLiteral(" Key:") << m_config.hikCameraC.cameraKey;

    // 初始化 TCP 服务器
    initializeTcpServer();

    // 初始化 FTP 监控器
    initializeFtpMonitor();

    // 检查相机是否已连接
    if (m_hikCameraCService->isConnected()) {
        qInfo(hikCControllerLog) << QStringLiteral("相机 C SDK 连接已建立");
    }
}

void HikCameraCController::stop()
{
    if (!m_started) {
        return;
    }

    // 停止测试定时器
    if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
        m_testCaptureTimer->stop();
        qInfo(hikCControllerLog) << "测试采集定时器已停止";
    }

    m_started = false;
    cleanupFtpMonitor();
    cleanupTcpServer();
    setState(HikCameraCState::Stopped, QStringLiteral("海康相机 C 控制器已停止"));
    qInfo(hikCControllerLog) << QStringLiteral("HikCameraCController 已停止。");
}

void HikCameraCController::initializeTcpServer()
{
    if (m_tcpServer != nullptr) {
        qWarning(hikCControllerLog) << QStringLiteral("TCP 服务器已初始化");
        return;
    }

    m_tcpServer = new HikSmartCameraTcpServer(this);

    // 连接 TCP 服务器信号
    connect(m_tcpServer, &HikSmartCameraTcpServer::serverStarted,
            this, &HikCameraCController::onTcpServerStarted);
    connect(m_tcpServer, &HikSmartCameraTcpServer::serverStopped,
            this, &HikCameraCController::onTcpServerStopped);
    connect(m_tcpServer, &HikSmartCameraTcpServer::cameraConnected,
            this, &HikCameraCController::onTcpCameraConnected);
    connect(m_tcpServer, &HikSmartCameraTcpServer::cameraDisconnected,
            this, &HikCameraCController::onTcpCameraDisconnected);
    connect(m_tcpServer, &HikSmartCameraTcpServer::heartbeatReceived,
            this, &HikCameraCController::onTcpHeartbeatReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::commandReceived,
            this, &HikCameraCController::onTcpCommandReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::imageDataReceived,
            this, &HikCameraCController::onTcpImageDataReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::error,
            this, &HikCameraCController::onTcpError);

    // 启动 TCP 服务器，监听地址和端口来自配置
    const QString listenIp = m_tcpListenIp;
    const quint16 listenPort = m_tcpListenPort;

    // 尝试启动服务器，如果失败则等待一段时间后重试
    if (!m_tcpServer->start(listenIp, listenPort)) {
        qWarning(hikCControllerLog) << QStringLiteral("首次启动 TCP 服务器失败，等待 2 秒后重试...");
        
        // 等待2秒让系统释放端口
        QThread::msleep(2000);
        
        // 重试一次
        if (!m_tcpServer->start(listenIp, listenPort)) {
            qCritical(hikCControllerLog) << QStringLiteral("重试后仍无法启动 TCP 服务器") << listenIp << QStringLiteral(":") << listenPort;
            setState(HikCameraCState::Error, QStringLiteral("TCP 服务器启动失败（端口可能被占用）"));
            emit fatalError(VisionErrorCode::DeviceOpenFailed,
                          QStringLiteral("TCP 服务器启动失败，端口 %1 可能被其他进程占用。").arg(listenPort));
        } else {
        qInfo(hikCControllerLog) << "重试后 TCP 服务器启动成功";
        }
    } else {
        qInfo(hikCControllerLog) << "TCP 服务器启动成功，地址" << listenIp << ":" << listenPort;
    }
}
// 清理 TCP 服务器资源
void HikCameraCController::cleanupTcpServer()
{
    if (m_tcpServer != nullptr) {
        m_tcpServer->stop();
        // 切断父子关系后同步删除，避免 QObject 树析构时二次释放
        m_tcpServer->setParent(nullptr);
        delete m_tcpServer;
        m_tcpServer = nullptr;
        qInfo(hikCControllerLog) << "TCP 服务器已清理";
    }
}

void HikCameraCController::initializeFtpMonitor()
{
    if (m_ftpMonitor != nullptr) {
        qWarning(hikCControllerLog) << QStringLiteral("FTP 监控器已初始化");
        return;
    }

    m_ftpMonitor = new HikSmartCameraFtpMonitor(this);

    // 连接 FTP 监控器信号
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::monitorStarted,
            this, &HikCameraCController::onFtpMonitorStarted);
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::monitorStopped,
            this, &HikCameraCController::onFtpMonitorStopped);
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::newImageDetected,
            this, &HikCameraCController::onFtpNewImageDetected);
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::imageReady,
            this, &HikCameraCController::onFtpImageReady);
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::error,
            this, &HikCameraCController::onFtpError);

    // 启动 FTP 监控器（失败不阻断 TCP 模式，仅告警）
    if (!m_ftpMonitor->start(m_ftpDirectory)) {
        qWarning(hikCControllerLog).noquote()
            << QStringLiteral("FTP 监控器启动失败，目录：") << m_ftpDirectory
            << QStringLiteral("（海康 C 仍可通过 TCP 通信）");
    } else {
        qInfo(hikCControllerLog) << "FTP 监控器启动成功，监控目录：" << m_ftpDirectory;
    }
}

void HikCameraCController::cleanupFtpMonitor()
{
    if (m_ftpMonitor != nullptr) {
        m_ftpMonitor->stop();
        m_ftpMonitor->setParent(nullptr);
        delete m_ftpMonitor;
        m_ftpMonitor = nullptr;
        qInfo(hikCControllerLog) << "FTP 监控器已清理";
    }
}

// 检查 TCP 服务器是否正在运行
bool HikCameraCController::isTcpServerRunning() const
{
    return m_tcpServer != nullptr && m_tcpServer->isListening();
}

bool HikCameraCController::isCameraConnectedToTcp() const
{
    if (m_tcpServer == nullptr) {
        return false;
    }
    return m_tcpServer->connectedCameras().contains(m_smartCameraIp);
}

bool HikCameraCController::isFtpMonitorRunning() const
{
    return m_ftpMonitor != nullptr && m_ftpMonitor->isMonitoring();
}

QString HikCameraCController::ftpDirectory() const
{
    return m_ftpDirectory;
}

bool HikCameraCController::requestCapture(CaptureType type)
{
    if (!isTcpServerRunning()) {
        qWarning(hikCControllerLog) << QStringLiteral("无法请求采集：TCP 服务器未运行");
        return false;
    }

    if (!isCameraConnectedToTcp()) {
        qWarning(hikCControllerLog) << QStringLiteral("无法请求采集：相机未通过 TCP 连接");
        return false;
    }

    m_currentCaptureType = type;
    m_captureCounter++;
    
    qInfo(hikCControllerLog) << "请求采集 #" << m_captureCounter 
                             << "从相机" << m_smartCameraIp
                             << "类型：" << getCaptureTypeString(type);
    
    return m_tcpServer->sendStartCaptureToCamera(m_smartCameraIp);
}

void HikCameraCController::enableTestMode(bool enable, int intervalMs)
{
    if (enable) {
        if (m_testCaptureTimer == nullptr) {
            m_testCaptureTimer = new QTimer(this);
            connect(m_testCaptureTimer, &QTimer::timeout, 
                    this, &HikCameraCController::onTestCaptureTimer);
        }
        
        m_testCaptureTimer->setInterval(intervalMs);
        m_testCaptureTimer->start();
        qInfo(hikCControllerLog) << "测试模式已启用，采集间隔：" << intervalMs << "ms";
    } else {
        if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
            m_testCaptureTimer->stop();
            qInfo(hikCControllerLog) << QStringLiteral("测试模式已禁用");
        }
    }
}

QString HikCameraCController::getCaptureTypeString(CaptureType type) const
{
    switch (type) {
        case CaptureType::SurfaceDefect:
            return QStringLiteral("SurfaceDefect");
        case CaptureType::WeldDefect:
            return QStringLiteral("WeldDefect");
        case CaptureType::NumberRecognition:
            return QStringLiteral("NumberRecognition");
        default:
            return QStringLiteral("Unknown");
    }
}

bool HikCameraCController::saveImageToFile(const QByteArray& imageData, CaptureType type)
{
    // 创建保存目录
    QString saveDir = QStringLiteral("./smart_camera_images");
    QDir dir;
    if (!dir.exists(saveDir)) {
        if (!dir.mkpath(saveDir)) {
            qWarning(hikCControllerLog) << QStringLiteral("创建目录失败：") << saveDir;
            return false;
        }
    }

    // 生成文件名：类型_时间戳_计数器.jpg
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString typeStr = getCaptureTypeString(type);
    QString filename = QString("%1/%2_%3_%4.jpg")
                           .arg(saveDir)
                           .arg(typeStr)
                           .arg(timestamp)
                           .arg(m_captureCounter, 4, 10, QChar('0'));

    // 保存文件
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning(hikCControllerLog) << QStringLiteral("打开文件写入失败：") << filename;
        return false;
    }

    qint64 written = file.write(imageData);
    file.close();

    if (written != imageData.size()) {
        qWarning(hikCControllerLog) << QStringLiteral("图像数据写入不完整：") << filename;
        return false;
    }

    qInfo(hikCControllerLog) << QStringLiteral("图像保存成功：") << filename
                             << QStringLiteral(" size=") << imageData.size() << QStringLiteral(" bytes");
    return true;
}

void HikCameraCController::setState(HikCameraCState state, const QString& description)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(state, description);
        qInfo(hikCControllerLog) << QStringLiteral("状态切换为") << static_cast<int>(state) << QStringLiteral("：") << description;
    }
}

// ============================================================================
// 相机服务信号槽
// ============================================================================

void HikCameraCController::onCameraCStateChanged(
    QString roleName,
    QString stateText,
    QString description)
{
    qInfo(hikCControllerLog) << QStringLiteral("相机 SDK 状态：") << roleName << stateText << description;

    // 当相机 SDK 连接成功后，记录状态（但主要依赖 TCP 连接）
    if (stateText == QStringLiteral("ready") && description.contains(QStringLiteral("已连接"))) {
        qInfo(hikCControllerLog) << "相机 SDK 连接就绪（将使用 TCP 通信）";
    }
}

void HikCameraCController::onCameraError(
    scan_tracking::vision::VisionErrorCode code,
    QString message)
{
    qWarning(hikCControllerLog) << QStringLiteral("相机 SDK 错误：") << static_cast<int>(code) << message;
    // 智能相机主要使用 TCP 通信，SDK 错误不一定影响功能
}

// ============================================================================
// TCP 服务器信号槽
// ============================================================================

void HikCameraCController::onTcpServerStarted(QString listenIp, quint16 port)
{
    qInfo(hikCControllerLog) << "TCP 服务器已启动，地址" << listenIp << ":" << port;
    qInfo(hikCControllerLog) << "等待智能相机" << m_smartCameraIp << "连接...";
}

void HikCameraCController::onTcpServerStopped()
{
    qInfo(hikCControllerLog) << "TCP 服务器已停止";
}

void HikCameraCController::onTcpCameraConnected(QString cameraIp, quint16 cameraPort)
{
    qInfo(hikCControllerLog) << QStringLiteral("智能相机已通过 TCP 连接：") << cameraIp << QStringLiteral(":") << cameraPort;
    
    if (cameraIp == m_smartCameraIp) {
        setState(HikCameraCState::Ready, QStringLiteral("智能相机已通过 TCP 连接并就绪"));

        // 连接成功后，启动自动拍照测试（每10秒一次）
        enableTestMode(true, 10000);
        qInfo(hikCControllerLog) << "自动采集已启用：每 10 秒一次";
    } else {
        qWarning(hikCControllerLog) << QStringLiteral("意外相机 IP 已连接：") << cameraIp
                                    << QStringLiteral("（期望：") << m_smartCameraIp << QStringLiteral("）");
    }
}

void HikCameraCController::onTcpCameraDisconnected(QString cameraIp)
{
    qWarning(hikCControllerLog) << QStringLiteral("智能相机 TCP 连接断开：") << cameraIp;
    
    if (cameraIp == m_smartCameraIp) {
        setState(HikCameraCState::Error, QStringLiteral("智能相机 TCP 连接断开"));
        
        // 停止测试定时器
        if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
            m_testCaptureTimer->stop();
            qInfo(hikCControllerLog) << "因断开连接，测试采集定时器已停止";
        }
    }
}

void HikCameraCController::onTcpHeartbeatReceived(QString cameraIp)
{
    // 心跳日志可以设置为 Debug 级别，避免刷屏
    qDebug(hikCControllerLog) << QStringLiteral("收到心跳：") << cameraIp;
}

void HikCameraCController::onTcpCommandReceived(QString cameraIp, QString command)
{
    qInfo(hikCControllerLog) << QStringLiteral("收到命令：") << cameraIp << QStringLiteral("：") << command;
    // TODO: 处理相机发送的其他指令
}

void HikCameraCController::onTcpImageDataReceived(QString cameraIp, QByteArray imageData)
{
    qInfo(hikCControllerLog) << QStringLiteral("收到图像数据：") << cameraIp
                             << QStringLiteral(" size=") << imageData.size() << QStringLiteral(" bytes")
                             << QStringLiteral(" type=") << getCaptureTypeString(m_currentCaptureType);
    
    // 保存图像到文件
    if (saveImageToFile(imageData, m_currentCaptureType)) {
        qInfo(hikCControllerLog) << QStringLiteral("图像保存成功");
    } else {
        qWarning(hikCControllerLog) << QStringLiteral("图像保存失败");
    }
    
    // 发射图像数据信号
    emit captureCompleted(m_currentCaptureType, imageData);
}

void HikCameraCController::onTcpError(QString errorMessage)
{
    qWarning(hikCControllerLog) << QStringLiteral("TCP 错误：") << errorMessage;
    emit fatalError(VisionErrorCode::DeviceOpenFailed, errorMessage);
}

void HikCameraCController::onTestCaptureTimer()
{
    if (!m_started || !isCameraConnectedToTcp()) {
        qWarning(hikCControllerLog) << QStringLiteral("测试采集跳过：未就绪");
        return;
    }
    
    // 循环测试三种拍照类型
    CaptureType types[] = {
        CaptureType::SurfaceDefect,
        CaptureType::WeldDefect,
        CaptureType::NumberRecognition
    };
    
    static int typeIndex = 0;
    CaptureType currentType = types[typeIndex];
    typeIndex = (typeIndex + 1) % 3;
    
    qInfo(hikCControllerLog) << "测试采集已触发，类型：" << getCaptureTypeString(currentType);
    requestCapture(currentType);
}

// ============================================================================
// FTP 监控器信号槽
// ============================================================================

void HikCameraCController::onFtpMonitorStarted(QString directory)
{
    qInfo(hikCControllerLog) << "FTP 监控器已启动，监控目录：" << directory;
}

void HikCameraCController::onFtpMonitorStopped()
{
    qInfo(hikCControllerLog) << "FTP 监控器已停止";
}

void HikCameraCController::onFtpNewImageDetected(ImageFileInfo imageInfo)
{
    // 静默处理，不打印日志
}

void HikCameraCController::onFtpImageReady(ImageFileInfo imageInfo)
{
    // 静默处理，只发射信号
    emit imageReceived(imageInfo.captureType, imageInfo.filePath, imageInfo.fileSize);
    
    // TODO: 这里可以触发后续的图像处理流程
    // 例如：调用缺陷识别算法、OCR识别等
}

void HikCameraCController::onFtpError(QString errorMessage)
{
    qWarning(hikCControllerLog) << QStringLiteral("FTP 监控器错误：") << errorMessage;
}

}  // namespace vision
}  // namespace scan_tracking
