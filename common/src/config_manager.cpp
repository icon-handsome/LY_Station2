#include "scan_tracking/common/config_manager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLoggingCategory>

#include "scan_tracking/common/logger.h"

namespace scan_tracking {
namespace common {

Q_LOGGING_CATEGORY(LOG_CONFIG, "config")

ConfigManager* ConfigManager::s_instance = nullptr;

namespace {

QString projectRootConfigPath()
{
    QDir rootDir(QCoreApplication::applicationDirPath());
    if (rootDir.cdUp() && rootDir.cdUp() && rootDir.cdUp()) {
        return rootDir.filePath(QStringLiteral("config.ini"));
    }
    return QCoreApplication::applicationDirPath() + QStringLiteral("/config.ini");
}

/**
 * @brief 获取扫描路径配置文件的路径
 * 
 * 优先查找项目根目录，如果找不到则使用可执行文件目录。
 * 
 * @return 扫描路径配置文件的完整路径
 */
QString scanPathsConfigPath()
{
    // 优先使用项目根目录
    QDir rootDir(QCoreApplication::applicationDirPath());
    if (rootDir.cdUp() && rootDir.cdUp() && rootDir.cdUp()) {
        QString rootPath = rootDir.filePath(QStringLiteral("scan_paths_config.json"));
        if (QFileInfo::exists(rootPath)) {
            return rootPath;
        }
    }
    
    // 回退到可执行文件目录
    return QCoreApplication::applicationDirPath() + QStringLiteral("/scan_paths_config.json");
}

}  // namespace

void ConfigManager::initialize()
{
    if (!s_instance) {
        s_instance = new ConfigManager();
        qInfo(LOG_CONFIG) << "ConfigManager 已初始化。";
    }
}

void ConfigManager::cleanup()
{
    if (s_instance) {
        delete s_instance;
        s_instance = nullptr;
        qInfo(LOG_CONFIG) << "ConfigManager 已清理。";
    }
}

ConfigManager* ConfigManager::instance()
{
    if (!s_instance) {
        qWarning(LOG_CONFIG) << "ConfigManager::instance() 在 initialize() 之前被调用！";
    }
    return s_instance;
}

ConfigManager::ConfigManager()
{
    const QString configPath = projectRootConfigPath();
    load(configPath);
    
    // 加载扫描路径配置
    const QString scanPathsPath = scanPathsConfigPath();
    loadScanPathsConfig(scanPathsPath);
}

ConfigManager::~ConfigManager() = default;

const AppConfig& ConfigManager::appConfig() const { return m_appConfig; }
const LoggerConfig& ConfigManager::loggerConfig() const { return m_loggerConfig; }
const ModbusConfig& ConfigManager::modbusConfig() const { return m_modbusConfig; }
const CameraConfig& ConfigManager::cameraConfig() const { return m_cameraConfig; }
const VisionConfig& ConfigManager::visionConfig() const { return m_visionConfig; }
const FlowControlConfig& ConfigManager::flowControlConfig() const { return m_flowControlConfig; }
const TrackingConfig& ConfigManager::trackingConfig() const { return m_trackingConfig; }
const LbPoseConfig& ConfigManager::lbPoseConfig() const { return m_lbPoseConfig; }
const LbnPoseConfig& ConfigManager::lbnPoseConfig() const { return m_lbnPoseConfig; }
const ScanPathsConfig& ConfigManager::scanPathsConfig() const { return m_scanPathsConfig; }

void ConfigManager::writeDefaults(QSettings& settings)
{
    settings.beginGroup("App");
    settings.setValue("version", "0.1.0");
    settings.setValue("environment", "production");
    settings.endGroup();

    settings.beginGroup("Logger");
    settings.setValue("level", 0);
    settings.setValue("rotateDays", 7);
    settings.endGroup();

    settings.beginGroup("Modbus");
    settings.setValue("host", "127.0.0.1");
    settings.setValue("port", 502);
    settings.setValue("unitId", 3);
    settings.setValue("timeoutMs", 1000);
    settings.setValue("reconnectIntervalMs", 2000);
    settings.endGroup();

    settings.beginGroup("Camera");
    settings.setValue("defaultCamera", "Mech-Eye Nano");
    settings.setValue("scanTimeoutMs", 5000);
    settings.endGroup();

    settings.beginGroup("Vision");
    settings.setValue("mechEyeCameraKey", "Mech-Eye Nano");
    settings.setValue("mechCaptureTimeoutMs", 5000);
    settings.setValue("hikConnectTimeoutMs", 3000);
    settings.setValue("hikCaptureTimeoutMs", 1000);
    settings.setValue("hikSdkRoot", "D:/work/scan-tracking/third_party/hik_mvs");
    settings.setValue("hikCameraAName", "hik_camera_a");
    settings.setValue("hikCameraAKey", "192.168.10.12");
    settings.setValue("hikCameraAIp", "192.168.10.12");
    settings.setValue("hikCameraASerial", "");
    settings.setValue("hikCameraBName", "hik_camera_b");
    settings.setValue("hikCameraBKey", "192.168.10.13");
    settings.setValue("hikCameraBIp", "192.168.10.13");
    settings.setValue("hikCameraBSerial", "");
    settings.setValue("hikCameraCName", "hik_camera_c");
    settings.setValue("hikCameraCKey", "192.168.8.100");
    settings.setValue("hikCameraCIp", "192.168.8.100");
    settings.setValue("hikCameraCSerial", "");
    settings.setValue("hikCameraCAccessMode", "monitor");
    settings.setValue("hikCameraCTcpListenIp", "192.168.8.13");
    settings.setValue("hikCameraCTcpListenPort", 8999);
    settings.setValue("hikCameraCFtpDirectory", "D:/HikCameraFTP");
    settings.endGroup();

    settings.beginGroup("LbPose");
    settings.setValue("dataRoot", "D:/work/scan-tracking/third_party/lb_pose_detection/data");
    settings.setValue("leftPattern", "");
    settings.setValue("rightPattern", "");
    settings.setValue("templateFile", "");
    settings.setValue("minDistance", 30.0);
    settings.setValue("maxDistance", 650.0);
    settings.setValue("cosTolerance", 0.015);
    settings.setValue("minPercent", 0.5);
    settings.endGroup();

    settings.beginGroup("LbnPose");
    settings.setValue("enabled", true);
    settings.setValue("dataRoot", "D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data");
    settings.setValue("templateFile", "");
    settings.setValue("minDistance", 30.0);
    settings.setValue("maxDistance", 650.0);
    settings.setValue("cosTolerance", 0.015);
    settings.setValue("minPercent", 0.5);
    settings.setValue("cloudSearchRadiusPx", 20);
    settings.endGroup();

    settings.beginGroup("FlowControl");
    settings.setValue("pollIntervalMs", 100);
    settings.setValue("heartbeatIntervalMs", 1000);
    settings.setValue("simulatedProcessingMs", 300);
    settings.endGroup();

    settings.beginGroup("Tracking");
    settings.setValue("firstStationOuterSegmentIndex", 1);
    settings.setValue("firstStationInnerSegmentIndex", 2);
    settings.setValue("firstStationHoleSegmentIndex", 3);
    settings.endGroup();

    settings.sync();
    qInfo(LOG_CONFIG) << "已在" << settings.fileName() << "生成默认 config.ini";
}

void ConfigManager::load(const QString& filePath)
{
    const QFileInfo fileInfo(filePath);
    const bool fileExists = fileInfo.exists() && fileInfo.size() > 0;

    QSettings settings(filePath, QSettings::IniFormat);
    if (!fileExists) {
        qWarning(LOG_CONFIG) << "config.ini 未找到或为空。正在生成默认配置...";
        writeDefaults(settings);
    }

    settings.beginGroup("App");
    m_appConfig.version = settings.value("version", "0.1.0").toString();
    m_appConfig.environment = settings.value("environment", "production").toString();
    settings.endGroup();

    settings.beginGroup("Logger");
    m_loggerConfig.level = settings.value("level", 0).toInt();
    m_loggerConfig.rotateDays = settings.value("rotateDays", 7).toInt();
    settings.endGroup();

    settings.beginGroup("Modbus");
    m_modbusConfig.host = settings.value("host", "127.0.0.1").toString();
    m_modbusConfig.port = settings.value("port", 502).toInt();
    m_modbusConfig.unitId = settings.value("unitId", 1).toInt();
    m_modbusConfig.timeoutMs = settings.value("timeoutMs", 1000).toInt();
    m_modbusConfig.reconnectIntervalMs = settings.value("reconnectIntervalMs", 2000).toInt();
    settings.endGroup();

    settings.beginGroup("Camera");
    m_cameraConfig.defaultCamera = settings.value("defaultCamera", "Mech-Eye Nano").toString();
    m_cameraConfig.scanTimeoutMs = settings.value("scanTimeoutMs", 5000).toInt();
    settings.endGroup();

    settings.beginGroup("Vision");
    m_visionConfig.mechEyeCameraKey = settings.value("mechEyeCameraKey", m_cameraConfig.defaultCamera).toString();
    m_visionConfig.mechCaptureTimeoutMs = settings.value("mechCaptureTimeoutMs", m_cameraConfig.scanTimeoutMs).toInt();
    m_visionConfig.mechDepthRangeMin = settings.value("mechDepthRangeMin", 100).toInt();
    m_visionConfig.mechDepthRangeMax = settings.value("mechDepthRangeMax", 2000).toInt();
    m_visionConfig.hikConnectTimeoutMs = settings.value("hikConnectTimeoutMs", 3000).toInt();
    m_visionConfig.hikCaptureTimeoutMs = settings.value("hikCaptureTimeoutMs", 1000).toInt();
    m_visionConfig.hikSdkRoot = settings.value("hikSdkRoot", "D:/work/scan-tracking/third_party/hik_mvs").toString();
    m_visionConfig.hikCameraA.logicalName = settings.value("hikCameraAName", "hik_camera_a").toString();
    m_visionConfig.hikCameraA.cameraKey = settings.value("hikCameraAKey", "192.168.10.12").toString();
    m_visionConfig.hikCameraA.ipAddress = settings.value("hikCameraAIp", "192.168.10.12").toString();
    m_visionConfig.hikCameraA.serialNumber = settings.value("hikCameraASerial", "").toString();
    m_visionConfig.hikCameraB.logicalName = settings.value("hikCameraBName", "hik_camera_b").toString();
    m_visionConfig.hikCameraB.cameraKey = settings.value("hikCameraBKey", "192.168.10.13").toString();
    m_visionConfig.hikCameraB.ipAddress = settings.value("hikCameraBIp", "192.168.10.13").toString();
    m_visionConfig.hikCameraB.serialNumber = settings.value("hikCameraBSerial", "").toString();
    m_visionConfig.hikCameraC.logicalName = settings.value("hikCameraCName", "hik_camera_c").toString();
    m_visionConfig.hikCameraC.cameraKey = settings.value("hikCameraCKey", "192.168.8.100").toString();
    m_visionConfig.hikCameraC.ipAddress = settings.value("hikCameraCIp", "192.168.8.100").toString();
    m_visionConfig.hikCameraC.serialNumber = settings.value("hikCameraCSerial", "").toString();
    m_visionConfig.hikCameraC.accessMode = settings.value("hikCameraCAccessMode", "monitor").toString();
    m_visionConfig.hikCameraCTcpListenIp = settings.value("hikCameraCTcpListenIp", "192.168.8.13").toString();
    m_visionConfig.hikCameraCTcpListenPort = static_cast<quint16>(settings.value("hikCameraCTcpListenPort", 8999).toUInt());
    m_visionConfig.hikCameraCFtpDirectory = settings.value("hikCameraCFtpDirectory", "D:/HikCameraFTP").toString();
    settings.endGroup();

    settings.beginGroup("LbPose");
    m_lbPoseConfig.dataRoot = settings.value("dataRoot", "D:/work/scan-tracking/third_party/lb_pose_detection/data").toString();
    m_lbPoseConfig.leftPattern = settings.value("leftPattern", "").toString();
    m_lbPoseConfig.rightPattern = settings.value("rightPattern", "").toString();
    m_lbPoseConfig.templateFile = settings.value("templateFile", "").toString();
    m_lbPoseConfig.minDistance = settings.value("minDistance", 30.0).toFloat();
    m_lbPoseConfig.maxDistance = settings.value("maxDistance", 650.0).toFloat();
    m_lbPoseConfig.cosTolerance = settings.value("cosTolerance", 0.015).toFloat();
    m_lbPoseConfig.minPercent = settings.value("minPercent", 0.5).toFloat();
    settings.endGroup();

    settings.beginGroup("LbnPose");
    m_lbnPoseConfig.enabled = settings.value("enabled", true).toBool();
    m_lbnPoseConfig.dataRoot = settings.value(
        "dataRoot",
        QStringLiteral("D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data"))
        .toString();
    m_lbnPoseConfig.templateFile = settings.value("templateFile", "").toString();
    m_lbnPoseConfig.minDistance = settings.value("minDistance", 30.0).toFloat();
    m_lbnPoseConfig.maxDistance = settings.value("maxDistance", 650.0).toFloat();
    m_lbnPoseConfig.cosTolerance = settings.value("cosTolerance", 0.015).toFloat();
    m_lbnPoseConfig.minPercent = settings.value("minPercent", 0.5).toFloat();
    m_lbnPoseConfig.cloudSearchRadiusPx = settings.value("cloudSearchRadiusPx", 20).toInt();
    settings.endGroup();

    settings.beginGroup("FlowControl");
    m_flowControlConfig.pollIntervalMs = settings.value("pollIntervalMs", 100).toInt();
    m_flowControlConfig.heartbeatIntervalMs = settings.value("heartbeatIntervalMs", 1000).toInt();
    m_flowControlConfig.simulatedProcessingMs = settings.value("simulatedProcessingMs", 300).toInt();
    settings.endGroup();

    settings.beginGroup("Tracking");
    m_trackingConfig.firstStationOuterSegmentIndex = settings.value("firstStationOuterSegmentIndex", 1).toInt();
    m_trackingConfig.firstStationInnerSegmentIndex = settings.value("firstStationInnerSegmentIndex", 2).toInt();
    m_trackingConfig.firstStationHoleSegmentIndex = settings.value("firstStationHoleSegmentIndex", 3).toInt();
    m_trackingConfig.scanSegmentTotal = settings.value("scanSegmentTotal", 3).toInt();
    settings.endGroup();

    QtMsgType minType = QtDebugMsg;
    switch (m_loggerConfig.level) {
        case 0: minType = QtDebugMsg; break;
        case 1: minType = QtInfoMsg; break;
        case 2: minType = QtWarningMsg; break;
        case 3: minType = QtCriticalMsg; break;
        default: minType = QtDebugMsg; break;
    }

    if (Logger* logger = Logger::instance()) {
        logger->setMinLevel(minType);
    }

    qInfo(LOG_CONFIG) << "已从以下位置加载 config.ini：" << filePath;
    qInfo(LOG_CONFIG).noquote()
        << "Modbus 配置："
        << "host=" << m_modbusConfig.host
        << "port=" << m_modbusConfig.port
        << "unitId=" << m_modbusConfig.unitId
        << "timeoutMs=" << m_modbusConfig.timeoutMs
        << "reconnectIntervalMs=" << m_modbusConfig.reconnectIntervalMs;
}

/**
 * @brief 加载扫描路径配置（JSON 格式）
 * 
 * 从 scan_paths_config.json 文件加载多路径扫描配置，包括：
 * - 标定矩阵 T0
 * - 所有扫描路径定义
 * - 执行策略
 * - 转盘配置
 * 
 * @param jsonFilePath JSON 配置文件路径
 */
void ConfigManager::loadScanPathsConfig(const QString& jsonFilePath)
{
    // 检查文件是否存在
    if (!QFileInfo::exists(jsonFilePath)) {
        qWarning(LOG_CONFIG) << "扫描路径配置文件不存在：" << jsonFilePath;
        qWarning(LOG_CONFIG) << "将使用空配置，多路径扫描功能不可用。";
        return;
    }
    
    // 读取 JSON 文件
    QFile file(jsonFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qCritical(LOG_CONFIG) << "无法打开扫描路径配置文件：" << jsonFilePath;
        return;
    }
    
    const QByteArray jsonData = file.readAll();
    file.close();
    
    // 解析 JSON
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(jsonData, &parseError);
    
    if (parseError.error != QJsonParseError::NoError) {
        qCritical(LOG_CONFIG) << "扫描路径配置文件 JSON 解析失败："
                              << parseError.errorString()
                              << "位置：" << parseError.offset;
        return;
    }
    
    if (!doc.isObject()) {
        qCritical(LOG_CONFIG) << "扫描路径配置文件格式错误：根节点不是对象";
        return;
    }
    
    const QJsonObject root = doc.object();
    
    // 1. 读取配置文件元数据
    m_scanPathsConfig.version = root.value("version").toString("1.0");
    m_scanPathsConfig.lastModified = root.value("lastModified").toString();
    
    // 2. 读取标定矩阵 T0
    const QJsonObject calibMatrixObj = root.value("calibrationMatrix").toObject();
    const QJsonArray t0Array = calibMatrixObj.value("T0").toArray();
    
    if (t0Array.size() == 4) {
        // 4x4 矩阵，行优先存储
        int index = 0;
        for (int row = 0; row < 4; ++row) {
            const QJsonArray rowArray = t0Array.at(row).toArray();
            for (int col = 0; col < 4; ++col) {
                m_scanPathsConfig.calibrationMatrixT0[index++] = 
                    static_cast<float>(rowArray.at(col).toDouble(row == col ? 1.0 : 0.0));
            }
        }
    } else {
        qWarning(LOG_CONFIG) << "标定矩阵 T0 格式错误，使用单位矩阵";
        // 初始化为单位矩阵
        m_scanPathsConfig.calibrationMatrixT0.fill(0.0f);
        m_scanPathsConfig.calibrationMatrixT0[0] = 1.0f;
        m_scanPathsConfig.calibrationMatrixT0[5] = 1.0f;
        m_scanPathsConfig.calibrationMatrixT0[10] = 1.0f;
        m_scanPathsConfig.calibrationMatrixT0[15] = 1.0f;
    }
    
    // 3. 读取扫描路径列表
    const QJsonArray pathsArray = root.value("scanPaths").toArray();
    m_scanPathsConfig.scanPaths.clear();
    m_scanPathsConfig.scanPaths.reserve(pathsArray.size());
    
    for (const QJsonValue& pathValue : pathsArray) {
        const QJsonObject pathObj = pathValue.toObject();
        
        ScanPathConfig pathConfig;
        pathConfig.pathId = pathObj.value("pathId").toInt();
        pathConfig.pathName = pathObj.value("pathName").toString();
        pathConfig.description = pathObj.value("description").toString();
        pathConfig.enabled = pathObj.value("enabled").toBool(true);
        pathConfig.totalPoints = pathObj.value("totalPoints").toInt();
        
        // 读取点位列表
        const QJsonArray pointsArray = pathObj.value("points").toArray();
        pathConfig.points.clear();
        pathConfig.points.reserve(pointsArray.size());
        
        for (const QJsonValue& pointValue : pointsArray) {
            const QJsonObject pointObj = pointValue.toObject();
            
            ScanPointConfig pointConfig;
            pointConfig.pointIndex = pointObj.value("pointIndex").toInt();
            pointConfig.pointName = pointObj.value("pointName").toString();
            pointConfig.needRotation = pointObj.value("needRotation").toBool(false);
            pointConfig.rotationAngle = static_cast<float>(pointObj.value("rotationAngle").toDouble(0.0));
            pointConfig.description = pointObj.value("description").toString();
            
            pathConfig.points.push_back(pointConfig);
        }
        
        m_scanPathsConfig.scanPaths.push_back(pathConfig);
    }
    
    // 4. 读取执行策略
    const QJsonObject execConfigObj = root.value("executionConfig").toObject();
    m_scanPathsConfig.executeAllPaths = execConfigObj.value("executeAllPaths").toBool(false);
    m_scanPathsConfig.allowPathSkipOnError = execConfigObj.value("allowPathSkipOnError").toBool(false);
    
    const QJsonArray selectedIdsArray = execConfigObj.value("selectedPathIds").toArray();
    m_scanPathsConfig.selectedPathIds.clear();
    m_scanPathsConfig.selectedPathIds.reserve(selectedIdsArray.size());
    for (const QJsonValue& idValue : selectedIdsArray) {
        m_scanPathsConfig.selectedPathIds.push_back(idValue.toInt());
    }
    
    // 5. 读取转盘配置
    const QJsonObject turntableObj = root.value("turntableConfig").toObject();
    m_scanPathsConfig.turntableEnabled = turntableObj.value("enabled").toBool(true);
    
    // 6. 验证配置
    QString validationError;
    if (!validateScanPathsConfig(&validationError)) {
        qWarning(LOG_CONFIG) << "扫描路径配置验证失败：" << validationError;
    }
    
    // 7. 输出加载信息
    qInfo(LOG_CONFIG) << "已从以下位置加载扫描路径配置：" << jsonFilePath;
    qInfo(LOG_CONFIG).noquote()
        << "扫描路径配置："
        << "版本=" << m_scanPathsConfig.version
        << "路径数=" << m_scanPathsConfig.scanPaths.size()
        << "执行所有路径=" << m_scanPathsConfig.executeAllPaths
        << "选中路径数=" << m_scanPathsConfig.selectedPathIds.size()
        << "转盘启用=" << m_scanPathsConfig.turntableEnabled;
    
    // 输出每条路径的详细信息
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        qInfo(LOG_CONFIG).noquote()
            << "  路径" << path.pathId << ":" << path.pathName
            << "启用=" << path.enabled
            << "点位数=" << path.points.size();
    }
}

/**
 * @brief 验证扫描路径配置的合法性
 * 
 * 检查配置是否符合以下规则：
 * - 路径 ID 唯一
 * - 点位索引连续（从 1 开始）
 * - 点位数量与 totalPoints 一致
 * - 选中的路径 ID 存在
 * 
 * @param errorMessage 输出参数，验证失败时包含错误信息
 * @return 验证是否通过
 */
bool ConfigManager::validateScanPathsConfig(QString* errorMessage) const
{
    // 1. 检查路径 ID 唯一性
    std::vector<int> pathIds;
    pathIds.reserve(m_scanPathsConfig.scanPaths.size());
    
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (std::find(pathIds.begin(), pathIds.end(), path.pathId) != pathIds.end()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 ID 重复：%1").arg(path.pathId);
            }
            return false;
        }
        pathIds.push_back(path.pathId);
        
        // 2. 检查点位数量
        if (static_cast<int>(path.points.size()) != path.totalPoints) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 %1 的点位数量不匹配：配置 %2，实际 %3")
                    .arg(path.pathId)
                    .arg(path.totalPoints)
                    .arg(path.points.size());
            }
            return false;
        }
        
        // 3. 检查点位索引连续性（从 1 开始）
        for (size_t i = 0; i < path.points.size(); ++i) {
            const int expectedIndex = static_cast<int>(i) + 1;
            if (path.points[i].pointIndex != expectedIndex) {
                if (errorMessage) {
                    *errorMessage = QStringLiteral("路径 %1 的点位索引不连续：期望 %2，实际 %3")
                        .arg(path.pathId)
                        .arg(expectedIndex)
                        .arg(path.points[i].pointIndex);
                }
                return false;
            }
        }
    }
    
    // 4. 检查选中的路径 ID 是否存在
    for (int selectedId : m_scanPathsConfig.selectedPathIds) {
        if (std::find(pathIds.begin(), pathIds.end(), selectedId) == pathIds.end()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("选中的路径 ID 不存在：%1").arg(selectedId);
            }
            return false;
        }
    }
    
    return true;
}

}  // namespace common
}  // namespace scan_tracking
