#include "scan_tracking/common/config_manager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLoggingCategory>
#include <QStringList>

#include "scan_tracking/common/logger.h"

namespace scan_tracking {
namespace common {

Q_LOGGING_CATEGORY(LOG_CONFIG, "config")

ConfigManager* ConfigManager::s_instance = nullptr;

namespace {

QString projectRootConfigPath()
{
    const QString exeDirConfig =
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config.ini"));
    if (QFileInfo::exists(exeDirConfig)) {
        return exeDirConfig;
    }

    QDir rootDir(QCoreApplication::applicationDirPath());
    if (rootDir.cdUp() && rootDir.cdUp() && rootDir.cdUp()) {
        return rootDir.filePath(QStringLiteral("config.ini"));
    }
    return exeDirConfig;
}

QString resolveConfigRelativePath(const QString& rawPath, const QString& configFilePath)
{
    const QString trimmed = rawPath.trimmed();
    if (trimmed.isEmpty()) {
        return QString();
    }

    const QFileInfo pathInfo(trimmed);
    if (pathInfo.isAbsolute()) {
        return QDir::cleanPath(trimmed);
    }

    const QString configDirPath = QFileInfo(configFilePath).absoluteDir().filePath(trimmed);
    if (QFileInfo::exists(configDirPath)) {
        return QDir::cleanPath(configDirPath);
    }

    const QString exeDirPath = QDir(QCoreApplication::applicationDirPath()).filePath(trimmed);
    if (QFileInfo::exists(exeDirPath)) {
        return QDir::cleanPath(exeDirPath);
    }

    return QDir::cleanPath(configDirPath);
}

/**
 * @brief 获取扫描路径配置文件的路径
 * 
 * Stage 1: 工位 profile 显式配置优先；未配置时保持旧 scan_paths_config.json fallback。
 * 
 * @return 扫描路径配置文件的完整路径
 */
QString scanPathsConfigPath(const StationProfile& stationProfile, const QString& configFilePath)
{
    if (!stationProfile.scanPathsConfigPath.trimmed().isEmpty()) {
        return resolveConfigRelativePath(stationProfile.scanPathsConfigPath, configFilePath);
    }

    const QString fallbackPath =
        resolveConfigRelativePath(QStringLiteral("config/scan_paths/station2_placeholder.json"), configFilePath);
    if (QFileInfo::exists(fallbackPath)) {
        return fallbackPath;
    }

    return fallbackPath;
}

void applyStationSettings(QSettings& settings, StationProfile& profile, QString* profileIni)
{
    if (settings.contains(QStringLiteral("stationId"))) {
        profile.stationId = stationIdFromInt(settings.value(QStringLiteral("stationId"), 2).toInt());
    }
    if (settings.contains(QStringLiteral("stationName"))) {
        profile.stationName = settings.value(QStringLiteral("stationName"), profile.stationName).toString();
    }
    if (settings.contains(QStringLiteral("scanPathsConfigPath"))) {
        profile.scanPathsConfigPath =
            settings.value(QStringLiteral("scanPathsConfigPath"), profile.scanPathsConfigPath).toString().trimmed();
    }
    if (settings.contains(QStringLiteral("defaultWorkMode"))) {
        bool ok = false;
        const QString raw = settings.value(QStringLiteral("defaultWorkMode")).toString();
        profile.defaultWorkMode = workModeIdFromString(raw, &ok);
        if (!ok) {
            qWarning(LOG_CONFIG).noquote()
                << QStringLiteral("[Station] 未知 defaultWorkMode=") << raw
                << QStringLiteral("，已回退 Unknown");
        }
    }
    if (settings.contains(QStringLiteral("enableLoadGrasp"))) {
        profile.enableLoadGrasp = settings.value(QStringLiteral("enableLoadGrasp"), profile.enableLoadGrasp).toBool();
    }
    if (settings.contains(QStringLiteral("enableUnloadCalc"))) {
        profile.enableUnloadCalc = settings.value(QStringLiteral("enableUnloadCalc"), profile.enableUnloadCalc).toBool();
    }
    if (settings.contains(QStringLiteral("enablePoseCheck"))) {
        profile.enablePoseCheck = settings.value(QStringLiteral("enablePoseCheck"), profile.enablePoseCheck).toBool();
    }
    if (settings.contains(QStringLiteral("enableTelescopicScan"))) {
        profile.enableTelescopicScan =
            settings.value(QStringLiteral("enableTelescopicScan"), profile.enableTelescopicScan).toBool();
    }
    if (settings.contains(QStringLiteral("enableHoistAssist"))) {
        profile.enableHoistAssist = settings.value(QStringLiteral("enableHoistAssist"), profile.enableHoistAssist).toBool();
    }
    if (settings.contains(QStringLiteral("enableCollisionMonitor"))) {
        profile.enableCollisionMonitor =
            settings.value(QStringLiteral("enableCollisionMonitor"), profile.enableCollisionMonitor).toBool();
    }
    if (profileIni != nullptr && settings.contains(QStringLiteral("profileIni"))) {
        *profileIni = settings.value(QStringLiteral("profileIni")).toString().trimmed();
    }
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
    const QString scanPathsPath = scanPathsConfigPath(m_stationProfile, configPath);
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
const OrbbecGeminiConfig& ConfigManager::orbbecGeminiConfig() const { return m_orbbecGeminiConfig; }
const LivoxMid360Config& ConfigManager::livoxMid360Config() const { return m_livoxMid360Config; }
const TfminiPlusConfig& ConfigManager::tfminiPlusConfig() const { return m_tfminiPlusConfig; }
QString ConfigManager::configFilePath() const { return m_configFilePath; }
const HmiConfig& ConfigManager::hmiConfig() const { return m_hmiConfig; }
const ScanPathsConfig& ConfigManager::scanPathsConfig() const { return m_scanPathsConfig; }
const StationProfile& ConfigManager::stationProfile() const { return m_stationProfile; }

void ConfigManager::writeDefaults(QSettings& settings)
{
    settings.beginGroup("App");
    settings.setValue("version", "0.1.0");
    settings.setValue("environment", "production");
    settings.endGroup();

    settings.beginGroup("Logger");
    settings.setValue("level", 0);
    settings.setValue("rotateDays", 0);  // 保留项，不触发日志删除/覆盖
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

    settings.beginGroup("Station");
    settings.setValue("stationId", 2);
    settings.setValue("stationName", QStringLiteral("第二工位-圆筒半成品"));
    settings.setValue("scanPathsConfigPath", QStringLiteral("config/scan_paths/station2_placeholder.json"));
    settings.setValue("defaultWorkMode", QStringLiteral("MODE_CYLINDER_SEMI"));
    settings.setValue("profileIni", QStringLiteral("config/station_profiles/station2_cylinder_semi.ini"));
    settings.endGroup();

    settings.beginGroup("Vision");
    settings.setValue("mechEyeCameraKey", "Mech-Eye Nano");
    settings.setValue("mechCaptureTimeoutMs", 5000);
    settings.setValue("hikConnectTimeoutMs", 3000);
    settings.setValue("hikCaptureTimeoutMs", 1000);
    settings.setValue("hikExposureTimeUs", 50000);
    settings.setValue("hikGain", 0.0);
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
    settings.setValue("hikCxpEnabled", true);
    settings.setValue("hikCxpCaptureTimeoutMs", 5000);
    settings.setValue("hikCxpExposureTimeUs", 50000);
    settings.setValue("hikCxpGain", 0.0);
    settings.setValue("hikCxpSmokeOutputDir", "D:/CxpSmokeTest");
    settings.setValue("hikCxpCameraAName", "ch250_a");
    settings.setValue("hikCxpCameraAKey", "DA9122997");
    settings.setValue("hikCxpCameraASerial", "DA9122997");
    settings.setValue("hikCxpCameraBName", "ch250_b");
    settings.setValue("hikCxpCameraBKey", "DA9122998");
    settings.setValue("hikCxpCameraBSerial", "DA9122998");
    settings.endGroup();

    settings.beginGroup("FlowControl");
    settings.setValue("pollIntervalMs", 100);
    settings.setValue("heartbeatIntervalMs", 1000);
    settings.setValue("simulatedProcessingMs", 300);
    settings.endGroup();

    settings.beginGroup("Tracking");
    settings.setValue("scanSegmentTotal", 3);
    settings.endGroup();

    settings.beginGroup("OrbbecGemini");
    settings.setValue("orbbecGeminiEnabled", false);
    settings.setValue("orbbecGeminiSdkRoot", QStringLiteral("C:/Program Files/OrbbecSDK 2.8.6"));
    settings.setValue("orbbecGeminiSerial", QString());
    settings.setValue("orbbecGeminiDeviceIndex", 0);
    settings.setValue("orbbecGeminiDepthWidth", 640);
    settings.setValue("orbbecGeminiDepthHeight", 480);
    settings.setValue("orbbecGeminiFps", 15);
    settings.setValue("orbbecGeminiCaptureTimeoutMs", 5000);
    settings.setValue("orbbecGeminiWarmupFrameCount", 5);
    settings.setValue("orbbecGeminiSaveCaptureToDisk", true);
    settings.setValue("orbbecGeminiCaptureCacheDir", QString());
    settings.setValue("orbbecGeminiEnableColorStream", false);
    settings.setValue("orbbecGeminiCaptureOnStart", false);
    settings.endGroup();

    settings.beginGroup("LivoxMid360");
    settings.setValue("livoxMid360Enabled", false);
    settings.setValue(
        "livoxMid360SdkRoot",
        QStringLiteral("third_party/Livox-SDK2"));
    settings.setValue("livoxMid360ConfigFile", QStringLiteral("bin/mid360_config.json"));
    settings.setValue("livoxMid360Serial", QStringLiteral("47MCNCN0035510"));
    settings.setValue("livoxMid360DiscoveryTimeoutMs", 10000);
    settings.endGroup();

    settings.beginGroup("TfminiPlus");
    settings.setValue("tfminiPlusEnabled", false);
    settings.setValue("tfminiPlusPort", QString());
    settings.setValue("tfminiPlusBaudRate", 115200);
    settings.setValue("collisionThresholdMm", 0);
    settings.setValue("tfminiPlusLogFrames", false);
    settings.endGroup();

    settings.beginGroup("Hmi");
    settings.setValue("enabled", true);
    settings.setValue("tcpPort", 9900);
    settings.endGroup();

    settings.sync();
    qInfo(LOG_CONFIG) << "已在" << settings.fileName() << "生成默认 config.ini";
}

void ConfigManager::loadStationProfile(QSettings& settings, const QString& configFilePath)
{
    StationProfile profile;
    QString profileIni;

    settings.beginGroup(QStringLiteral("Station"));
    applyStationSettings(settings, profile, &profileIni);
    settings.endGroup();

    // Stage 1 merge priority:
    // 1. config.ini [Station] is the base.
    // 2. profileIni, when present and existing, overrides same-name fields.
    // 3. The merged StationProfile is read-only for this stage.
    const QString resolvedProfileIni = resolveConfigRelativePath(profileIni, configFilePath);
    if (!resolvedProfileIni.isEmpty() && QFileInfo::exists(resolvedProfileIni)) {
        QSettings profileSettings(resolvedProfileIni, QSettings::IniFormat);
        profileSettings.beginGroup(QStringLiteral("Station"));
        applyStationSettings(profileSettings, profile, nullptr);
        profileSettings.endGroup();
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("[Station] 已合并 profileIni=") << resolvedProfileIni;
    } else if (!profileIni.isEmpty()) {
        qWarning(LOG_CONFIG).noquote()
            << QStringLiteral("[Station] profileIni 不存在，忽略：")
            << resolvedProfileIni;
    }

    m_stationProfile = profile;
    qInfo(LOG_CONFIG).noquote()
        << QStringLiteral("[Station] stationId=") << stationIdToInt(m_stationProfile.stationId)
        << QStringLiteral(" name=") << m_stationProfile.stationName
        << QStringLiteral(" scanPaths=") << (m_stationProfile.scanPathsConfigPath.isEmpty()
                                                ? QStringLiteral("<fallback scan_paths_config.json>")
                                                : m_stationProfile.scanPathsConfigPath)
        << QStringLiteral(" workMode=") << workModeIdToString(m_stationProfile.defaultWorkMode);
}

void ConfigManager::load(const QString& filePath)
{
    const QFileInfo fileInfo(filePath);
    const bool fileExists = fileInfo.exists() && fileInfo.size() > 0;
    m_configFilePath = QDir::cleanPath(filePath);

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

    loadStationProfile(settings, filePath);

    settings.beginGroup("Vision");
    m_visionConfig.mechEyeCameraKey = settings.value("mechEyeCameraKey", m_cameraConfig.defaultCamera).toString();
    m_visionConfig.mechCaptureTimeoutMs = settings.value("mechCaptureTimeoutMs", m_cameraConfig.scanTimeoutMs).toInt();
    m_visionConfig.mechDepthRangeMin = settings.value("mechDepthRangeMin", 100).toInt();
    m_visionConfig.mechDepthRangeMax = settings.value("mechDepthRangeMax", 2000).toInt();
    m_visionConfig.hikConnectTimeoutMs = settings.value("hikConnectTimeoutMs", 3000).toInt();
    m_visionConfig.hikCaptureTimeoutMs = settings.value("hikCaptureTimeoutMs", 1000).toInt();
    m_visionConfig.hikExposureTimeUs =
        static_cast<float>(settings.value("hikExposureTimeUs", 50000).toDouble());
    m_visionConfig.hikGain = static_cast<float>(settings.value("hikGain", 0.0).toDouble());
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
    m_visionConfig.hikCxpEnabled = settings.value("hikCxpEnabled", false).toBool();
    m_visionConfig.hikCxpCaptureTimeoutMs = settings.value("hikCxpCaptureTimeoutMs", 5000).toInt();
    m_visionConfig.hikCxpExposureTimeUs =
        static_cast<float>(settings.value("hikCxpExposureTimeUs", 50000).toDouble());
    m_visionConfig.hikCxpGain = static_cast<float>(settings.value("hikCxpGain", 0.0).toDouble());
    m_visionConfig.hikCxpSmokeOutputDir =
        settings.value("hikCxpSmokeOutputDir", "D:/CxpSmokeTest").toString();
    m_visionConfig.hikCxpCameraA.logicalName =
        settings.value("hikCxpCameraAName", "ch250_a").toString();
    m_visionConfig.hikCxpCameraA.cameraKey =
        settings.value("hikCxpCameraAKey", "DA9122997").toString();
    m_visionConfig.hikCxpCameraA.serialNumber =
        settings.value("hikCxpCameraASerial", "DA9122997").toString();
    m_visionConfig.hikCxpCameraB.logicalName =
        settings.value("hikCxpCameraBName", "ch250_b").toString();
    m_visionConfig.hikCxpCameraB.cameraKey =
        settings.value("hikCxpCameraBKey", "DA9122998").toString();
    m_visionConfig.hikCxpCameraB.serialNumber =
        settings.value("hikCxpCameraBSerial", "DA9122998").toString();
    settings.endGroup();

    settings.beginGroup("FlowControl");
    m_flowControlConfig.pollIntervalMs = settings.value("pollIntervalMs", 100).toInt();
    m_flowControlConfig.heartbeatIntervalMs = settings.value("heartbeatIntervalMs", 1000).toInt();
    m_flowControlConfig.simulatedProcessingMs = settings.value("simulatedProcessingMs", 300).toInt();
    settings.endGroup();

    settings.beginGroup("Tracking");
    m_trackingConfig.scanSegmentTotal = settings.value("scanSegmentTotal", 3).toInt();
    settings.endGroup();

    settings.beginGroup("OrbbecGemini");
    m_orbbecGeminiConfig.enabled = settings.value("orbbecGeminiEnabled", false).toBool();
    m_orbbecGeminiConfig.sdkRoot = settings.value(
        "orbbecGeminiSdkRoot",
        QStringLiteral("C:/Program Files/OrbbecSDK 2.8.6")).toString();
    m_orbbecGeminiConfig.serial = settings.value("orbbecGeminiSerial", QString()).toString();
    m_orbbecGeminiConfig.deviceIndex = settings.value("orbbecGeminiDeviceIndex", 0).toInt();
    m_orbbecGeminiConfig.depthWidth = settings.value("orbbecGeminiDepthWidth", 640).toInt();
    m_orbbecGeminiConfig.depthHeight = settings.value("orbbecGeminiDepthHeight", 480).toInt();
    m_orbbecGeminiConfig.fps = settings.value("orbbecGeminiFps", 15).toInt();
    m_orbbecGeminiConfig.captureTimeoutMs =
        settings.value("orbbecGeminiCaptureTimeoutMs", 5000).toInt();
    m_orbbecGeminiConfig.warmupFrameCount =
        settings.value("orbbecGeminiWarmupFrameCount", 5).toInt();
    m_orbbecGeminiConfig.saveCaptureToDisk =
        settings.value("orbbecGeminiSaveCaptureToDisk", true).toBool();
    m_orbbecGeminiConfig.captureCacheDir =
        settings.value("orbbecGeminiCaptureCacheDir", QString()).toString().trimmed();
    m_orbbecGeminiConfig.enableColorStream =
        settings.value("orbbecGeminiEnableColorStream", false).toBool();
    m_orbbecGeminiConfig.captureOnStart =
        settings.value("orbbecGeminiCaptureOnStart", false).toBool();
    settings.endGroup();

    settings.beginGroup("LivoxMid360");
    m_livoxMid360Config.enabled = settings.value("livoxMid360Enabled", false).toBool();
    m_livoxMid360Config.sdkRoot = settings.value(
        "livoxMid360SdkRoot",
        QStringLiteral("third_party/Livox-SDK2")).toString();
    m_livoxMid360Config.configFile = settings.value(
        "livoxMid360ConfigFile",
        QStringLiteral("bin/mid360_config.json")).toString();
    m_livoxMid360Config.serial = settings.value("livoxMid360Serial", QString()).toString();
    m_livoxMid360Config.discoveryTimeoutMs =
        settings.value("livoxMid360DiscoveryTimeoutMs", 10000).toInt();
    settings.endGroup();

    settings.beginGroup("TfminiPlus");
    m_tfminiPlusConfig.enabled = settings.value("tfminiPlusEnabled", false).toBool();
    m_tfminiPlusConfig.portName = settings.value("tfminiPlusPort", QString()).toString().trimmed();
    m_tfminiPlusConfig.baudRate = settings.value("tfminiPlusBaudRate", 115200).toInt();
    m_tfminiPlusConfig.collisionThresholdMm =
        settings.value("collisionThresholdMm", 0).toInt();
    m_tfminiPlusConfig.logFrames =
        settings.value("tfminiPlusLogFrames", false).toBool();
    settings.endGroup();

    settings.beginGroup("Hmi");
    {
        const int port = settings.value("tcpPort", 9900).toInt();
        m_hmiConfig.tcpPort = static_cast<quint16>(
            qBound(1, port, 65535));
    }
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
        << QStringLiteral("Modbus 配置：")
        << QStringLiteral(" host=") << m_modbusConfig.host
        << QStringLiteral(" port=") << m_modbusConfig.port
        << QStringLiteral(" unitId=") << m_modbusConfig.unitId
        << QStringLiteral(" timeoutMs=") << m_modbusConfig.timeoutMs
        << QStringLiteral(" reconnectIntervalMs=") << m_modbusConfig.reconnectIntervalMs;
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

    const QJsonArray pathsArray = root.value("scanPaths").toArray();
    m_scanPathsConfig.scanPaths.clear();
    m_scanPathsConfig.scanPaths.reserve(pathsArray.size());
    
    for (const QJsonValue& pathValue : pathsArray) {
        const QJsonObject pathObj = pathValue.toObject();
        
        ScanPathConfig pathConfig;
        pathConfig.pathId = pathObj.value("pathId").toInt();
        pathConfig.enabled = pathObj.value("enabled").toBool(true);
        pathConfig.totalPoints = pathObj.value("totalPoints").toInt();

        const QJsonArray pointsArray = pathObj.value("points").toArray();
        pathConfig.points.clear();
        pathConfig.points.reserve(pointsArray.size());
        
        for (const QJsonValue& pointValue : pointsArray) {
            const QJsonObject pointObj = pointValue.toObject();
            
            ScanPointConfig pointConfig;
            pointConfig.pointIndex = pointObj.value("pointIndex").toInt();
            pointConfig.needRotation = pointObj.value("needRotation").toBool(false);
            
            pathConfig.points.push_back(pointConfig);
        }
        
        m_scanPathsConfig.scanPaths.push_back(pathConfig);
    }

    QString validationError;
    if (!validateScanPathsConfig(&validationError)) {
        qWarning(LOG_CONFIG) << "扫描路径配置验证失败：" << validationError;
    }

    qInfo(LOG_CONFIG) << "已从以下位置加载扫描路径配置：" << jsonFilePath;
    qInfo(LOG_CONFIG).noquote()
        << "扫描路径配置："
        << "版本=" << m_scanPathsConfig.version
        << "路径数=" << m_scanPathsConfig.scanPaths.size();
    
    // 输出每条路径的详细信息
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        qInfo(LOG_CONFIG).noquote()
            << "  路径" << path.pathId
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

    return true;
}

}  // namespace common
}  // namespace scan_tracking
