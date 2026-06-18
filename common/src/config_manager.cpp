/**
 * @file config_manager.cpp
 * @brief ConfigManager 实现：加载 config.ini、工位 profile 与 scan_paths JSON
 *
 * 配置文件查找顺序（projectRootConfigPath）：
 * 1. 可执行文件同目录下的 config.ini
 * 2. 自 exe 向上三级目录（开发态 CMake 构建树中的项目根）
 *
 * 相对路径解析（resolveConfigRelativePath）：config.ini 所在目录 → exe 目录。
 */

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

/**
 * @brief 定位主配置文件 config.ini 的路径
 *
 * 优先使用与可执行文件同目录的 config.ini（部署场景）；
 * 若不存在则尝试 CMake 开发构建树中的项目根目录。
 */
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

/**
 * @brief 将配置中的相对路径解析为绝对路径
 *
 * 解析顺序：已是绝对路径则直接规范化；否则依次尝试
 * config.ini 所在目录、可执行文件目录。均不存在时仍返回 config 目录下的拼接结果。
 *
 * @param rawPath         配置项中的原始路径字符串
 * @param configFilePath  主 config.ini 的绝对路径（用作相对路径基准）
 */
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
 * @brief 确定 scan_paths JSON 文件的完整路径
 *
 * 优先级：
 * 1. StationProfile::scanPathsConfigPath（来自 [Station] 或 profileIni 覆盖）
 * 2. 默认 fallback：config/scan_paths/station2_placeholder.json
 *
 * @param stationProfile  已合并的工位 profile
 * @param configFilePath  主 config.ini 路径，用于相对路径解析
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

/**
 * @brief 从 QSettings 的 [Station] 节读取字段并写入 profile
 *
 * 仅当键存在时才覆盖 profile 中对应字段，便于多层配置合并。
 * 若 profileIni 非空且 settings 含 profileIni 键，则将其路径写入 *profileIni。
 *
 * @param settings    已定位到 [Station] 组内或即将 beginGroup 的 QSettings
 * @param profile     读入目标，就地修改
 * @param profileIni  可选输出；非 nullptr 时读取 profileIni 键
 */
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
    // 幂等：多次调用仅首次分配单例；构造函数内完成 load + loadScanPathsConfig
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

    // load() 已解析 StationProfile，据此定位 scan_paths JSON 并加载
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

const ScanPointConfig* ConfigManager::findScanPointByIndex(int segmentIndex) const
{
    // 仅搜索 enabled 路径；返回指向内部 vector 元素的指针，生命周期与 ConfigManager 相同
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (!path.enabled) {
            continue;
        }
        for (const auto& point : path.points) {
            if (point.pointIndex == segmentIndex) {
                return &point;
            }
        }
    }
    return nullptr;
}

int ConfigManager::enabledScanPointCount() const
{
    // 累加所有已启用路径的 points 数量；JSON 未加载或全禁用时为 0
    int total = 0;
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (path.enabled) {
            total += static_cast<int>(path.points.size());
        }
    }
    return total;
}

/**
 * @brief 写入 config.ini 各节的出厂默认值
 *
 * 在 config.ini 不存在或为空时由 load() 调用；写入后立即 sync() 落盘。
 * 默认值与 load() 中的 fallback 值保持一致，便于首次部署直接运行。
 */
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
    settings.setValue("hikSdkRoot", "third_party/MVS");
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

/**
 * @brief 加载并合并工位 profile
 *
 * 合并优先级（Stage 1）：
 * 1. config.ini [Station] 作为基础
 * 2. profileIni 指向的外部 INI 中 [Station] 同名字段覆盖基础值
 * 3. 结果写入 m_stationProfile，本阶段为只读配置
 */
void ConfigManager::loadStationProfile(QSettings& settings, const QString& configFilePath)
{
    StationProfile profile;
    QString profileIni;

    settings.beginGroup(QStringLiteral("Station"));
    applyStationSettings(settings, profile, &profileIni);
    settings.endGroup();

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

/**
 * @brief 从 config.ini 加载全部 INI 配置到成员结构体
 *
 * 按节顺序读取：App → Logger → Modbus → Camera → Station（profile）→
 * Vision → FlowControl → Tracking → OrbbecGemini → LivoxMid360 → TfminiPlus → Hmi。
 * 读取 Logger.level 后同步设置 Logger 单例的最低输出级别。
 */
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

    // --- [App] ---
    settings.beginGroup("App");
    m_appConfig.version = settings.value("version", "0.1.0").toString();
    m_appConfig.environment = settings.value("environment", "production").toString();
    settings.endGroup();

    // --- [Logger] ---
    settings.beginGroup("Logger");
    m_loggerConfig.level = settings.value("level", 0).toInt();
    m_loggerConfig.rotateDays = settings.value("rotateDays", 7).toInt();
    settings.endGroup();

    // --- [Modbus] ---
    settings.beginGroup("Modbus");
    m_modbusConfig.host = settings.value("host", "127.0.0.1").toString();
    m_modbusConfig.port = settings.value("port", 502).toInt();
    m_modbusConfig.unitId = settings.value("unitId", 1).toInt();
    m_modbusConfig.timeoutMs = settings.value("timeoutMs", 1000).toInt();
    m_modbusConfig.reconnectIntervalMs = settings.value("reconnectIntervalMs", 2000).toInt();
    settings.endGroup();

    // --- [Camera] ---
    settings.beginGroup("Camera");
    m_cameraConfig.defaultCamera = settings.value("defaultCamera", "Mech-Eye Nano").toString();
    m_cameraConfig.scanTimeoutMs = settings.value("scanTimeoutMs", 5000).toInt();
    settings.endGroup();

    // --- [Station] + profileIni 合并 ---
    loadStationProfile(settings, filePath);

    // --- [Vision] ---
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
    m_visionConfig.hikSdkRoot = settings.value("hikSdkRoot", "third_party/MVS").toString();
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

    // --- [FlowControl] ---
    settings.beginGroup("FlowControl");
    m_flowControlConfig.pollIntervalMs = settings.value("pollIntervalMs", 100).toInt();
    m_flowControlConfig.heartbeatIntervalMs = settings.value("heartbeatIntervalMs", 1000).toInt();
    m_flowControlConfig.simulatedProcessingMs = settings.value("simulatedProcessingMs", 300).toInt();
    settings.endGroup();

    // --- [Tracking] ---
    settings.beginGroup("Tracking");
    m_trackingConfig.scanSegmentTotal = settings.value("scanSegmentTotal", 3).toInt();
    settings.endGroup();

    // --- [OrbbecGemini] ---
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

    // --- [LivoxMid360] ---
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

    // --- [TfminiPlus] ---
    settings.beginGroup("TfminiPlus");
    m_tfminiPlusConfig.enabled = settings.value("tfminiPlusEnabled", false).toBool();
    m_tfminiPlusConfig.portName = settings.value("tfminiPlusPort", QString()).toString().trimmed();
    m_tfminiPlusConfig.baudRate = settings.value("tfminiPlusBaudRate", 115200).toInt();
    m_tfminiPlusConfig.collisionThresholdMm =
        settings.value("collisionThresholdMm", 0).toInt();
    m_tfminiPlusConfig.logFrames =
        settings.value("tfminiPlusLogFrames", false).toBool();
    settings.endGroup();

    // --- [Hmi] ---
    settings.beginGroup("Hmi");
    {
        const int port = settings.value("tcpPort", 9900).toInt();
        m_hmiConfig.tcpPort = static_cast<quint16>(
            qBound(1, port, 65535));
    }
    settings.endGroup();

    // 将 Logger.level 映射为 Qt 消息级别并应用到 Logger 单例
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
 * 期望的根对象字段：
 * - version, lastModified：元数据
 * - scanPaths[]：每条含 pathId, enabled, segmentKind, description,
 *   totalPoints, points[]（pointIndex, needRotation）
 *
 * 解析完成后调用 validateScanPathsConfig；校验失败仅警告，不阻止加载。
 * 文件缺失或解析失败时保留空的 m_scanPathsConfig。
 *
 * @param jsonFilePath 由 scanPathsConfigPath() 解析得到的 JSON 绝对路径
 */
void ConfigManager::loadScanPathsConfig(const QString& jsonFilePath)
{
    if (!QFileInfo::exists(jsonFilePath)) {
        qWarning(LOG_CONFIG) << "扫描路径配置文件不存在：" << jsonFilePath;
        qWarning(LOG_CONFIG) << "将使用空配置，多路径扫描功能不可用。";
        return;
    }

    // 读取并解析 JSON 文档
    QFile file(jsonFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qCritical(LOG_CONFIG) << "无法打开扫描路径配置文件：" << jsonFilePath;
        return;
    }
    
    const QByteArray jsonData = file.readAll();
    file.close();

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
        pathConfig.segmentKind = pathObj.value("segmentKind").toString(QStringLiteral("external"));
        pathConfig.description = pathObj.value("description").toString();
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

    for (const auto& path : m_scanPathsConfig.scanPaths) {
        qInfo(LOG_CONFIG).noquote()
            << "  路径" << path.pathId
            << "启用=" << path.enabled
            << "类型=" << path.segmentKind
            << "点位数=" << path.points.size();
    }
}

/**
 * @brief 验证扫描路径配置的合法性
 *
 * 规则：
 * - 所有路径的 pathId 互不重复
 * - 每条路径 points.size() 必须等于 totalPoints
 * - 已启用路径内：pointIndex > 0，且全局段号在启用路径间不重复
 *
 * @param errorMessage 可选；失败时写入中文错误描述
 * @return true 表示通过全部检查
 */
bool ConfigManager::validateScanPathsConfig(QString* errorMessage) const
{
    std::vector<int> globalPointIndices;

    // 遍历所有路径，检查 ID 唯一性与点位数量
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

        if (static_cast<int>(path.points.size()) != path.totalPoints) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 %1 的点位数量不匹配：配置 %2，实际 %3")
                    .arg(path.pathId)
                    .arg(path.totalPoints)
                    .arg(path.points.size());
            }
            return false;
        }

        // 仅对已启用路径校验全局段号唯一性与正整数约束
        if (path.enabled) {
            for (const auto& point : path.points) {
                if (point.pointIndex <= 0) {
                    if (errorMessage) {
                        *errorMessage = QStringLiteral("路径 %1 的点位索引无效：%2")
                            .arg(path.pathId)
                            .arg(point.pointIndex);
                    }
                    return false;
                }
                if (std::find(globalPointIndices.begin(), globalPointIndices.end(), point.pointIndex) !=
                    globalPointIndices.end()) {
                    if (errorMessage) {
                        *errorMessage = QStringLiteral("全局段号重复：%1").arg(point.pointIndex);
                    }
                    return false;
                }
                globalPointIndices.push_back(point.pointIndex);
            }
        }
    }

    return true;
}

}  // namespace common
}  // namespace scan_tracking
