#pragma once

#include <QSettings>
#include <QString>
#include <QVector>
#include <QtCore/QtGlobal>
#include <vector>

#include "scan_tracking/common/station_profile.h"

namespace scan_tracking {
namespace common {

struct AppConfig {
    QString version;
    QString environment;
};

struct LoggerConfig {
    int level;
    int rotateDays;
};

struct ModbusConfig {
    QString host;
    int port;
    int unitId;
    int timeoutMs;
    int reconnectIntervalMs;
};

struct CameraConfig {
    QString defaultCamera;
    int scanTimeoutMs;
};

struct VisionCameraEndpointConfig {
    QString logicalName;
    QString cameraKey;
    QString ipAddress;
    QString serialNumber;
    QString accessMode = QStringLiteral("exclusive");
};

struct VisionConfig {
    QString mechEyeCameraKey;
    int mechCaptureTimeoutMs;
    int mechDepthRangeMin;
    int mechDepthRangeMax;
    int hikConnectTimeoutMs;
    int hikCaptureTimeoutMs;
    float hikExposureTimeUs = 50000.0f;
    float hikGain = 0.0f;
    QString hikSdkRoot;
    VisionCameraEndpointConfig hikCameraA;
    VisionCameraEndpointConfig hikCameraB;
    VisionCameraEndpointConfig hikCameraC;
    bool hikCxpEnabled = false;
    int hikCxpCaptureTimeoutMs = 5000;
    float hikCxpExposureTimeUs = 50000.0f;
    float hikCxpGain = 0.0f;
    QString hikCxpSmokeOutputDir;
    VisionCameraEndpointConfig hikCxpCameraA;
    VisionCameraEndpointConfig hikCxpCameraB;
    QString hikCameraCTcpListenIp;
    quint16 hikCameraCTcpListenPort;
    QString hikCameraCFtpDirectory;
};

struct FlowControlConfig {
    int pollIntervalMs;
    int heartbeatIntervalMs;
    int simulatedProcessingMs;
};

struct TrackingConfig {
    int scanSegmentTotal = 3;
};

struct OrbbecGeminiConfig {
    bool enabled = false;
    QString sdkRoot;
    QString serial;
    int deviceIndex = 0;
    int depthWidth = 640;
    int depthHeight = 480;
    int fps = 15;
    int captureTimeoutMs = 5000;
    int warmupFrameCount = 5;
    bool saveCaptureToDisk = true;
    QString captureCacheDir;
    bool enableColorStream = false;
    bool captureOnStart = true;
};

struct LivoxMid360Config {
    bool enabled = false;
    QString sdkRoot;
    QString configFile;
    QString serial;
    int discoveryTimeoutMs = 10000;
};

struct TfminiPlusConfig {
    bool enabled = false;
    QString portName;
    int baudRate = 115200;
    int collisionThresholdMm = 0;
    bool logFrames = false;
};

struct HmiConfig {
    bool enabled = true;
    quint16 tcpPort = 9900;
};

struct ScanPointConfig {
    int pointIndex = 0;
    bool needRotation = false;
};

struct ScanPathConfig {
    int pathId = 0;
    bool enabled = true;
    int totalPoints = 0;
    std::vector<ScanPointConfig> points;
};

struct ScanPathsConfig {
    std::vector<ScanPathConfig> scanPaths;
    QString version;
    QString lastModified;
};

class ConfigManager {
public:
    static void initialize();
    static void cleanup();
    static ConfigManager* instance();

    const AppConfig& appConfig() const;
    const LoggerConfig& loggerConfig() const;
    const ModbusConfig& modbusConfig() const;
    const CameraConfig& cameraConfig() const;
    const VisionConfig& visionConfig() const;
    const FlowControlConfig& flowControlConfig() const;
    const TrackingConfig& trackingConfig() const;
    const OrbbecGeminiConfig& orbbecGeminiConfig() const;
    const LivoxMid360Config& livoxMid360Config() const;
    const TfminiPlusConfig& tfminiPlusConfig() const;
    QString configFilePath() const;
    const HmiConfig& hmiConfig() const;
    const ScanPathsConfig& scanPathsConfig() const;
    const StationProfile& stationProfile() const;

private:
    ConfigManager();
    ~ConfigManager();

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    void load(const QString& filePath);
    void loadStationProfile(QSettings& settings, const QString& configFilePath);
    void writeDefaults(QSettings& settings);
    void loadScanPathsConfig(const QString& jsonFilePath);
    bool validateScanPathsConfig(QString* errorMessage = nullptr) const;

    static ConfigManager* s_instance;

    AppConfig m_appConfig;
    LoggerConfig m_loggerConfig;
    ModbusConfig m_modbusConfig;
    CameraConfig m_cameraConfig;
    VisionConfig m_visionConfig;
    FlowControlConfig m_flowControlConfig;
    TrackingConfig m_trackingConfig;
    OrbbecGeminiConfig m_orbbecGeminiConfig;
    LivoxMid360Config m_livoxMid360Config;
    TfminiPlusConfig m_tfminiPlusConfig;
    QString m_configFilePath;
    HmiConfig m_hmiConfig;
    StationProfile m_stationProfile;
    ScanPathsConfig m_scanPathsConfig;
};

}  // namespace common
}  // namespace scan_tracking
