#pragma once

#include <QSettings>
#include <QString>
#include <QtCore/QtGlobal>
#include <array>
#include <vector>

namespace scan_tracking {
namespace common {

struct AppConfig {
    QString version;
    QString environment;
};

struct LoggerConfig {
    int level;
    // 保留字段：当前未实现自动清理/轮转，历史日志永久保留。
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
    // "exclusive"（默认，独占控制）或 "monitor"（只读监控，允许与 SCMVS 共存）
    QString accessMode = QStringLiteral("exclusive");
};

struct VisionConfig {
    QString mechEyeCameraKey;
    int mechCaptureTimeoutMs;
    int mechDepthRangeMin;
    int mechDepthRangeMax;
    int hikConnectTimeoutMs;
    int hikCaptureTimeoutMs;
    float hikExposureTimeUs = 50000.0f;  ///< 海康 A/B 双目曝光（微秒），连接时写入相机
    float hikGain = 0.0f;              ///< 海康 A/B 增益（dB）
    QString hikSdkRoot;
    VisionCameraEndpointConfig hikCameraA;
    VisionCameraEndpointConfig hikCameraB;
    VisionCameraEndpointConfig hikCameraC;
    // 智能相机 C 的 TCP 服务端参数（IPC 作为服务端，相机主动连入）
    QString hikCameraCTcpListenIp;
    quint16 hikCameraCTcpListenPort;
    // 智能相机 C 的 FTP 落图目录（FileZilla Server 存图根目录）
    QString hikCameraCFtpDirectory;
};

struct FlowControlConfig {
    int pollIntervalMs;
    int heartbeatIntervalMs;
    int simulatedProcessingMs;
    QString scanCacheDirectory;  ///< 采集缓存根目录，空则 <app>/ScanTracking_CaptureCache
    bool retainSegmentPly = true; ///< true：检测/复位后不删磁盘 PLY（现场复盘）
};

struct TrackingConfig {
    int firstStationOuterSegmentIndex;
    int firstStationInnerSegmentIndex;
    int firstStationHoleSegmentIndex;
    int scanSegmentTotal;  // 扫描段总数（从 config.ini 获取，PLC不下发）
};

/// HMI 显控 TCP 服务配置（[Hmi]）
struct HmiConfig {
    bool enabled = true;       ///< 是否启动 HMI TCP 服务端
    quint16 tcpPort = 9900;    ///< 监听端口
    /// 是否允许显控发送 cmd.debug_trigger_inspection（用缓存点云跑蓝友并推送，不写 PLC）
    bool allowDebugTriggerInspection = false;
};

struct LbPoseConfig {
    QString dataRoot;
    QString leftPattern;
    QString rightPattern;
    QString templateFile;
    float minDistance;
    float maxDistance;
    float cosTolerance;
    float minPercent;
};

/** LBN 位姿检测配置。生产环境请多工况标定，勿仅按单帧离线 success 放大容差。 */
struct LbnPoseConfig {
    bool enabled = true;
    /// TODO(marker): 转盘未装标记点联调时 true：跳过 LBN 检测，Rt 用 4×4 单位阵，T0' 不变。
    bool useIdentityRtWithoutMarkers = false;
    QString dataRoot;
    QString templateFile;
    float minDistance = 30.0f;   // mm，过小易纳入杂点三角形
    float maxDistance = 650.0f;
    float cosTolerance = 0.015f; // 过大易误匹配模板点
    float minPercent = 0.5f;     // 与 FastGeoHash::getResult 联动
    int cloudSearchRadiusPx = 20;
    int markerMinArea = 400;
    int markerMaxArea = 30000;
    int markerIntensityThreshold = 50;
    float markerDebscanDistPx = 300.0f;
};

/**
 * @brief 扫描点位配置
 *
 * 定义单个扫描点位的参数；转盘角度由 LBN/状态机运行时解算，不在配置中指定。
 */
struct ScanPointConfig {
    int pointIndex;           // 点位索引（从 1 开始）
    bool needRotation;        // 是否需要转动转盘（关键标志）
};

/**
 * @brief 扫描路径配置
 *
 * 定义一条完整的扫描路径，包含多个点位。
 */
struct ScanPathConfig {
    int pathId;               // 路径唯一标识符
    bool enabled;             // 是否启用此路径
    int totalPoints;          // 路径包含的点位总数
    std::vector<ScanPointConfig> points;  // 点位配置列表
};

/**
 * @brief 扫描路径总配置
 * 
 * 包含所有扫描路径配置、标定矩阵、执行策略等。
 * 从 scan_paths_config.json 文件加载。
 */
struct ScanPathsConfig {
    // 标定矩阵 T0（4x4 矩阵，行优先存储）
    std::array<float, 16> calibrationMatrixT0;
    
    // 所有扫描路径定义
    std::vector<ScanPathConfig> scanPaths;
    
    // 执行策略
    bool executeAllPaths;                // 是否执行所有启用的路径
    std::vector<int> selectedPathIds;    // 指定要执行的路径 ID 列表
    bool allowPathSkipOnError;           // 路径失败时是否跳过继续执行
    
    // 转盘配置
    bool turntableEnabled;               // 是否启用转盘控制
    
    // 配置文件元数据
    QString version;                     // 配置文件版本
    QString lastModified;                // 最后修改时间
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
    const HmiConfig& hmiConfig() const;
    const LbPoseConfig& lbPoseConfig() const;
    const LbnPoseConfig& lbnPoseConfig() const;
    const ScanPathsConfig& scanPathsConfig() const;  // 新增：获取扫描路径配置

private:
    ConfigManager();
    ~ConfigManager();

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    void load(const QString& filePath);
    void writeDefaults(QSettings& settings);
    
    // 新增：加载扫描路径配置（JSON 格式）
    void loadScanPathsConfig(const QString& jsonFilePath);
    
    // 新增：验证扫描路径配置的合法性
    bool validateScanPathsConfig(QString* errorMessage = nullptr) const;

    static ConfigManager* s_instance;

    AppConfig m_appConfig;
    LoggerConfig m_loggerConfig;
    ModbusConfig m_modbusConfig;
    CameraConfig m_cameraConfig;
    VisionConfig m_visionConfig;
    FlowControlConfig m_flowControlConfig;
    TrackingConfig m_trackingConfig;
    HmiConfig m_hmiConfig;
    LbPoseConfig m_lbPoseConfig;
    LbnPoseConfig m_lbnPoseConfig;
    ScanPathsConfig m_scanPathsConfig;  // 新增：扫描路径配置
};

}  // namespace common
}  // namespace scan_tracking
