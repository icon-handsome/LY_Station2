#include "scan_tracking/app/console_runtime.h"

#ifdef _WIN32
#include <windows.h>
#ifdef round
#undef round
#endif
#ifdef Round
#undef Round
#endif
#endif

#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>
#include <QtCore/QSettings>
#include <QtCore/QString>
#include <QtCore/QTimer>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/tracking/tracking_service.h"
#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/hik_mono_io.h"
#include "scan_tracking/vision/vision_types.h"
#include "scan_tracking/hmi_server/hmi_tcp_server.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/common/capture_cache_paths.h"

Q_LOGGING_CATEGORY(appLog, "app")

namespace scan_tracking::app {

namespace {

#ifdef _WIN32
BOOL WINAPI handleConsoleSignal(DWORD signal)
{
    switch (signal) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        if (QCoreApplication::instance() != nullptr) {
            QMetaObject::invokeMethod(
                QCoreApplication::instance(),
                []() { QCoreApplication::quit(); },
                Qt::QueuedConnection);
            return TRUE;
        }
        return FALSE;
    default:
        return FALSE;
    }
}
#endif

}  // namespace

ConsoleRuntime::ConsoleRuntime(QCoreApplication& application)
    : application_(application)
{
}

ConsoleRuntime::~ConsoleRuntime() = default;

int ConsoleRuntime::run()
{
#ifdef _WIN32
    SetConsoleCtrlHandler(handleConsoleSignal, TRUE);
#endif

    QObject::connect(
        &application_,
        &QCoreApplication::aboutToQuit,
        &application_,
        [this]() { printShutdownStatus(); });

    printStartupStatus();
    initModules();
    return application_.exec();
}

void ConsoleRuntime::initModules()
{
    qInfo(appLog) << "正在初始化模块...";
    int startupStage = 5;
    const QByteArray startupStageEnv = qgetenv("SCAN_TRACKING_STARTUP_STAGE");
	bool startupStageOk = false;
    if (!startupStageEnv.isEmpty()) {
        startupStage = startupStageEnv.toInt(&startupStageOk);
        if (!startupStageOk) {
            startupStage = 5;
        }
    }
    qInfo(appLog) << "启动阶段 =" << startupStage
                  << "(0=Modbus, 1=+MechEye, 2=+Hik, 3=+VisionPipeline, 4=+Tracking, 5=+StateMachine)";

    modbusService_ = std::make_unique<scan_tracking::modbus::ModbusService>(&application_);
    qInfo(appLog) << "Modbus 服务已创建。";

    if (startupStage < 1) {
        qInfo(appLog) << "启动阶段仅到 Modbus。";
        if (!modbusService_->connectDevice()) {
            qWarning(appLog) << "Modbus 连接初始化失败。";
        }
        qInfo(appLog) << "所有模块已初始化。";
        return;
    }
    // MechEye 服务先启动，保证后续状态机和视觉集成层都能复用同一份点云采集入口。
    mechEyeService_ = std::make_unique<scan_tracking::mech_eye::MechEyeService>();

    QObject::connect(
        mechEyeService_.get(),
        &scan_tracking::mech_eye::MechEyeService::stateChanged,
        [](scan_tracking::mech_eye::CameraRuntimeState state, const QString& description) {
            qInfo(appLog) << "[MechEye] 状态 =" << static_cast<int>(state) << description;
        });

    QObject::connect(
        mechEyeService_.get(),
        &scan_tracking::mech_eye::MechEyeService::fatalError,
        [](scan_tracking::mech_eye::CaptureErrorCode code, const QString& message) {
            qCritical(appLog) << "[MechEye] 致命错误：" << static_cast<int>(code) << message;
        });

    mechEyeService_->start();
    qInfo(appLog) << "MechEye 服务已启动。";

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const auto visionConfig = configManager != nullptr
        ? configManager->visionConfig()
        : scan_tracking::common::VisionConfig{};

    // 两台海康相机当前只接入服务骨架，不落具体 SDK 调用。
    hikCameraAService_ = std::make_unique<scan_tracking::vision::HikCameraService>(
        QStringLiteral("hik_camera_a"));
    hikCameraBService_ = std::make_unique<scan_tracking::vision::HikCameraService>(
        QStringLiteral("hik_camera_b"));

    QObject::connect(
        hikCameraAService_.get(),
        &scan_tracking::vision::HikCameraService::stateChanged,
        [](const QString& roleName, const QString& stateText, const QString& description) {
            qInfo(appLog) << "[HikCamera]" << roleName << stateText << description;
        });
    QObject::connect(
        hikCameraBService_.get(),
        &scan_tracking::vision::HikCameraService::stateChanged,
        [](const QString& roleName, const QString& stateText, const QString& description) {
            qInfo(appLog) << "[HikCamera]" << roleName << stateText << description;
        });

    hikCameraAService_->start(
        visionConfig.hikCameraA,
        visionConfig.hikCaptureTimeoutMs,
        visionConfig.hikExposureTimeUs,
        visionConfig.hikGain);
    hikCameraBService_->start(
        visionConfig.hikCameraB,
        visionConfig.hikCaptureTimeoutMs,
        visionConfig.hikExposureTimeUs,
        visionConfig.hikGain);

    // 第三台海康相机（独立用途，不同型号 - 读码相机）
    hikCameraCService_ = std::make_unique<scan_tracking::vision::HikCameraService>(
        QStringLiteral("hik_camera_c"));

    QObject::connect(
        hikCameraCService_.get(),
        &scan_tracking::vision::HikCameraService::stateChanged,
        [](const QString& roleName, const QString& stateText, const QString& description) {
            qInfo(appLog) << "[HikCamera]" << roleName << stateText << description;
        });
    QObject::connect(
        hikCameraCService_.get(),
        &scan_tracking::vision::HikCameraService::fatalError,
        [](scan_tracking::vision::VisionErrorCode code, const QString& message) {
            qCritical(appLog) << "[HikCamera] hik_camera_c 致命错误：" 
                              << static_cast<int>(code) << message;
        });

    hikCameraCService_->start(visionConfig.hikCameraC, visionConfig.hikCaptureTimeoutMs);
    qInfo(appLog) << "HikCamera C 服务已启动（仅连接，不采集）。";

    // 海康相机 C 控制器（独立管理第三台相机 - 智能相机）
    // 使用 TCP 通信协议进行控制和图像获取
    hikCameraCController_ = std::make_unique<scan_tracking::vision::HikCameraCController>(
        hikCameraCService_.get());

    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::stateChanged,
        [](scan_tracking::vision::HikCameraCState state, const QString& description) {
            qInfo(appLog) << "[HikCameraCController] 状态 =" << static_cast<int>(state) << description;
        });
    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::fatalError,
        [](scan_tracking::vision::VisionErrorCode code, const QString& message) {
            qCritical(appLog) << "[HikCameraCController] 致命错误：" 
                              << static_cast<int>(code) << message;
        });
    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::captureCompleted,
        [](scan_tracking::vision::CaptureType type, const QByteArray& imageData) {
            QString typeStr;
            switch (type) {
                case scan_tracking::vision::CaptureType::SurfaceDefect:
                    typeStr = "SurfaceDefect";
                    break;
                case scan_tracking::vision::CaptureType::WeldDefect:
                    typeStr = "WeldDefect";
                    break;
                case scan_tracking::vision::CaptureType::NumberRecognition:
                    typeStr = "NumberRecognition";
                    break;
                default:
                    typeStr = "Unknown";
                    break;
            }
            qInfo(appLog) << "[HikCameraCController] 采集完成：" << typeStr
                          << imageData.size() << "字节";
        });

    hikCameraCController_->start(visionConfig);
    qInfo(appLog) << "HikCamera C 控制器已启动（TCP 通信模式）。";


    // 统一视觉编排层负责把“1 份点云 + 2 份矩阵”收口为一个算法输入包。
    visionPipelineService_ = std::make_unique<scan_tracking::vision::VisionPipelineService>(
        mechEyeService_.get(),
        hikCameraAService_.get(),
        hikCameraBService_.get());

    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::stateChanged,
        [](scan_tracking::vision::VisionPipelineState state, const QString& description) {
            qInfo(appLog) << "[VisionPipeline] 状态 =" << static_cast<int>(state) << description;
        });
    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::bundleCaptureFinished,
        &application_,
        [this](const scan_tracking::vision::MultiCameraCaptureBundle& bundle) {
            qInfo(appLog) << "[VisionPipeline]" << bundle.summary();
            if (m_autoLatencyTestPending) {
                onAutoLatencyBundleFinished(bundle);
            }
        });

    visionPipelineService_->start(visionConfig);
    qInfo(appLog) << "视觉集成框架已启动。";

    trackingService_ = std::make_unique<scan_tracking::tracking::TrackingService>();

    stateMachine_ = std::make_unique<scan_tracking::flow_control::StateMachine>(
        modbusService_.get(),
        mechEyeService_.get(),
        visionPipelineService_.get(),
        trackingService_.get(),
        &application_);

    // HMI：先注入依赖并绑定信号，再 listen / 启动状态机，避免 start() 内重复 connect 或漏接早期事件
    const auto& hmiConfig = scan_tracking::common::ConfigManager::instance()->hmiConfig();
    if (hmiConfig.enabled) {
        hmiTcpServer_ = std::make_unique<scan_tracking::hmi_server::HmiTcpServer>(
            static_cast<int>(hmiConfig.tcpPort), &application_);
        hmiTcpServer_->setStateMachine(stateMachine_.get());
        hmiTcpServer_->setModbusService(modbusService_.get());
        hmiTcpServer_->setMechEyeService(mechEyeService_.get());
        hmiTcpServer_->setVisionPipelineService(visionPipelineService_.get());
        hmiTcpServer_->setTrackingService(trackingService_.get());
        hmiTcpServer_->setHikCameraServices(
            hikCameraAService_.get(), hikCameraBService_.get(), hikCameraCService_.get());
        hmiTcpServer_->setHikCameraCController(hikCameraCController_.get());
        hmiTcpServer_->bindServiceSignals();

        // 演示初版：蓝友 inspectSegments 返回后立即经 HMI TCP 推送 event.inspection.finished（含失败）
        const auto publishInspectionToHmi =
            [this](const scan_tracking::tracking::InspectionResult& inspectionResult) {
                if (hmiTcpServer_) {
                    hmiTcpServer_->publishInspectionResult(inspectionResult);
                }
            };
        trackingService_->setInspectionResultNotifier(publishInspectionToHmi);
        stateMachine_->setInspectionResultPublisher(publishInspectionToHmi);

        if (!hmiTcpServer_->start()) {
            qWarning(appLog) << "HMI TCP 服务器在端口" << hmiConfig.tcpPort << "启动失败。";
        } else {
            qInfo(appLog) << "HMI TCP 服务器已在端口" << hmiConfig.tcpPort << "启动。";
        }
    } else {
        qInfo(appLog) << "HMI TCP 服务已在 config.ini [Hmi] enabled=false 下禁用。";
    }

    stateMachine_->start();

    // 等状态机接好信号后再建 Modbus 链路，避免启动期漏掉 connected 事件。
    if (!modbusService_->connectDevice()) {
        qWarning(appLog) << "Modbus 连接初始化失败。";
    }
    qInfo(appLog) << "所有模块已初始化。";
    scheduleAutoLatencyBundleTest();
}

void ConsoleRuntime::stopAutoLatencyBundleTest()
{
    if (m_autoLatencyTimer != nullptr) {
        m_autoLatencyTimer->stop();
    }
}

void ConsoleRuntime::scheduleAutoLatencyBundleTest()
{
    if (visionPipelineService_ == nullptr) {
        return;
    }

    QString iniPath = QCoreApplication::applicationDirPath() + QStringLiteral("/config.ini");
    QDir rootDir(QCoreApplication::applicationDirPath());
    if (rootDir.cdUp() && rootDir.cdUp() && rootDir.cdUp()) {
        const QString projectIni = rootDir.filePath(QStringLiteral("config.ini"));
        if (QFile::exists(projectIni)) {
            iniPath = projectIni;
        }
    }

    QSettings settings(iniPath, QSettings::IniFormat);
    settings.beginGroup(QStringLiteral("Debug"));
    const bool enabled = settings.value(QStringLiteral("autoLatencyBundleTestEnabled"), false).toBool();
    const int delayMs = settings.value(QStringLiteral("autoLatencyBundleTestDelayMs"), 20000).toInt();
    const bool periodic = settings.value(QStringLiteral("autoLatencyBundleTestPeriodic"), false).toBool();
    settings.endGroup();

    if (!enabled) {
        return;
    }

    m_autoLatencyPeriodic = periodic;
    m_autoLatencyIntervalMs = delayMs > 0 ? delayMs : 20000;
    m_latencyBundleSeq = 0;

    if (m_autoLatencyTimer == nullptr) {
        m_autoLatencyTimer = new QTimer(&application_);
        QObject::connect(m_autoLatencyTimer, &QTimer::timeout, &application_, [this]() {
            triggerAutoLatencyBundleTest();
        });
    }

    m_autoLatencyTimer->setInterval(m_autoLatencyIntervalMs);

    if (m_autoLatencyPeriodic) {
        m_autoLatencyTimer->setSingleShot(false);
        m_autoLatencyTimer->start();
        qInfo(appLog).noquote()
            << "[LatencyTest] 周期模式已启动：每" << m_autoLatencyIntervalMs
            << "ms 组合拍一次（Mech 2D + 海康 A/B）；关闭周期请设 autoLatencyBundleTestPeriodic=false";
    } else {
        m_autoLatencyTimer->setSingleShot(true);
        m_autoLatencyTimer->start();
        qInfo(appLog).noquote()
            << "[LatencyTest] 单次模式已调度：" << m_autoLatencyIntervalMs
            << "ms 后触发一次（Mech 2D + 海康 A/B）；关闭请设 autoLatencyBundleTestEnabled=false";
    }
}

void ConsoleRuntime::triggerAutoLatencyBundleTest()
{
    if (visionPipelineService_ == nullptr || !visionPipelineService_->isStarted()) {
        qWarning(appLog) << "[LatencyTest] 视觉流水线未就绪，跳过自动采集。";
        return;
    }

    if (m_autoLatencyTestPending) {
        const QString modeText =
            m_autoLatencyPeriodic ? QStringLiteral("周期") : QStringLiteral("单次");
        qWarning(appLog).noquote()
            << QStringLiteral("[LatencyTest] 上一轮采集尚未结束，跳过本次%1触发").arg(modeText);
        return;
    }

    const qint64 triggerMs = QDateTime::currentMSecsSinceEpoch();
    m_autoLatencyTriggerMs = triggerMs;
    m_autoLatencyTestPending = true;

    constexpr int kTestSegmentIndex = 1;
    const quint32 roundIndex = m_latencyBundleSeq;
    const quint32 taskId = 90001u + roundIndex;
    m_latencyBundleSeq = roundIndex + 1u;

    const QString periodicText = m_autoLatencyPeriodic ? QStringLiteral("true") : QStringLiteral("false");

    qInfo(appLog).noquote()
        << QStringLiteral("[ScanSync] trigger %1 (auto latency test round=%2)")
               .arg(triggerMs)
               .arg(roundIndex);

    const quint64 requestId = visionPipelineService_->requestCaptureBundle(
        kTestSegmentIndex,
        taskId,
        scan_tracking::mech_eye::CaptureMode::Capture2DOnly);

    if (requestId == 0) {
        m_autoLatencyTestPending = false;
        qWarning(appLog) << "[LatencyTest] 组合采集请求被拒绝。";
        return;
    }

    qInfo(appLog).noquote()
        << QStringLiteral(
               "[LatencyTest] 已发送组合采集 requestId=%1 segment=%2 taskId=%3 mechMode=Capture2DOnly "
               "periodic=%4")
               .arg(requestId)
               .arg(kTestSegmentIndex)
               .arg(taskId)
               .arg(periodicText);
}

void ConsoleRuntime::onAutoLatencyBundleFinished(
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    m_autoLatencyTestPending = false;

    const qint64 triggerMs = m_autoLatencyTriggerMs;
    const quint32 roundCount = m_latencyBundleSeq;
    quint32 roundIndex = 0u;
    quint32 taskId = 90001u;
    if (roundCount > 0u) {
        roundIndex = roundCount - 1u;
        taskId = 90000u + roundCount;
    }

    const qint64 finishedMs = QDateTime::currentMSecsSinceEpoch();
    qint64 wallMs = -1;
    if (triggerMs > 0) {
        wallMs = finishedMs - triggerMs;
    }

    const bool mechOk = bundle.mechEyeResult.success();
    const qint64 mechElapsedMs = bundle.mechEyeResult.elapsedMs;
    const int mechTexW = bundle.mechEyeResult.texture2D.width;
    const int mechTexH = bundle.mechEyeResult.texture2D.height;

    const bool hikAOk = bundle.hikCameraAResult.success();
    const qint64 hikAElapsedMs = bundle.hikCameraAResult.elapsedMs;
    const int hikAW = bundle.hikCameraAResult.frame.width;
    const int hikAH = bundle.hikCameraAResult.frame.height;
    const qint64 hikATs = bundle.hikCameraAResult.frame.timestampMs;

    const bool hikBOk = bundle.hikCameraBResult.success();
    const qint64 hikBElapsedMs = bundle.hikCameraBResult.elapsedMs;
    const int hikBW = bundle.hikCameraBResult.frame.width;
    const int hikBH = bundle.hikCameraBResult.frame.height;
    const qint64 hikBTs = bundle.hikCameraBResult.frame.timestampMs;

    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] ========== 延时汇总 round=%1 ==========").arg(roundIndex);
    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] 触发时刻 triggerMs=%1 完成时刻 finishedMs=%2 墙钟总耗时 wallMs=%3")
               .arg(triggerMs)
               .arg(finishedMs)
               .arg(wallMs);
    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] Mech 2D: success=%1 elapsedMs=%2 texture=%3x%4")
               .arg(mechOk)
               .arg(mechElapsedMs)
               .arg(mechTexW)
               .arg(mechTexH);
    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] Hik A: success=%1 elapsedMs=%2 frame=%3x%4 timestampMs=%5")
               .arg(hikAOk)
               .arg(hikAElapsedMs)
               .arg(hikAW)
               .arg(hikAH)
               .arg(hikATs);
    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] Hik B: success=%1 elapsedMs=%2 frame=%3x%4 timestampMs=%5")
               .arg(hikBOk)
               .arg(hikBElapsedMs)
               .arg(hikBW)
               .arg(hikBH)
               .arg(hikBTs);

    if (triggerMs > 0) {
        if (hikATs > 0) {
            qInfo(appLog).noquote()
                << QStringLiteral("[LatencyTest] Hik A 相对触发 deltaMs=%1").arg(hikATs - triggerMs);
        }
        if (hikBTs > 0) {
            qInfo(appLog).noquote()
                << QStringLiteral("[LatencyTest] Hik B 相对触发 deltaMs=%1").arg(hikBTs - triggerMs);
        }
    }

    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] 日志中搜索 [ScanSync] 可对比 trigger / mech / hik_a / hik_b 的 epoch ms");
    qInfo(appLog).noquote() << QStringLiteral("[LatencyTest] ================================");

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const QString configuredRoot = configManager != nullptr
        ? configManager->flowControlConfig().scanCacheDirectory
        : QString();
    const QString cacheRoot = scan_tracking::common::resolveCaptureCacheRoot(configuredRoot);
    const QString timestamp = scan_tracking::common::buildCaptureTimestamp();
    constexpr int kTestSegmentIndex = 1;

    QString mechPngPath;
    QString hikAPath;
    QString hikBPath;

    if (bundle.mechEyeResult.texture2D.isValid()) {
        mechPngPath = scan_tracking::mech_eye::buildSegmentMech2DPngPath(
            configuredRoot, kTestSegmentIndex, taskId, timestamp);
        scan_tracking::mech_eye::saveGrayTextureFrameToPng(
            bundle.mechEyeResult.texture2D, mechPngPath);
    }

    if (bundle.hikCameraAResult.success() && bundle.hikCameraAResult.frame.isValid()) {
        hikAPath = scan_tracking::vision::buildSegmentHikMonoPath(
            configuredRoot, kTestSegmentIndex, taskId, QStringLiteral("hikA"), timestamp);
        scan_tracking::vision::saveHikMonoFrameToBmp(
            bundle.hikCameraAResult.frame, hikAPath);
    }

    if (bundle.hikCameraBResult.success() && bundle.hikCameraBResult.frame.isValid()) {
        hikBPath = scan_tracking::vision::buildSegmentHikMonoPath(
            configuredRoot, kTestSegmentIndex, taskId, QStringLiteral("hikB"), timestamp);
        scan_tracking::vision::saveHikMonoFrameToBmp(
            bundle.hikCameraBResult.frame, hikBPath);
    }

    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] 落盘目录 cacheRoot=%1 mech2d=%2 hikA=%3 hikB=%4")
               .arg(cacheRoot, mechPngPath, hikAPath, hikBPath);
}

void ConsoleRuntime::printStartupStatus()
{
    qInfo(appLog).noquote()
        << "正在启动"
        << QString::fromStdString(scan_tracking::common::ApplicationInfo::name())
        << "版本"
        << QString::fromStdString(scan_tracking::common::ApplicationInfo::version());
    qInfo(appLog).noquote() << "事件循环已激活。按 Ctrl+C 退出。";
}

void ConsoleRuntime::printShutdownStatus()
{
    stopAutoLatencyBundleTest();

    // 关闭顺序按依赖逆序执行，避免退出过程中还有异步请求落到已析构对象上。
    // HmiTcpServer 必须最先停止：它持有所有其他服务的裸指针，
    // 必须在那些服务析构之前切断 TCP 连接和定时器，防止悬垂指针访问。
    if (hmiTcpServer_) {
        hmiTcpServer_->stop();
        hmiTcpServer_.reset();
    }
    if (stateMachine_) {
        stateMachine_->stop();
    }
    if (visionPipelineService_) {
        visionPipelineService_->stop();
    }
    if (hikCameraCController_) {
        hikCameraCController_->stop();
    }
    if (hikCameraAService_) {
        hikCameraAService_->stop();
    }
    if (hikCameraBService_) {
        hikCameraBService_->stop();
    }
    if (hikCameraCService_) {
        hikCameraCService_->stop();
    }
    if (mechEyeService_) {
        mechEyeService_->stop();
    }
    if (modbusService_) {
        modbusService_->disconnectDevice();
    }
    qInfo(appLog).noquote() << "正在停止扫描跟踪核心框架。";
}

}  // namespace scan_tracking::app
