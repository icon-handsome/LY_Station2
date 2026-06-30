#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"
#include "scan_tracking/livox_mid360/livox_mid360_service.h"
#include "scan_tracking/tfmini_plus/tfmini_plus_service.h"
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
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>
#include <QtCore/QPointer>
#include <QtCore/QTimer>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/vision_types.h"
#include "scan_tracking/hmi_server/hmi_tcp_server.h"

Q_LOGGING_CATEGORY(appLog, "app")

namespace scan_tracking::app {

namespace {

#ifdef _WIN32
// Windows 控制台 Ctrl+C：在独立线程中仅排队 QCoreApplication::quit()，禁止在此调用 Qt 对象或 lambda。
BOOL WINAPI handleConsoleSignal(DWORD signal)
{
    switch (signal) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT: {
        if (QCoreApplication* app = QCoreApplication::instance()) {
            QMetaObject::invokeMethod(app, "quit", Qt::QueuedConnection);
        }
        return TRUE;
    }
    default:
        return FALSE;
    }
}
#endif

}  // namespace

ConsoleRuntime::ConsoleRuntime(QCoreApplication& application)
    : QObject(&application)
    , application_(application)
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
        this,
        &ConsoleRuntime::handleAboutToQuit);

    printStartupStatus();
    initModules();

    const int exitCode = application_.exec();

#ifdef _WIN32
    SetConsoleCtrlHandler(handleConsoleSignal, FALSE);
#endif

    return exitCode;
}

void ConsoleRuntime::handleAboutToQuit()
{
    printShutdownStatus();
}

void ConsoleRuntime::runDeferredStartupTasks()
{
    if (shuttingDown_) {
        return;
    }
    if (hmiTcpServer_) {
        hmiTcpServer_->bindServiceSignals();
        const auto port =
            scan_tracking::common::ConfigManager::instance()->hmiConfig().tcpPort;
        if (!hmiTcpServer_->start()) {
            qWarning(appLog) << "HMI TCP 服务器在端口" << port << "启动失败。";
        } else {
            qInfo(appLog) << "HMI TCP 服务器已在端口" << port << "启动。";
        }
    }
    if (orbbecGeminiService_) {
        orbbecGeminiService_->start();
        qInfo(appLog) << QStringLiteral("[OrbbecGemini] service started.");
    }
    if (livoxMid360Service_) {
        livoxMid360Service_->start();
        qInfo(appLog) << QStringLiteral("[LivoxMid360] service started.");
    }
    if (tfminiPlusService_) {
        tfminiPlusService_->start();
        qInfo(appLog) << QStringLiteral("[TfminiPlus] service started.");
    }
}

void ConsoleRuntime::onOrbbecOpenFinished(
    bool success,
    scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary,
    const QString& errorMessage)
{
    if (shuttingDown_) {
        return;
    }
    if (!success && !errorMessage.isEmpty()) {
        qWarning(appLog).noquote()
            << QStringLiteral("[OrbbecGemini] Open failed:")
            << errorMessage;
        return;
    }
    if (!success || !orbbecCaptureOnStart_ || !orbbecSaveCaptureToDisk_) {
        return;
    }
    if (orbbecGeminiService_ == nullptr) {
        return;
    }
    const quint64 requestId =
        orbbecGeminiService_->requestCapture(orbbecCaptureTimeoutMs_, true);
    if (requestId == 0) {
        qWarning(appLog).noquote()
            << QStringLiteral("[OrbbecGemini] Startup capture request rejected");
    } else {
        qInfo(appLog).noquote()
            << QStringLiteral("[OrbbecGemini] Startup capture requested req=")
            << requestId;
    }
}

void ConsoleRuntime::initModules()
{
    qInfo(appLog) << "正在初始化模块...";

    // SCAN_TRACKING_STARTUP_STAGE 用于分步联调：0=仅 Modbus，5=全量模块
    int startupStage = 5;
    const QByteArray startupStageEnv = qgetenv("SCAN_TRACKING_STARTUP_STAGE");
	bool startupStageOk = false;
    if (!startupStageEnv.isEmpty()) {
        startupStage = startupStageEnv.toInt(&startupStageOk);
        if (!startupStageOk) {
            startupStage = 5;
        }
    }
    qInfo(appLog) << QStringLiteral("启动阶段 =") << startupStage
                  << QStringLiteral(" (0=Modbus, 1=+MechEye, 2=+Hik, 3=+VisionPipeline, 4=+Tracking, 5=+StateMachine)");

    modbusService_ = std::make_unique<scan_tracking::modbus::ModbusService>();
    qInfo(appLog) << "Modbus 服务已创建。";

    // stage 0：仅 Modbus，用于 PLC 通信单独验证
    if (startupStage < 1) {
        qInfo(appLog) << "启动阶段仅到 Modbus。";
        if (!modbusService_->connectDevice()) {
            qWarning(appLog) << "Modbus 连接初始化失败。";
        }
        qInfo(appLog) << "所有模块已初始化。";
        return;
    }
    // MechEye 服务先启动，保证后续状态机和视觉集成层都能复用同一份点云采集入口。
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr) {
        const auto& profile = configManager->stationProfile();
        qInfo(appLog).noquote()
            << QStringLiteral("[Station] stationId=") << scan_tracking::common::stationIdToInt(profile.stationId)
            << QStringLiteral(" name=") << profile.stationName
            << QStringLiteral(" scanPaths=") << (profile.scanPathsConfigPath.isEmpty()
                                                    ? QStringLiteral("<fallback scan_paths_config.json>")
                                                    : profile.scanPathsConfigPath)
            << QStringLiteral(" workMode=") << scan_tracking::common::workModeIdToString(profile.defaultWorkMode);
        qInfo(appLog).noquote()
            << QStringLiteral("[Station] enableLoadGrasp=") << profile.enableLoadGrasp
            << QStringLiteral(" enableUnloadCalc=") << profile.enableUnloadCalc
            << QStringLiteral(" enablePoseCheck=") << profile.enablePoseCheck
            << QStringLiteral(" enableTelescopicScan=") << profile.enableTelescopicScan
            << QStringLiteral(" enableHoistAssist=") << profile.enableHoistAssist
            << QStringLiteral(" enableCollisionMonitor=") << profile.enableCollisionMonitor
            << QStringLiteral(" (reserved, not enforced in stage1)");
    }

    const auto visionConfigForMech = configManager != nullptr
                                         ? configManager->visionConfig()
                                         : scan_tracking::common::VisionConfig{};
    const QString telescopicMechKey = visionConfigForMech.telescopicGroup.mechEye.cameraKey;
    const QString armMechKey = visionConfigForMech.armGroup.mechEye.cameraKey;

    mechEyeTelescopicService_ = std::make_unique<scan_tracking::mech_eye::MechEyeService>();
    mechEyeArmService_ = std::make_unique<scan_tracking::mech_eye::MechEyeService>();

    const auto connectMechEyeLogs = [this](scan_tracking::mech_eye::MechEyeService* service,
                                       const char* label) {
        if (service == nullptr) {
            return;
        }
        QObject::connect(
            service,
            &scan_tracking::mech_eye::MechEyeService::stateChanged,
            this,
            [label](scan_tracking::mech_eye::CameraRuntimeState state, const QString& description) {
                qInfo(appLog) << label << QStringLiteral("状态 =") << static_cast<int>(state) << description;
            });
        QObject::connect(
            service,
            &scan_tracking::mech_eye::MechEyeService::fatalError,
            this,
            [label](scan_tracking::mech_eye::CaptureErrorCode code, const QString& message) {
                qCritical(appLog) << label << QStringLiteral("致命错误：") << static_cast<int>(code) << message;
            });
    };
    connectMechEyeLogs(mechEyeTelescopicService_.get(), "[梅卡-伸缩杆]");
    connectMechEyeLogs(mechEyeArmService_.get(), "[梅卡-机械臂]");

    mechEyeTelescopicService_->start(telescopicMechKey);
    mechEyeArmService_->start(armMechKey);
    qInfo(appLog).noquote()
        << QStringLiteral("梅卡相机服务已启动：伸缩杆=") << telescopicMechKey
        << QStringLiteral(" 机械臂=") << armMechKey;

    if (configManager != nullptr) {
        const auto& orbbecConfig = configManager->orbbecGeminiConfig();
        if (!orbbecConfig.enabled) {
            qInfo(appLog) << QStringLiteral("[OrbbecGemini] disabled (orbbecGeminiEnabled=false)");
        } else {
            orbbecGeminiService_ = std::make_unique<scan_tracking::orbbec_gemini::OrbbecGeminiService>();
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::logMessage,
                this,
                [](const QString& message) {
                    qInfo(appLog).noquote() << message;
                });
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::stateChanged,
                this,
                [](scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState state,
                   const QString& description) {
                    qInfo(appLog).noquote()
                        << QStringLiteral("[OrbbecGemini] state=")
                        << static_cast<int>(state)
                        << description;
                });
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::captureFinished,
                this,
                [](const scan_tracking::orbbec_gemini::OrbbecCaptureResult& result) {
                    if (result.errorCode
                        != scan_tracking::orbbec_gemini::OrbbecCaptureErrorCode::Success) {
                        qWarning(appLog).noquote()
                            << QStringLiteral("[OrbbecGemini] Capture failed:")
                            << result.errorMessage;
                        return;
                    }
                    qInfo(appLog).noquote()
                        << QStringLiteral("[OrbbecGemini] Capture saved req=") << result.requestId
                        << QStringLiteral(" depthRaw=") << result.depthRawPngPath
                        << QStringLiteral(" depthPreview=") << result.depthPreviewPngPath
                        << QStringLiteral(" pointCloud=") << result.pointCloudPlyPath;
                });
            orbbecCaptureOnStart_ = orbbecConfig.captureOnStart;
            orbbecSaveCaptureToDisk_ = orbbecConfig.saveCaptureToDisk;
            orbbecCaptureTimeoutMs_ = orbbecConfig.captureTimeoutMs;
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::openFinished,
                this,
                &ConsoleRuntime::onOrbbecOpenFinished);
            // 延后到状态机/HMI 初始化完成后再打开设备，避免 SDK 后台线程与启动期竞态。
            qInfo(appLog) << QStringLiteral("[OrbbecGemini] service created (deferred start).");
        }

        const auto& livoxConfig = configManager->livoxMid360Config();
        if (!livoxConfig.enabled) {
            qInfo(appLog) << QStringLiteral("[LivoxMid360] disabled (livoxMid360Enabled=false)");
        } else {
            livoxMid360Service_ = std::make_unique<scan_tracking::livox_mid360::LivoxMid360Service>();
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::logMessage,
                [](const QString& message) {
                    qInfo(appLog).noquote() << message;
                });
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::stateChanged,
                [](scan_tracking::livox_mid360::LivoxMid360RuntimeState state,
                   const QString& description) {
                    qInfo(appLog).noquote()
                        << QStringLiteral("[LivoxMid360] state=")
                        << static_cast<int>(state)
                        << description;
                });
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::openFinished,
                [](bool success,
                   scan_tracking::livox_mid360::LivoxMid360DeviceSummary,
                   const QString& errorMessage) {
                    if (!success && !errorMessage.isEmpty()) {
                        qWarning(appLog).noquote()
                            << QStringLiteral("[LivoxMid360] Open failed:")
                            << errorMessage;
                    }
                });
            qInfo(appLog) << QStringLiteral("[LivoxMid360] service created (deferred start).");
        }

        const auto& tfminiConfig = configManager->tfminiPlusConfig();
        if (!tfminiConfig.enabled) {
            qInfo(appLog) << QStringLiteral("[TfminiPlus] disabled (tfminiPlusEnabled=false)");
        } else {
            // 第二工位吊装/内壁防碰辅助测距；Worker 按协议解析后丢弃，不写 PLC 安全位。
            tfminiPlusService_ = std::make_unique<scan_tracking::tfmini_plus::TfminiPlusService>();
            QObject::connect(
                tfminiPlusService_.get(),
                &scan_tracking::tfmini_plus::TfminiPlusService::logMessage,
                [](const QString& message) {
                    qInfo(appLog).noquote() << message;
                });
            QObject::connect(
                tfminiPlusService_.get(),
                &scan_tracking::tfmini_plus::TfminiPlusService::stateChanged,
                [](scan_tracking::tfmini_plus::TfminiPlusRuntimeState state,
                   const QString& description) {
                    qInfo(appLog).noquote()
                        << QStringLiteral("[TfminiPlus] state=")
                        << static_cast<int>(state)
                        << description;
                });
            QObject::connect(
                tfminiPlusService_.get(),
                &scan_tracking::tfmini_plus::TfminiPlusService::openFinished,
                [](bool success, const QString& errorMessage) {
                    if (!success && !errorMessage.isEmpty()) {
                        qWarning(appLog).noquote()
                            << QStringLiteral("[TfminiPlus] Open failed:")
                            << errorMessage;
                    }
                });
            // TODO: Worker 恢复 emit distanceUpdated 后，在此接入打印/告警/碰撞阈值过滤。
            qInfo(appLog) << QStringLiteral("[TfminiPlus] service created (deferred start).");
        }
    }

    const auto visionConfig = configManager != nullptr
        ? configManager->visionConfig()
        : scan_tracking::common::VisionConfig{};

    if (!visionConfig.hikCxpEnabled) {
        qInfo(appLog) << QStringLiteral("CXP 双目已跳过（hikCxpEnabled=false），组合采集使用梅卡 + 海康智能 C。");
    } else {
        // CXP 左/右目各一个 HikCxpCameraService 实例，roleName 区分 ch250_a / ch250_b
        hikCxpCameraAService_ = std::make_unique<scan_tracking::vision::HikCxpCameraService>(
            QStringLiteral("ch250_a"));
        hikCxpCameraBService_ = std::make_unique<scan_tracking::vision::HikCxpCameraService>(
            QStringLiteral("ch250_b"));

        QObject::connect(
            hikCxpCameraAService_.get(),
            &scan_tracking::vision::HikCxpCameraService::stateChanged,
            [](const QString& roleName, const QString& stateText, const QString& description) {
                qInfo(appLog) << QStringLiteral("[CXP]") << roleName << stateText << description;
            });
        QObject::connect(
            hikCxpCameraBService_.get(),
            &scan_tracking::vision::HikCxpCameraService::stateChanged,
            [](const QString& roleName, const QString& stateText, const QString& description) {
                qInfo(appLog) << QStringLiteral("[CXP]") << roleName << stateText << description;
            });

        hikCxpCameraAService_->start(
            visionConfig.hikCxpCameraA,
            visionConfig.hikCxpCaptureTimeoutMs,
            visionConfig.hikCxpExposureTimeUs,
            visionConfig.hikCxpGain);
        hikCxpCameraBService_->start(
            visionConfig.hikCxpCameraB,
            visionConfig.hikCxpCaptureTimeoutMs,
            visionConfig.hikCxpExposureTimeUs,
            visionConfig.hikCxpGain);
        qInfo(appLog) << QStringLiteral("CXP 双目相机服务已启动。");
    }

    // 海康相机 C（智能相机）：纯 TCP 控制，不通过 MVS SDK 打开设备，避免与 SCMVS 冲突
    hikCameraCController_ = std::make_unique<scan_tracking::vision::HikCameraCController>();

    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::stateChanged,
        this,
        [](scan_tracking::vision::HikCameraCState state, QString description) {
            qInfo(appLog) << QStringLiteral("[海康C控制器] 状态 =") << static_cast<int>(state) << description;
        });
    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::fatalError,
        this,
        [](scan_tracking::vision::VisionErrorCode code, const QString& message) {
            qCritical(appLog) << QStringLiteral("[海康C控制器] 致命错误：")
                              << static_cast<int>(code) << message;
        });
    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::captureCompleted,
        this,
        [](scan_tracking::vision::CaptureType type,
           const QString& cameraIp,
           const QByteArray& imageData) {
            QString typeStr;
            switch (type) {
                case scan_tracking::vision::CaptureType::SurfaceDefect:
                    typeStr = QStringLiteral("表面缺陷");
                    break;
                case scan_tracking::vision::CaptureType::WeldDefect:
                    typeStr = QStringLiteral("焊缝缺陷");
                    break;
                case scan_tracking::vision::CaptureType::NumberRecognition:
                    typeStr = QStringLiteral("编号识别");
                    break;
                default:
                    typeStr = QStringLiteral("未知");
                    break;
            }
            qInfo(appLog).noquote()
                << QStringLiteral("[海康C控制器] 采集完成：") << cameraIp << typeStr
                << imageData.size() << QStringLiteral("字节");
        });

    hikCameraCController_->start(visionConfig);
    qInfo(appLog) << QStringLiteral("海康 C 相机控制器已启动（TCP 通信模式）。");


    // 统一视觉编排层负责把“1 份点云 + 2 份矩阵”收口为一个算法输入包。
    visionPipelineService_ = std::make_unique<scan_tracking::vision::VisionPipelineService>(
        mechEyeTelescopicService_.get(),
        mechEyeArmService_.get(),
        hikCxpCameraAService_.get(),
        hikCxpCameraBService_.get(),
        hikCameraCController_.get());

    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::stateChanged,
        this,
        [](scan_tracking::vision::VisionPipelineState state, const QString& description) {
            qInfo(appLog) << QStringLiteral("[视觉流水线] 状态 =") << static_cast<int>(state) << description;
        });
    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::bundleCaptureFinished,
        this,
        [](const scan_tracking::vision::MultiCameraCaptureBundle& bundle) {
            qInfo(appLog) << QStringLiteral("[视觉流水线]") << bundle.summary();
        });

    visionPipelineService_->start(visionConfig);
    qInfo(appLog) << QStringLiteral("视觉集成框架已启动。");

    // StateMachine 是主流程编排核心，注入 Modbus / 视觉等依赖
    stateMachine_ = std::make_unique<scan_tracking::flow_control::StateMachine>(
        modbusService_.get(),
        mechEyeTelescopicService_.get(),
        mechEyeArmService_.get(),
        visionPipelineService_.get(),
        hikCameraCController_.get());

    // HMI：先注入依赖；bind/listen 延后到事件循环，避免与 MechEye/视觉 worker 启动期竞态崩溃。
    const auto& hmiConfig = scan_tracking::common::ConfigManager::instance()->hmiConfig();
    if (hmiConfig.enabled) {
        hmiTcpServer_ = std::make_unique<scan_tracking::hmi_server::HmiTcpServer>(
            static_cast<int>(hmiConfig.tcpPort));
        hmiTcpServer_->setStateMachine(stateMachine_.get());
        hmiTcpServer_->setModbusService(modbusService_.get());
        hmiTcpServer_->setMechEyeServices(
            mechEyeTelescopicService_.get(), mechEyeArmService_.get());
        hmiTcpServer_->setVisionPipelineService(visionPipelineService_.get());
        hmiTcpServer_->setHikCameraServices(
            hikCxpCameraAService_.get(), hikCxpCameraBService_.get(), nullptr);
        hmiTcpServer_->setHikCameraCController(hikCameraCController_.get());
    } else {
        qInfo(appLog) << "HMI TCP 服务已在 config.ini [Hmi] enabled=false 下禁用。";
    }

    stateMachine_->start();

    // 等状态机接好信号后再建 Modbus 链路，避免启动期漏掉 connected 事件。
    if (!modbusService_->connectDevice()) {
        qWarning(appLog) << "Modbus 连接初始化失败。";
    }
    qInfo(appLog) << "所有模块已初始化。";

    QTimer::singleShot(0, this, &ConsoleRuntime::runDeferredStartupTasks);
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
    if (shuttingDown_) {
        return;
    }
    shuttingDown_ = true;

    // 断开本对象上的所有信号连接，避免模块 stop/析构期间再触发带捕获的槽。
    disconnect(this, nullptr, nullptr, nullptr);

    // 关闭顺序按依赖逆序执行，避免退出过程中还有异步请求落到已析构对象上。
    // HmiTcpServer 必须最先停止：它持有所有其他服务的裸指针。
    if (hmiTcpServer_) {
        hmiTcpServer_->stop();
        hmiTcpServer_.reset();
    }

    if (visionPipelineService_) {
        visionPipelineService_->stop();
    }

    if (modbusService_ && modbusService_->isConnected()) {
        modbusService_->resetIpcResultBlock();
    }

    if (stateMachine_) {
        stateMachine_->stop();
        stateMachine_.reset();
    }

    if (hikCameraCController_) {
        hikCameraCController_->stop();
        hikCameraCController_.reset();
    }
    if (visionPipelineService_) {
        visionPipelineService_.reset();
    }
    if (hikCxpCameraAService_) {
        hikCxpCameraAService_->stop();
        hikCxpCameraAService_.reset();
    }
    if (hikCxpCameraBService_) {
        hikCxpCameraBService_->stop();
        hikCxpCameraBService_.reset();
    }
    if (orbbecGeminiService_) {
        orbbecGeminiService_->stop();
        orbbecGeminiService_.reset();
    }
    if (livoxMid360Service_) {
        livoxMid360Service_->stop();
        livoxMid360Service_.reset();
    }
    if (tfminiPlusService_) {
        tfminiPlusService_->stop();
        tfminiPlusService_.reset();
    }
    if (mechEyeTelescopicService_) {
        mechEyeTelescopicService_->stop();
        mechEyeTelescopicService_.reset();
    }
    if (mechEyeArmService_) {
        mechEyeArmService_->stop();
        mechEyeArmService_.reset();
    }
    if (modbusService_) {
        modbusService_->disconnectDevice();
        modbusService_.reset();
    }

    qInfo(appLog).noquote() << "正在停止扫描跟踪核心框架。";
}

}  // namespace scan_tracking::app
