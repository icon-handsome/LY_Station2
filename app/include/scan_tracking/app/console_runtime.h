#pragma once

// ConsoleRuntime 是控制台版扫描跟踪应用的生命周期管理器。

#include <QtCore/QCoreApplication>

namespace scan_tracking {
namespace modbus { class ModbusService; }
namespace mech_eye { class MechEyeService; }
namespace flow_control { class StateMachine; }
namespace orbbec_gemini {
class OrbbecGeminiService;
struct OrbbecGeminiDeviceSummary;
}
namespace livox_mid360 { class LivoxMid360Service; }
namespace tfmini_plus { class TfminiPlusService; }
namespace vision {
class HikCxpCameraService;
class VisionPipelineService;
class HikCameraCController;
}
namespace hmi_server { class HmiTcpServer; }
}

namespace scan_tracking::app {

class ConsoleRuntime final : public QObject {
    Q_OBJECT

public:
    explicit ConsoleRuntime(QCoreApplication& application);
    ~ConsoleRuntime() override;

    int run();

private slots:
    void handleAboutToQuit();
    void runDeferredStartupTasks();

private:
    void printStartupStatus();
    void printShutdownStatus();
    void initModules();
    void onOrbbecOpenFinished(
        bool success,
        scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary summary,
        const QString& errorMessage);

    bool shuttingDown_ = false;
    QCoreApplication& application_;
    std::unique_ptr<scan_tracking::modbus::ModbusService> modbusService_;
    std::unique_ptr<scan_tracking::mech_eye::MechEyeService> mechEyeTelescopicService_;
    std::unique_ptr<scan_tracking::mech_eye::MechEyeService> mechEyeArmService_;
    std::unique_ptr<scan_tracking::orbbec_gemini::OrbbecGeminiService> orbbecGeminiService_;
    std::unique_ptr<scan_tracking::livox_mid360::LivoxMid360Service> livoxMid360Service_;
    std::unique_ptr<scan_tracking::tfmini_plus::TfminiPlusService> tfminiPlusService_;
    std::unique_ptr<scan_tracking::vision::HikCxpCameraService> hikCxpCameraAService_;
    std::unique_ptr<scan_tracking::vision::HikCxpCameraService> hikCxpCameraBService_;
    std::unique_ptr<scan_tracking::vision::VisionPipelineService> visionPipelineService_;
    std::unique_ptr<scan_tracking::vision::HikCameraCController> hikCameraCController_;
    std::unique_ptr<scan_tracking::flow_control::StateMachine> stateMachine_;
    std::unique_ptr<scan_tracking::hmi_server::HmiTcpServer> hmiTcpServer_;
    bool orbbecCaptureOnStart_ = false;
    bool orbbecSaveCaptureToDisk_ = false;
    int orbbecCaptureTimeoutMs_ = 0;
};

}  // namespace scan_tracking::app
