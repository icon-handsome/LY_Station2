#pragma once

#include <QtCore/QCoreApplication>
#include <memory>

class QTimer;

#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace modbus { class ModbusService; }
namespace mech_eye { class MechEyeService; }
namespace tracking { class TrackingService; }
namespace flow_control { class StateMachine; }
namespace vision {
class HikCxpCameraService;
class VisionPipelineService;
class HikCameraCController;
}
namespace hmi_server { class HmiTcpServer; }
}
#include "scan_tracking/hmi_server/hmi_tcp_server.h"

namespace scan_tracking::app {

class ConsoleRuntime final {
public:
    explicit ConsoleRuntime(QCoreApplication& application);
    ~ConsoleRuntime();

    int run();

private:
    void printStartupStatus();
    void printShutdownStatus();
    void initModules();
    void scheduleAutoLatencyBundleTest();
    void triggerAutoLatencyBundleTest();
    void onAutoLatencyBundleFinished(const scan_tracking::vision::MultiCameraCaptureBundle& bundle);

    void stopAutoLatencyBundleTest();

    qint64 m_autoLatencyTriggerMs = 0;
    bool m_autoLatencyTestPending = false;
    bool m_autoLatencyPeriodic = false;
    int m_autoLatencyIntervalMs = 20000;
    quint32 m_latencyBundleSeq = 0;
    QTimer* m_autoLatencyTimer = nullptr;

    /// 指针成员声明
    QCoreApplication& application_;
    std::unique_ptr<scan_tracking::modbus::ModbusService> modbusService_;
    std::unique_ptr<scan_tracking::mech_eye::MechEyeService> mechEyeService_;
    std::unique_ptr<scan_tracking::vision::HikCxpCameraService> hikCxpCameraAService_;
    std::unique_ptr<scan_tracking::vision::HikCxpCameraService> hikCxpCameraBService_;
    std::unique_ptr<scan_tracking::vision::VisionPipelineService> visionPipelineService_;
    std::unique_ptr<scan_tracking::vision::HikCameraCController> hikCameraCController_;
    std::unique_ptr<scan_tracking::tracking::TrackingService> trackingService_;
    std::unique_ptr<scan_tracking::flow_control::StateMachine> stateMachine_;
    std::unique_ptr<scan_tracking::hmi_server::HmiTcpServer> hmiTcpServer_;
};

}
