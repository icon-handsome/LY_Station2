#pragma once

// ConsoleRuntime 是控制台版扫描跟踪应用的生命周期管理器。

#include <QtCore/QCoreApplication>
#include <memory>

#include "scan_tracking/livox_mid360/livox_mid360_service.h"
#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"
#include "scan_tracking/tfmini_plus/tfmini_plus_service.h"

namespace scan_tracking {
namespace modbus { class ModbusService; }
namespace flow_control { class StateMachine; }
namespace vision {
class VisionPipelineService;
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

    QCoreApplication& application_;
    std::unique_ptr<scan_tracking::modbus::ModbusService> modbusService_;
    std::unique_ptr<scan_tracking::orbbec_gemini::OrbbecGeminiService> orbbecGeminiService_;
    std::unique_ptr<scan_tracking::livox_mid360::LivoxMid360Service> livoxMid360Service_;
    std::unique_ptr<scan_tracking::tfmini_plus::TfminiPlusService> tfminiPlusService_;
    std::unique_ptr<scan_tracking::vision::VisionPipelineService> visionPipelineService_;
    std::unique_ptr<scan_tracking::flow_control::StateMachine> stateMachine_;
    std::unique_ptr<scan_tracking::hmi_server::HmiTcpServer> hmiTcpServer_;
};

}
