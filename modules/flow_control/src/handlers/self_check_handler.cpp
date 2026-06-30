#include "scan_tracking/flow_control/handlers/self_check_handler.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

const char* SelfCheckHandler::triggerName() const { return "Trig_SelfCheck"; }
int SelfCheckHandler::trigOffset() const { return 26; }

void SelfCheckHandler::execute(TaskHandlerContext& ctx)
{
    auto* mechTelescopic = ctx.host.mechEyeTelescopicService();
    auto* mechArm = ctx.host.mechEyeArmService();
    auto* vision = ctx.host.visionPipelineService();
    auto* hikController = ctx.host.hikCameraCController();

    const bool modbusReady = ctx.host.isModbusConnected();
    const auto mechReady = [](const mech_eye::MechEyeService* service) {
        return service != nullptr && service->state() != mech_eye::CameraRuntimeState::Error;
    };
    const bool mechTelescopicReady = mechReady(mechTelescopic);
    const bool mechArmReady = mechReady(mechArm);
    const bool visionReady = vision != nullptr && vision->isStarted();

    const auto* configMgr = common::ConfigManager::instance();
    const QString telescopicHikIp = configMgr != nullptr
                                        ? configMgr->visionConfig().telescopicGroup.hikCameraC.ipAddress.trimmed()
                                        : QString();
    const QString armHikIp = configMgr != nullptr
                                 ? configMgr->visionConfig().armGroup.hikCameraC.ipAddress.trimmed()
                                 : QString();
    const bool hikTelescopicReady =
        hikController != nullptr && !telescopicHikIp.isEmpty()
        && hikController->isCameraConnected(telescopicHikIp);
    const bool hikArmReady =
        hikController != nullptr && !armHikIp.isEmpty()
        && hikController->isCameraConnected(armHikIp);

    QVector<quint16> failWords = {
        static_cast<quint16>(modbusReady ? 0 : (1u << 1)),
        static_cast<quint16>(mechTelescopicReady ? 0 : (1u << 0)),
        static_cast<quint16>(mechArmReady ? 0 : (1u << 2)),
        static_cast<quint16>(hikTelescopicReady ? 0 : (1u << 3)),
        static_cast<quint16>(hikArmReady ? 0 : (1u << 4)),
    };
    if (!modbusReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：Modbus 不可用。");
    }
    if (!mechTelescopicReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：伸缩杆梅卡不可用。");
    }
    if (!mechArmReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：机械臂梅卡不可用。");
    }
    if (!visionReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：视觉流水线不可用。");
    }
    if (!hikTelescopicReady) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("自检：伸缩杆海康 C 未连接 TCP：") << telescopicHikIp;
    }
    if (!hikArmReady) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("自检：机械臂海康 C 未连接 TCP：") << armHikIp;
    }

    const bool allReady = modbusReady && mechTelescopicReady && mechArmReady && visionReady
                          && hikTelescopicReady && hikArmReady;
    const quint16 resultCode = allReady ? 1 : 0;
    if (modbusReady) {
        ctx.host.writeSelfCheckFailWords(failWords);
    }
    ctx.host.completeActiveTask(
        resultCode,
        resultCode == 1 ? protocol::AckState::Completed : protocol::AckState::Failed,
        resultCode == 1);
    ctx.host.notifySelfCheckFinished(resultCode, failWords.value(0));
}

}  // namespace scan_tracking::flow_control
