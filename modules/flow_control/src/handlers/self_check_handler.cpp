#include "scan_tracking/flow_control/handlers/self_check_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

const char* SelfCheckHandler::triggerName() const { return "Trig_SelfCheck"; }
int SelfCheckHandler::trigOffset() const { return 26; }

void SelfCheckHandler::execute(TaskHandlerContext& ctx)
{
    auto* mechEye = ctx.host.mechEyeService();
    auto* vision = ctx.host.visionPipelineService();

    const bool modbusReady = ctx.host.isModbusConnected();
    const bool mechEyeReady = mechEye != nullptr && mechEye->state() != mech_eye::CameraRuntimeState::Error;
    const bool visionReady = vision != nullptr && vision->isStarted();

    QVector<quint16> failWords = {
        static_cast<quint16>(modbusReady ? 0 : (1u << 1)),
        static_cast<quint16>(mechEyeReady ? 0 : (1u << 0)),
    };
    if (!modbusReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：Modbus 不可用。");
    }
    if (!mechEyeReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：MechEye 不可用。");
    }
    if (!visionReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：视觉流水线不可用。");
    }

    const quint16 resultCode = (modbusReady && mechEyeReady && visionReady) ? 1 : 0;
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
