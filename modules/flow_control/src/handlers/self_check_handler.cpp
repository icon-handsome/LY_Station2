#include "scan_tracking/flow_control/handlers/self_check_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

const char* SelfCheckHandler::triggerName() const { return "Trig_SelfCheck"; }
int SelfCheckHandler::trigOffset() const { return 26; }

void SelfCheckHandler::execute(TaskHandlerContext& ctx)
{
    auto* orbbec = ctx.host.orbbecGeminiService();
    auto* vision = ctx.host.visionPipelineService();

    const bool modbusReady = ctx.host.isModbusConnected();
    const bool orbbecReady = orbbec != nullptr
        && orbbec->state() != orbbec_gemini::OrbbecGeminiRuntimeState::Failed;
    const bool visionReady = vision != nullptr && vision->isStarted();

    QVector<quint16> failWords = {
        static_cast<quint16>(modbusReady ? 0 : (1u << 1)),
        static_cast<quint16>(orbbecReady ? 0 : (1u << 0)),
    };
    if (!modbusReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：Modbus 不可用。");
    }
    if (!orbbecReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：Orbbec 不可用。");
    }
    if (!visionReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：视觉流水线不可用。");
    }

    const quint16 resultCode = (modbusReady && orbbecReady && visionReady) ? 1 : 0;
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
