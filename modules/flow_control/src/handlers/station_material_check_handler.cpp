#include "scan_tracking/flow_control/handlers/station_material_check_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"

namespace scan_tracking::flow_control {

const char* StationMaterialCheckHandler::triggerName() const { return "Trig_StationMaterialCheck"; }
int StationMaterialCheckHandler::trigOffset() const { return 21; }

void StationMaterialCheckHandler::execute(TaskHandlerContext& ctx)
{
    const bool hasModbus = ctx.host.isModbusConnected();
    auto* orbbec = ctx.host.orbbecGeminiService();
    const bool orbbecReady = orbbec != nullptr
        && orbbec->state() == orbbec_gemini::OrbbecGeminiRuntimeState::Ready;

    if (!hasModbus || !orbbecReady) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("工位检材不可用：")
            << QStringLiteral(" modbus=") << hasModbus
            << QStringLiteral(" orbbecReady=") << orbbecReady;
        ctx.host.writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("NO"));
        ctx.host.completeActiveTask(5, protocol::AckState::Failed, false);
        return;
    }

    // TODO: 接入 Orbbec 2D/深度有料 AI 判定；当前仅校验设备就绪。
    ctx.host.writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("OK"));
    ctx.host.completeActiveTask(1, protocol::AckState::Completed, true);
}

}  // namespace scan_tracking::flow_control
