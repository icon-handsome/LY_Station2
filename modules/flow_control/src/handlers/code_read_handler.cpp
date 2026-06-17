#include "scan_tracking/flow_control/handlers/code_read_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking::flow_control {

const char* CodeReadHandler::triggerName() const { return "Trig_CodeRead"; }
int CodeReadHandler::trigOffset() const { return 27; }

void CodeReadHandler::execute(TaskHandlerContext& ctx)
{
    qInfo(LOG_FLOW).noquote() << QStringLiteral("收到 Trig_CodeRead，当前为占位实现。");

    if (ctx.host.isModbusConnected()) {
        ctx.host.writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            QStringLiteral("RD"));
    }

    ctx.host.completeActiveTask(9, protocol::AckState::Failed, false);
    ctx.host.notifyCodeReadFinished(9, QStringLiteral("RD"));
}

}  // namespace scan_tracking::flow_control
