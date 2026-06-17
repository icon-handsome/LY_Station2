#include "scan_tracking/flow_control/handlers/result_reset_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

namespace scan_tracking::flow_control {

const char* ResultResetHandler::triggerName() const { return "Trig_ResultReset"; }
int ResultResetHandler::trigOffset() const { return 28; }

void ResultResetHandler::execute(TaskHandlerContext& ctx)
{
    ctx.host.resetSafetyInterlockState();
    ctx.host.resetScanSegmentCache();

    if (ctx.host.isModbusConnected()) {
        ctx.host.clearScanSegmentDoneRegisters();
        ctx.host.clearInspectionResultRegisters();
        ctx.host.clearIpcSafetyActionWord();
    }

    ctx.host.completeActiveTask(1);
    ctx.host.notifyResultResetFinished(1);
}

}  // namespace scan_tracking::flow_control
