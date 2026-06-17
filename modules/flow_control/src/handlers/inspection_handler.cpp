#include "scan_tracking/flow_control/handlers/inspection_handler.h"

namespace scan_tracking::flow_control {

const char* InspectionHandler::triggerName() const { return "Trig_Inspection"; }
int InspectionHandler::trigOffset() const { return 24; }

void InspectionHandler::execute(TaskHandlerContext& ctx)
{
    const InspectionResult result = ctx.host.evaluateInspectionForActiveTask();
    ctx.host.finishInspection(result);
}

}  // namespace scan_tracking::flow_control
