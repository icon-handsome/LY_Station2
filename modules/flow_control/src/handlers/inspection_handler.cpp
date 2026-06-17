#include "scan_tracking/flow_control/handlers/inspection_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

namespace scan_tracking {
namespace flow_control {

const char* InspectionHandler::triggerName() const { return "Trig_Inspection"; }
int InspectionHandler::trigOffset() const { return 24; }
void InspectionHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeInspectionTask(); }

}  // namespace flow_control
}  // namespace scan_tracking
