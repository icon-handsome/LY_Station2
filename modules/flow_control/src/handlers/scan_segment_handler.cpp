#include "scan_tracking/flow_control/handlers/scan_segment_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

namespace scan_tracking {
namespace flow_control {

const char* ScanSegmentHandler::triggerName() const { return "Trig_ScanSegment"; }
int ScanSegmentHandler::trigOffset() const { return 23; }
void ScanSegmentHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeScanSegmentTask(); }

}  // namespace flow_control
}  // namespace scan_tracking
