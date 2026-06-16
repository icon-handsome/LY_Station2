#include "scan_tracking/flow_control/handlers/scan_segment_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking {
namespace flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* ScanSegmentHandler::triggerName() const { return "Trig_ScanSegment"; }
int ScanSegmentHandler::trigOffset() const { return 23; }
void ScanSegmentHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeScanSegmentTask(); }

void StateMachine::executeScanSegmentTask()
{
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ScanSegment：第二工位流程未实现，Res=8");
    completeActiveTask(8, protocol::AckState::Failed, false);
}

}  // namespace flow_control
}  // namespace scan_tracking
