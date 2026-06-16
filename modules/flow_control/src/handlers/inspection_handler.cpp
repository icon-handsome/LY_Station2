#include "scan_tracking/flow_control/handlers/inspection_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking {
namespace flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* InspectionHandler::triggerName() const { return "Trig_Inspection"; }
int InspectionHandler::trigOffset() const { return 24; }
void InspectionHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeInspectionTask(); }

void StateMachine::executeInspectionTask()
{
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection：第二工位流程未实现，Res=8");
    completeActiveTask(8, protocol::AckState::Failed, false);
}

}  // namespace flow_control
}  // namespace scan_tracking
