#include "scan_tracking/flow_control/handlers/pose_check_handler.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"
#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking::flow_control {

const char* PoseCheckHandler::triggerName() const { return "Trig_PoseCheck"; }
int PoseCheckHandler::trigOffset() const { return 22; }

void PoseCheckHandler::execute(TaskHandlerContext& ctx)
{
    const QVector<double> identityRt = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    qInfo(LOG_FLOW).noquote() << QStringLiteral("Trig_PoseCheck：占位实现（LB 算法已移除）。");
    ctx.host.writeFloatPlaceholder(protocol::registers::kPoseDeviationMm, 0.0f);
    ctx.host.completeActiveTask(1, protocol::AckState::Completed, true);
    ctx.host.notifyPoseCheckFinished(
        true, 1, 0.0, identityRt, QStringLiteral("位姿检查占位 OK"));
}

}  // namespace scan_tracking::flow_control
