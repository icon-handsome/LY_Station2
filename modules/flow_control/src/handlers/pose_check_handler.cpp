#include "scan_tracking/flow_control/handlers/pose_check_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* PoseCheckHandler::triggerName() const { return "Trig_PoseCheck"; }
int PoseCheckHandler::trigOffset() const { return 22; }
void PoseCheckHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executePoseCheckTask(); }

void StateMachine::executePoseCheckTask()
{
    const QVector<double> identityRt = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    qInfo(LOG_FLOW).noquote() << QStringLiteral("Trig_PoseCheck：占位实现（LB 算法已移除）。");
    writeFloatPlaceholder(protocol::registers::kPoseDeviationMm, 0.0f);
    completeActiveTask(1, protocol::AckState::Completed, true);
    emit poseCheckFinished(true, 1, 0.0, identityRt, QStringLiteral("位姿检查占位 OK"));
}

}  // namespace scan_tracking::flow_control
