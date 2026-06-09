#include "scan_tracking/flow_control/handlers/code_read_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* CodeReadHandler::triggerName() const { return "Trig_CodeRead"; }
int CodeReadHandler::trigOffset() const { return 27; }
void CodeReadHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeCodeReadTask(); }

void StateMachine::executeCodeReadTask()
{
    qInfo(LOG_FLOW).noquote() << QStringLiteral("收到 Trig_CodeRead，当前为占位实现。");
    if (m_modbus && m_modbus->isConnected()) {
        writeAsciiPlaceholder(protocol::registers::kCodeValueAscii, protocol::registers::kCodeValueRegisterCount, QStringLiteral("RD"));
    }
    completeActiveTask(9, protocol::AckState::Failed, false);
    emit codeReadFinished(9, QStringLiteral("RD"));
}


}  // namespace scan_tracking::flow_control
