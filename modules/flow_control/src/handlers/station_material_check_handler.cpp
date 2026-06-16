#include "scan_tracking/flow_control/handlers/station_material_check_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* StationMaterialCheckHandler::triggerName() const { return "Trig_StationMaterialCheck"; }
int StationMaterialCheckHandler::trigOffset() const { return 21; }
void StationMaterialCheckHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeStationMaterialCheckTask(); }

void StateMachine::executeStationMaterialCheckTask()
{
    const bool hasModbus = m_modbus != nullptr && m_modbus->isConnected();
    const bool hasMechEye = m_mechEye != nullptr;

    if (!hasModbus || !hasMechEye) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("工位检材不可用：")
            << QStringLiteral(" modbus=") << hasModbus
            << QStringLiteral(" mechEye=") << hasMechEye;
        writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("NO"));
        completeActiveTask(5, protocol::AckState::Failed, false);
        return;
    }

    writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("OK"));
    completeActiveTask(1, protocol::AckState::Completed, true);
}


}  // namespace scan_tracking::flow_control
