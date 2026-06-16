#include "scan_tracking/flow_control/handlers/self_check_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* SelfCheckHandler::triggerName() const { return "Trig_SelfCheck"; }
int SelfCheckHandler::trigOffset() const { return 26; }
void SelfCheckHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeSelfCheckTask(); }

void StateMachine::executeSelfCheckTask()
{
    const bool modbusReady = m_modbus != nullptr && m_modbus->isConnected();
    const bool mechEyeReady = m_mechEye != nullptr && m_mechEye->state() != mech_eye::CameraRuntimeState::Error;
    const bool visionReady = m_visionPipeline != nullptr && m_visionPipeline->isStarted();

    QVector<quint16> failWords = {
        static_cast<quint16>(modbusReady ? 0 : (1u << 1)),
        static_cast<quint16>(mechEyeReady ? 0 : (1u << 0)),
    };
    if (!modbusReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：Modbus 不可用。");
    }
    if (!mechEyeReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：MechEye 不可用。");
    }
    if (!visionReady) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("自检：视觉流水线不可用。");
    }

    const quint16 resultCode = (modbusReady && mechEyeReady && visionReady) ? 1 : 0;
    if (m_modbus && m_modbus->isConnected()) {
        m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord0, failWords);
        m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord1, {0});
    }
    completeActiveTask(resultCode, resultCode == 1 ? protocol::AckState::Completed : protocol::AckState::Failed, resultCode == 1);
    emit selfCheckFinished(resultCode, failWords.value(0));
}


}  // namespace scan_tracking::flow_control
