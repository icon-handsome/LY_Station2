#include "scan_tracking/flow_control/handlers/unload_calc_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* UnloadCalcHandler::triggerName() const { return "Trig_UnloadCalc"; }
int UnloadCalcHandler::trigOffset() const { return 25; }
void UnloadCalcHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeUnloadCalcTask(); }

/**
 * @brief 执行卸载计算任务（Trig_UnloadCalc）
 * 
 * 向 PLC 写入模拟的卸料位姿数据，然后立即完成任务。
 * 这是一个占位实现，实际应用中可能需要进行路径规划或碰撞检测。
 */
void StateMachine::executeUnloadCalcTask()
{
    const auto poseSource = resolveUnloadCalcPoseSource();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("UnloadCalc 位姿来源：")
        << poseSource.sourceName
        << QStringLiteral(" 可用=") << poseSource.available
        << QStringLiteral(" 成功=") << poseSource.success
        << QStringLiteral(" 消息=") << poseSource.message;
    writeUnloadCalcResult();  // 写入卸料位姿结果到 PLC 寄存器
    const quint16 resultCode = poseSource.success ? 1 : 7;
    completeActiveTask(resultCode, poseSource.success ? protocol::AckState::Completed
                                                       : protocol::AckState::Failed,
                       poseSource.success);
    emit unloadCalcFinished(resultCode, poseSource.x, poseSource.y, poseSource.z,
                            poseSource.rx, poseSource.ry, poseSource.rz);
}


}  // namespace scan_tracking::flow_control
