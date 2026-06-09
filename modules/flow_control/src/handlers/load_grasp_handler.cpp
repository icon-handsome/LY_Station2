#include "scan_tracking/flow_control/handlers/load_grasp_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* LoadGraspHandler::triggerName() const { return "Trig_LoadGrasp"; }
int LoadGraspHandler::trigOffset() const { return 20; }
void LoadGraspHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeLoadGraspTask(); }

/**
 * @brief 执行加载抓取任务（Trig_LoadGrasp）
 * 
 * 向 PLC 写入模拟的加载位姿数据，然后立即完成任务。
 * 这是一个占位实现，实际应用中可能需要调用视觉定位或机器人接口。
 */
void StateMachine::executeLoadGraspTask()
{
    const auto poseSource = resolveLoadGraspPoseSource();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("LoadGrasp 位姿来源：")
        << poseSource.sourceName
        << QStringLiteral(" 可用=") << poseSource.available
        << QStringLiteral(" 成功=") << poseSource.success
        << QStringLiteral(" 消息=") << poseSource.message;
    writeLoadGraspResult();   // 写入加载位姿结果到 PLC 寄存器
    const quint16 resultCode = poseSource.success ? 1 : 7;
    completeActiveTask(resultCode, poseSource.success ? protocol::AckState::Completed
                                                       : protocol::AckState::Failed,
                       poseSource.success);
    emit loadGraspFinished(resultCode, poseSource.x, poseSource.y, poseSource.z,
                           poseSource.rx, poseSource.ry, poseSource.rz);
}


}  // namespace scan_tracking::flow_control
