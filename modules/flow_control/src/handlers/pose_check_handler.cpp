#include "scan_tracking/flow_control/handlers/pose_check_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* PoseCheckHandler::triggerName() const { return "Trig_PoseCheck"; }
int PoseCheckHandler::trigOffset() const { return 22; }
void PoseCheckHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executePoseCheckTask(); }

/**
 * @brief 执行位姿检查任务（Trig_PoseCheck）
 *
 * 直接调用 LB 位姿算法，输出位姿偏差值并按正式 PLC 结果码回写。
 */
void StateMachine::executePoseCheckTask()
{
    const QVector<double> identityRt = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    if (m_tracking == nullptr) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("位姿检查：Tracking 服务不可用。");
        writeFloatPlaceholder(protocol::registers::kPoseDeviationMm, 0.0f);
        completeActiveTask(7, protocol::AckState::Failed, false);
        emit poseCheckFinished(false, 7, 0.0, identityRt, QStringLiteral("跟踪服务不可用"));
        return;
    }

    const tracking::PoseCheckResult poseResult = m_tracking->checkPose();
    writeFloatPlaceholder(
        protocol::registers::kPoseDeviationMm,
        static_cast<float>(poseResult.poseDeviationMm));

    if (!poseResult.invoked) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("位姿检查未调用 LB 算法：")
            << poseResult.message;
        completeActiveTask(7, protocol::AckState::Failed, false);
        emit poseCheckFinished(false, 7, poseResult.poseDeviationMm, identityRt, poseResult.message);
        return;
    }

    QVector<double> rt;
    rt.reserve(static_cast<int>(poseResult.rt.size()));
    for (double value : poseResult.rt) {
        rt.append(value);
    }

    if (!poseResult.success) {
        const quint16 resultCode = poseResult.resultCode == 0 ? 7 : poseResult.resultCode;
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("位姿检查失败：")
            << poseResult.message
            << QStringLiteral(" resultCode=") << resultCode
            << QStringLiteral(" deviationMm=") << poseResult.poseDeviationMm;
        completeActiveTask(resultCode, protocol::AckState::Failed, false);
        emit poseCheckFinished(false, resultCode, poseResult.poseDeviationMm, rt, poseResult.message);
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("位姿检查成功")
        << QStringLiteral(" inputPoints=") << poseResult.inputPointCount
        << QStringLiteral(" deviationMm=") << poseResult.poseDeviationMm
        << QStringLiteral(" rt00=") << poseResult.rt[0]
        << QStringLiteral(" hasPoseMatrix=") << poseResult.hasPoseMatrix();
    completeActiveTask(1, protocol::AckState::Completed, true);
    emit poseCheckFinished(true, 1, poseResult.poseDeviationMm, rt, poseResult.message);
}


}  // namespace scan_tracking::flow_control
