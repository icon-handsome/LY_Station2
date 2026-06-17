#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station2_inspection.h"

namespace scan_tracking::flow_control {

void StateMachine::setInspectionResultPublisher(InspectionResultPublisher publisher)
{
    m_inspectionResultPublisher = std::move(publisher);
}

int StateMachine::resolveExpectedScanSegmentCount() const
{
    const auto* configMgr = common::ConfigManager::instance();
    if (configMgr == nullptr) {
        return 0;
    }
    const int fromScanPaths = configMgr->enabledScanPointCount();
    if (fromScanPaths > 0) {
        return fromScanPaths;
    }
    return configMgr->trackingConfig().scanSegmentTotal;
}

InspectionResult StateMachine::evaluateCachedInspection(quint32 taskId) const
{
    const quint32 effectiveTaskId = taskId != 0 ? taskId : m_scanSegmentCache.runTaskId();
    return evaluateStation2Inspection(
        m_scanSegmentCache,
        effectiveTaskId,
        resolveExpectedScanSegmentCount());
}

InspectionResult StateMachine::evaluateInspectionForActiveTask() const
{
    return evaluateCachedInspection(m_activeTask.taskId);
}

void StateMachine::finishInspection(const InspectionResult& result)
{
    InspectionSummary summary;
    summary.resultCode = result.resultCode;
    summary.ngReasonWord0 = result.ngReasonWord0;
    summary.ngReasonWord1 = result.ngReasonWord1;
    summary.measureItemCount = result.measureItemCount;
    writeInspectionResult(summary);

    const bool dataValid = result.resultCode == 1 || result.resultCode == 2;
    const protocol::AckState ackState = dataValid ? protocol::AckState::Completed
                                                  : protocol::AckState::Failed;
    completeActiveTask(result.resultCode, ackState, dataValid);

    emit inspectionFinished(
        result.resultCode,
        result.ngReasonWord0,
        result.ngReasonWord1,
        result.measureItemCount,
        result.measurement,
        result.message);

    if (m_inspectionResultPublisher) {
        m_inspectionResultPublisher(result);
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection：已完成 Res=") << result.resultCode
        << QStringLiteral(" qualityCode=") << result.measurement.qualityCode
        << QStringLiteral(" segments=") << result.sourcePointCount
        << QStringLiteral(" message=") << result.message;
}

}  // namespace scan_tracking::flow_control
