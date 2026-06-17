#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

#include <atomic>
#include <array>
#include <functional>
#include <memory>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"
#include "scan_tracking/flow_control/task_handler_context.h"
#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace mech_eye {
class MechEyeService;
}
namespace vision {
class VisionPipelineService;
}
namespace flow_control {

class TaskHandlerRegistry;

enum class AppState {
    Init,
    Ready,
    Scanning,
    Error,
};

class StateMachine : public QObject {
    Q_OBJECT

public:
    explicit StateMachine(
        modbus::ModbusService* modbusService,
        mech_eye::MechEyeService* mechEyeService = nullptr,
        vision::VisionPipelineService* visionPipelineService = nullptr,
        QObject* parent = nullptr);
    ~StateMachine();

    void start();
    void stop();

    AppState currentState() const { return m_state; }
    protocol::IpcState ipcState() const { return m_ipcState; }
    protocol::Stage currentStage() const { return m_currentStage; }
    quint16 alarmLevel() const { return m_alarmLevel; }
    quint16 alarmCode() const { return m_alarmCode; }
    quint16 warnCode() const { return m_warnCode; }
    quint16 progress() const { return m_progress; }

    const QVector<quint16>& lastCommandBlock() const { return m_lastCommandBlock; }
    protocol::registers::Pose6f robotTcpPose() const { return m_robotTcpPose; }
    quint16 robotStatusWord() const
    {
        return m_lastCommandBlock.value(protocol::registers::kRobotStatusWord, 0);
    }

    void setAlarm(quint16 level, quint16 code, const QString& message);
    bool reportPersonZoneAlarm(bool alarm);

    using InspectionResultPublisher = std::function<void(const InspectionResult&)>;
    void setInspectionResultPublisher(InspectionResultPublisher publisher);

    /// 从段缓存评估检测结果（不写 PLC）；供 HMI cmd.debug_trigger_inspection 使用。
    InspectionResult evaluateCachedInspection(quint32 taskId = 0) const;

    const ScanSegmentCache& scanSegmentCache() const { return m_scanSegmentCache; }

signals:
    void stateChanged(AppState newState);
    void protocolEvent(const QString& message);
    void scanStarted(int segmentIndex, quint32 taskId);
    void scanFinished(int segmentIndex, quint16 resultCode, int imageCount, int cloudFrameCount);
    void inspectionFinished(quint16 resultCode, quint16 ngReasonWord0, quint16 ngReasonWord1,
                            quint16 measureItemCount,
                            const InspectionMeasurement& measurement,
                            const QString& message);
    void poseCheckFinished(bool success, quint16 resultCode, double poseDeviationMm,
                           const QVector<double>& rt, const QString& message);
    void loadGraspFinished(quint16 resultCode, float x, float y, float z,
                           float rx, float ry, float rz);
    void unloadCalcFinished(quint16 resultCode, float x, float y, float z,
                            float rx, float ry, float rz);
    void selfCheckFinished(quint16 resultCode, quint16 failWord0);
    void codeReadFinished(quint16 resultCode, const QString& codeValue);
    void resultResetFinished(quint16 resultCode);

private slots:
    void pollPlcState();
    void handleRegistersRead(int startAddress, const QVector<quint16>& values);
    void onRegisterReadFailed(int startAddress, const QString& errorString);
    void onRegisterWriteFailed(int startAddress, const QString& errorString);
    void onModbusConnected();
    void onModbusDisconnected();
    void onModbusError(const QString& errorString);
    void onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message);
    void onVisionPipelineFatalError(vision::VisionErrorCode code, QString message);
    void onBundleCaptureFinished(vision::MultiCameraCaptureBundle bundle);
    void onProcessTimeout();

private:
    friend class CodeReadHandler;
    friend class InspectionHandler;
    friend class LoadGraspHandler;
    friend class PoseCheckHandler;
    friend class ResultResetHandler;
    friend class ScanSegmentHandler;
    friend class SelfCheckHandler;
    friend class StationMaterialCheckHandler;
    friend class UnloadCalcHandler;

public:
    struct PoseSourceResult {
        bool available = false;
        bool success = false;
        QString sourceName;
        QString message;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float rx = 0.0f;
        float ry = 0.0f;
        float rz = 0.0f;
    };

private:
    struct InspectionSummary {
        quint16 resultCode = 1;
        quint16 ngReasonWord0 = 0;
        quint16 ngReasonWord1 = 0;
        quint16 measureItemCount = 0;
    };

    void setState(AppState newState);
    void processTrigger(const protocol::TriggerDefinition& trigger, const QVector<quint16>& commandBlock);
    void rejectDisabledTrigger(const protocol::TriggerDefinition& trigger);
    void executeActiveTask();
    void executeLoadGraspTask();
    void executeStationMaterialCheckTask();
    void executeUnloadCalcTask();
    void executeScanSegmentTask();
    void executeInspectionTask();
    void executePoseCheckTask();
    void executeSelfCheckTask();
    void executeCodeReadTask();
    void executeResultResetTask();

    void sendAck(const protocol::TriggerDefinition& definition, protocol::AckState ackState);
    void sendRes(const protocol::TriggerDefinition& definition, quint16 resultCode);
    void publishIpcStatus();
    void resetPlcOutputRegisters();
    void publishHeartbeat();
    bool completeActiveTask(
        quint16 resultCode,
        protocol::AckState finalAckState = protocol::AckState::Completed,
        bool dataValid = true);
    void finalizeCompletedTaskIfTriggerReleased(const QVector<quint16>& commandBlock);
    void clearActiveTask();
    void resetScanSegmentCache();

    const protocol::TriggerDefinition* selectPendingTrigger(const QVector<quint16>& commandBlock) const;

    void recordModbusFailure(quint16 alarmCode, const QString& message);
    void resetModbusFailureCounter();
    void enterFaultState(quint16 alarmCode, const QString& message, bool abortCurrentTask, bool notifyPlc);
    void abortActiveTaskForFault(quint16 resultCode);
    bool writeIpcSafetyActionWord();

    void writeFloatPlaceholder(int startOffset, float value);
    PoseSourceResult resolveLoadGraspPoseSource() const;
    PoseSourceResult resolveUnloadCalcPoseSource() const;
    void writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text);
    void writeLoadGraspResult();
    void writeUnloadCalcResult();
    void writeScanSegmentResult(int segmentIndex, int imageCount, int cloudFrameCount);
    void writeInspectionResult(const InspectionSummary& summary);
    void completeScanSegmentCapture(
        quint16 resultCode,
        int imageCount,
        int cloudFrameCount,
        protocol::AckState finalAckState,
        bool dataValid);
    void finishInspection(const InspectionResult& result);
    int resolveExpectedScanSegmentCount() const;

    quint32 readTaskId(const QVector<quint16>& commandBlock) const;
    quint16 resolveScanSegmentIndex(const QVector<quint16>& commandBlock) const;

    modbus::ModbusService* m_modbus = nullptr;
    mech_eye::MechEyeService* m_mechEye = nullptr;
    vision::VisionPipelineService* m_visionPipeline = nullptr;
    QTimer* m_pollTimer = nullptr;
    QTimer* m_heartbeatTimer = nullptr;
    QTimer* m_timeoutTimer = nullptr;
    std::unique_ptr<TaskHandlerRegistry> m_handlerRegistry;
    AppState m_state = AppState::Init;
    protocol::IpcState m_ipcState = protocol::IpcState::Uninitialized;
    protocol::Stage m_currentStage = protocol::Stage::Idle;
    ActiveTaskState m_activeTask;
    quint16 m_heartbeatCounter = 0;
    quint16 m_alarmLevel = 0;
    quint16 m_alarmCode = 0;
    quint16 m_warnCode = 0;
    quint16 m_progress = 0;
    quint16 m_ipcSafetyActionWord = 0;
    bool m_personZoneAlarmActive = false;
    bool m_dataValid = false;
    bool m_isPollingPlc = false;
    quint64 m_pollRequestSequence = 0;
    quint64 m_activePollRequestSequence = 0;
    QElapsedTimer m_pollRequestTimer;
    int m_consecutiveModbusFailures = 0;
    QVector<quint16> m_lastCommandBlock;
    protocol::registers::Pose6f m_robotTcpPose;
    ScanSegmentCache m_scanSegmentCache;
    InspectionResultPublisher m_inspectionResultPublisher;
    std::atomic_bool m_stopped{false};

    static constexpr int kMaxConsecutiveModbusFailures = 3;
};

}  // namespace flow_control
}  // namespace scan_tracking
