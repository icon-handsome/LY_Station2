#pragma once

#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking {
namespace modbus {
class ModbusService;
}
namespace orbbec_gemini {
class OrbbecGeminiService;
}
namespace vision {
class VisionPipelineService;
}
namespace flow_control {

/// 上/下料位姿占位解析结果（LoadGrasp / UnloadCalc Handler 使用）。
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

/// Handler 执行 PLC 触发任务时所需的 StateMachine 宿主能力（不含 execute* 入口）。
class PlcTaskHost {
public:
    virtual ~PlcTaskHost() = default;

    virtual modbus::ModbusService* modbusService() const = 0;
    virtual orbbec_gemini::OrbbecGeminiService* orbbecGeminiService() const = 0;
    virtual vision::VisionPipelineService* visionPipelineService() const = 0;
    virtual bool isModbusConnected() const = 0;

    virtual bool completeActiveTask(
        quint16 resultCode,
        protocol::AckState finalAckState = protocol::AckState::Completed,
        bool dataValid = true) = 0;

    virtual void publishIpcStatus() = 0;
    virtual void setTaskProgress(quint16 progress) = 0;

    virtual PoseSourceResult resolveLoadGraspPoseSource() const = 0;
    virtual PoseSourceResult resolveUnloadCalcPoseSource() const = 0;
    virtual void writeLoadGraspResult(const PoseSourceResult& pose) = 0;
    virtual void writeUnloadCalcResult(const PoseSourceResult& pose) = 0;
    virtual void writeFloatPlaceholder(int startOffset, float value) = 0;
    virtual void writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text) = 0;
    virtual void clearInspectionResultRegisters() = 0;
    virtual bool writeSelfCheckFailWords(const QVector<quint16>& failWords) = 0;
    virtual bool clearScanSegmentDoneRegisters() = 0;
    virtual bool clearIpcSafetyActionWord() = 0;

    virtual void completeScanSegmentCapture(
        quint16 resultCode,
        int imageCount,
        int cloudFrameCount,
        protocol::AckState finalAckState,
        bool dataValid) = 0;
    virtual void notifyScanStarted(int segmentIndex, quint32 taskId) = 0;

    virtual InspectionResult evaluateInspectionForActiveTask() const = 0;
    virtual void finishInspection(const InspectionResult& result) = 0;

    virtual void resetScanSegmentCache() = 0;
    virtual void resetSafetyInterlockState() = 0;

    virtual void notifyLoadGraspFinished(quint16 resultCode, const PoseSourceResult& pose) = 0;
    virtual void notifyUnloadCalcFinished(quint16 resultCode, const PoseSourceResult& pose) = 0;
    virtual void notifyPoseCheckFinished(
        bool success,
        quint16 resultCode,
        double poseDeviationMm,
        const QVector<double>& rt,
        const QString& message) = 0;
    virtual void notifySelfCheckFinished(quint16 resultCode, quint16 failWord0) = 0;
    virtual void notifyCodeReadFinished(quint16 resultCode, const QString& codeValue) = 0;
    virtual void notifyResultResetFinished(quint16 resultCode) = 0;
};

}  // namespace flow_control
}  // namespace scan_tracking
