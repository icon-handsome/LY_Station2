#pragma once

#include <QtCore/QVector>

#include "scan_tracking/flow_control/plc_protocol.h"

namespace scan_tracking {
namespace flow_control {

class StateMachine;

struct ActiveTaskState {
    const protocol::TriggerDefinition* definition = nullptr;  // 触发定义指针
    quint32 taskId = 0;                                        // 任务 ID
    quint16 timeoutSeconds = 0;                                // 超时时间（秒）
    bool completionAnnounced = false;                          // 是否已宣布完成
    int scanSegmentIndex = 0;                                  // 扫描段索引
    int scanSegmentTotal = 0;                                  // 扫描段总数
    int inspectionPathId = 0;                                  // 本次 Trig_Inspection 对应的路径 ID
    quint64 captureRequestId = 0;                              // 采集请求 ID
};

struct TaskHandlerContext {
    StateMachine& machine;
    const QVector<quint16>& commandBlock;
    ActiveTaskState& activeTask;
};

}  // namespace flow_control
}  // namespace scan_tracking
