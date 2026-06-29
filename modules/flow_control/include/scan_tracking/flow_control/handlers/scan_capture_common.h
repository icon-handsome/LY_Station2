#pragma once

#include "scan_tracking/flow_control/task_handler_context.h"

namespace scan_tracking {
namespace flow_control {

/// 按 scan_paths 配置发起分段采集；telescopicInternal 区分机械臂外部 / 伸缩杆内部触发。
void executeConfiguredScanCapture(
    TaskHandlerContext& ctx,
    const char* triggerLabel,
    bool telescopicInternal);

}  // namespace flow_control
}  // namespace scan_tracking
