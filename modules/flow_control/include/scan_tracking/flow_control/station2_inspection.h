#pragma once

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"

namespace scan_tracking::flow_control {

/// 基于 ScanSegmentCache 执行第二工位综合检测（当前为占位：校验缓存完整性，未接入缺陷/编号/3D 算法）。
InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    int expectedSegmentCount);

}  // namespace scan_tracking::flow_control
