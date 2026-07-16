#pragma once

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"

namespace scan_tracking::flow_control {

struct InspectionQuota {
    int expectedArmCount = 0;
    int expectedTelescopicCount = 0;

    int total() const { return expectedArmCount + expectedTelescopicCount; }
};

/// 基于 ScanSegmentCache 执行第二工位焊缝截面测量（WeldMeasure.dll）。
/// 当双设备配额均满足（机械臂 N + 伸缩杆 M）后才进入算法。
InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota);

}  // namespace scan_tracking::flow_control
