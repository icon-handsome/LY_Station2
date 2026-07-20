#pragma once

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"

namespace scan_tracking::flow_control {

struct InspectionQuota {
    int expectedArmCount = 0;
    int expectedTelescopicCount = 0;
    int pathId = 0;
    QString pathName;
    QString algorithm;

    int total() const { return expectedArmCount + expectedTelescopicCount; }
};

/// 基于 ScanSegmentCache 执行第二工位综合检测。
/// - weld_section：焊缝截面（path1/5）
/// - thickness_inner_surface：厚度配对 + 内表面成对平均（path3）
/// - length_volume：外表面长度 + 容积（path4）
/// - 其它算法本轮返回明确未接入
InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota);

}  // namespace scan_tracking::flow_control
