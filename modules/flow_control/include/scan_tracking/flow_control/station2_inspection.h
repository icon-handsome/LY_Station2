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

/// 基于 ScanSegmentCache 执行第二工位综合检测（pathId 以现场配置为准）。
/// - weld_section：焊缝截面（path1/5）
/// - length_volume：外表面长度 + 容积（path3）
/// - thickness_inner_surface：厚度配对 + 内表面两端平均（path4）
/// - 其它算法本轮返回明确未接入
///
/// 内部持有全局评估锁；后台工作线程可阻塞等待。GUI/主线程请用 tryEvaluateStation2Inspection。
InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota);

/// 非阻塞评估：拿不到锁时返回 false，并写入 resultCode=3 的繁忙说明（不阻塞 GUI）。
bool tryEvaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out);

}  // namespace scan_tracking::flow_control
