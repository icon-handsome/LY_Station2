#include "scan_tracking/flow_control/station2_inspection.h"

#include <QtCore/QLoggingCategory>

Q_LOGGING_CATEGORY(LOG_STATION2_INSPECTION, "flow_control.station2_inspection")

namespace scan_tracking::flow_control {

namespace {

constexpr quint16 kNgReasonIncompleteSegments = 1u << 0;
constexpr quint16 kNgReasonBundleInvalid = 1u << 1;

}  // namespace

InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    int expectedSegmentCount)
{
    InspectionResult result;
    result.sourcePointCount = expectedSegmentCount > 0 ? expectedSegmentCount : cache.cachedSegmentCount();

    if (cache.cachedSegmentCount() == 0) {
        result.resultCode = 3;
        result.message = QStringLiteral("无扫描段缓存，请先完成 ScanSegment 采集。");
        return result;
    }

    const quint32 cacheTaskId = cache.runTaskId();
    if (taskId != 0 && cacheTaskId != 0 && cacheTaskId != taskId) {
        result.resultCode = 3;
        result.message = QStringLiteral("缓存 taskId=%1 与当前任务 taskId=%2 不一致。")
                             .arg(cacheTaskId)
                             .arg(taskId);
        return result;
    }

    if (expectedSegmentCount > 0 && cache.cachedSegmentCount() < expectedSegmentCount) {
        result.resultCode = 3;
        result.ngReasonWord0 = kNgReasonIncompleteSegments;
        result.message = QStringLiteral("扫描段不完整：已缓存 %1/%2 段。")
                             .arg(cache.cachedSegmentCount())
                             .arg(expectedSegmentCount);
        return result;
    }

    for (int segmentIndex : cache.cachedSegmentIndices()) {
        const ScanSegmentCacheEntry* entry = cache.entry(segmentIndex);
        if (entry == nullptr || !entry->bundle.success()) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonBundleInvalid;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message = QStringLiteral("段 %1 采集数据无效，无法检测。").arg(segmentIndex);
            return result;
        }
    }

    result.resultCode = 1;
    result.measureItemCount = 1;
    result.measurement.qualityCode = 1;
    result.message = QStringLiteral(
        "检测通过（占位：已校验 %1 段缓存；缺陷/编号/3D 算法待接入）")
                         .arg(cache.cachedSegmentCount());
    return result;
}

}  // namespace scan_tracking::flow_control
