#include "scan_tracking/flow_control/station2_inspection.h"

#include "scan_tracking/weld_measure/weld_measure_service.h"

#include "scan_tracking/common/config_manager.h"

#include <QtCore/QLoggingCategory>

#include <cmath>
#include <vector>

Q_LOGGING_CATEGORY(LOG_STATION2_INSPECTION, "flow_control.station2_inspection")

namespace scan_tracking::flow_control {

namespace {

constexpr quint16 kNgReasonIncompleteSegments = 1u << 0;
constexpr quint16 kNgReasonBundleInvalid = 1u << 1;
constexpr quint16 kNgReasonPointCloudInvalid = 1u << 2;
constexpr quint16 kNgReasonAlgorithmFailed = 1u << 3;

constexpr double kPi = 3.14159265358979323846;

scan_tracking::weld_measure::WeldMeasureService& sharedWeldMeasureService()
{
    static scan_tracking::weld_measure::WeldMeasureService service;
    return service;
}

bool ensureWeldMeasureReady(QString* errorMessage)
{
    auto& service = sharedWeldMeasureService();
    if (service.isReady()) {
        return true;
    }

    scan_tracking::weld_measure::WeldMeasureError error;
    if (!service.initialize(QString(), &error)) {
        if (errorMessage != nullptr) {
            *errorMessage = error.message;
        }
        return false;
    }
    return true;
}

bool extractFiniteXyz(
    const scan_tracking::mech_eye::PointCloudFrame& frame,
    std::vector<float>* xyzOut,
    int* finitePointCount)
{
    if (xyzOut == nullptr || finitePointCount == nullptr || !frame.isValid()) {
        return false;
    }

    xyzOut->clear();
    xyzOut->reserve(static_cast<size_t>(frame.pointCount) * 3u);
    int finite = 0;
    const auto& points = *frame.pointsXYZ;
    const int count = frame.pointCount;
    for (int i = 0; i < count; ++i) {
        const float x = points[static_cast<size_t>(i) * 3u + 0u];
        const float y = points[static_cast<size_t>(i) * 3u + 1u];
        const float z = points[static_cast<size_t>(i) * 3u + 2u];
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }
        xyzOut->push_back(x);
        xyzOut->push_back(y);
        xyzOut->push_back(z);
        ++finite;
    }

    *finitePointCount = finite;
    return finite > 0;
}

void accumulateMeasurement(
    InspectionMeasurement* agg,
    const scan_tracking::weld_measure::WeldSectionMeasurement& section)
{
    if (agg == nullptr) {
        return;
    }

    if (agg->measuredSegmentCount == 0) {
        agg->mismatchMm = section.mismatchMm;
        agg->reinforcementMm = section.reinforcementMm;
        agg->angularityMm = section.angularityMm;
        agg->includedAngleDeg = section.includedAngleRad * 180.0 / kPi;
        agg->leftUndercutMm = section.leftUndercutMm;
        agg->rightUndercutMm = section.rightUndercutMm;
        agg->maxUndercutMm = section.maxUndercutMm;
    } else {
        // Keep the worst-case undercut / mismatch for multi-segment runs.
        if (section.mismatchMm > agg->mismatchMm) {
            agg->mismatchMm = section.mismatchMm;
        }
        if (section.reinforcementMm > agg->reinforcementMm) {
            agg->reinforcementMm = section.reinforcementMm;
        }
        if (section.angularityMm > agg->angularityMm) {
            agg->angularityMm = section.angularityMm;
        }
        if (section.maxUndercutMm > agg->maxUndercutMm) {
            agg->maxUndercutMm = section.maxUndercutMm;
            agg->leftUndercutMm = section.leftUndercutMm;
            agg->rightUndercutMm = section.rightUndercutMm;
            agg->includedAngleDeg = section.includedAngleRad * 180.0 / kPi;
        }
    }
    ++agg->measuredSegmentCount;
}

}  // namespace

InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota)
{
    InspectionResult result;
    const int expectedTotal = quota.total();
    result.sourcePointCount = expectedTotal > 0 ? expectedTotal : cache.cachedSegmentCount();

    if (cache.cachedSegmentCount() == 0) {
        result.resultCode = 3;
        result.message = QStringLiteral("无扫描段缓存，请先完成机械臂/伸缩杆采集。");
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

    const int armCached = cache.cachedCountForDevice(common::ScanDeviceKind::Arm);
    const int telescopicCached =
        cache.cachedCountForDevice(common::ScanDeviceKind::Telescopic);
    if (!cache.meetsDeviceQuotas(quota.expectedArmCount, quota.expectedTelescopicCount)) {
        result.resultCode = 3;
        result.ngReasonWord0 = kNgReasonIncompleteSegments;
        result.message =
            QStringLiteral("扫描未齐：机械臂 %1/%2，伸缩杆 %3/%4（需两侧均满足才可检测）")
                .arg(armCached)
                .arg(quota.expectedArmCount)
                .arg(telescopicCached)
                .arg(quota.expectedTelescopicCount);
        return result;
    }

    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex);
        if (entry == nullptr || !entry->bundle.success()) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonBundleInvalid;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message =
                QStringLiteral("%1 段 %2 采集数据无效，无法检测。")
                    .arg(common::ConfigManager::scanDeviceKindToString(key.device))
                    .arg(key.localIndex);
            return result;
        }
    }

    QString initError;
    if (!ensureWeldMeasureReady(&initError)) {
        result.resultCode = 2;
        result.ngReasonWord0 = kNgReasonAlgorithmFailed;
        result.measurement.qualityCode = 2;
        result.measureItemCount = 1;
        result.message = QStringLiteral("焊缝测量算法初始化失败：%1").arg(initError);
        return result;
    }

    auto& weldService = sharedWeldMeasureService();
    std::vector<float> xyz;
    int measuredOk = 0;

    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex);
        const auto& cloud = entry->bundle.mechEyeResult.pointCloud;

        int finiteCount = 0;
        if (!extractFiniteXyz(cloud, &xyz, &finiteCount)) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonPointCloudInvalid;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message =
                QStringLiteral("%1 段 %2 点云无效或无可测点。")
                    .arg(common::ConfigManager::scanDeviceKindToString(key.device))
                    .arg(key.localIndex);
            return result;
        }

        scan_tracking::weld_measure::WeldSectionMeasurement section;
        scan_tracking::weld_measure::WeldMeasureError error;
        const size_t pointCount = static_cast<size_t>(finiteCount);
        if (!weldService.measureSection(xyz.data(), pointCount, &section, &error)) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message =
                QStringLiteral("%1 段 %2 焊缝测量失败：%3")
                    .arg(common::ConfigManager::scanDeviceKindToString(key.device))
                    .arg(key.localIndex)
                    .arg(error.message);
            qWarning(LOG_STATION2_INSPECTION)
                << common::ConfigManager::scanDeviceKindToString(key.device)
                << "localIndex" << key.localIndex
                << "measure failed:" << error.message
                << "points=" << finiteCount;
            return result;
        }

        accumulateMeasurement(&result.measurement, section);
        ++measuredOk;
        qInfo(LOG_STATION2_INSPECTION)
            << common::ConfigManager::scanDeviceKindToString(key.device)
            << "localIndex" << key.localIndex
            << "mismatch=" << section.mismatchMm
            << "reinforcement=" << section.reinforcementMm
            << "angularity=" << section.angularityMm
            << "maxUndercut=" << section.maxUndercutMm
            << "points=" << finiteCount;
    }

    result.resultCode = 1;
    result.measureItemCount = 1;
    result.measurement.qualityCode = 1;
    result.message = QStringLiteral(
        "检测通过：已测量 %1 段（机械臂 %2/%3，伸缩杆 %4/%5）；"
        "错边=%6mm 余高=%7mm 棱角度=%8mm 最大咬边=%9mm")
                         .arg(measuredOk)
                         .arg(armCached)
                         .arg(quota.expectedArmCount)
                         .arg(telescopicCached)
                         .arg(quota.expectedTelescopicCount)
                         .arg(result.measurement.mismatchMm, 0, 'f', 3)
                         .arg(result.measurement.reinforcementMm, 0, 'f', 3)
                         .arg(result.measurement.angularityMm, 0, 'f', 3)
                         .arg(result.measurement.maxUndercutMm, 0, 'f', 3);
    return result;
}

}  // namespace scan_tracking::flow_control
