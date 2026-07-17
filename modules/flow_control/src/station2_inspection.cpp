#include "scan_tracking/flow_control/station2_inspection.h"

#include "scan_tracking/inner_surface_measure/inner_surface_measure_service.h"
#include "scan_tracking/thickness_measure/thickness_measure_service.h"
#include "scan_tracking/weld_measure/weld_measure_service.h"

#include "scan_tracking/common/config_manager.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QPair>
#include <QtCore/QVector>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

Q_LOGGING_CATEGORY(LOG_STATION2_INSPECTION, "flow_control.station2_inspection")

namespace scan_tracking::flow_control {

namespace {

constexpr quint16 kNgReasonIncompleteSegments = 1u << 0;
constexpr quint16 kNgReasonBundleInvalid = 1u << 1;
constexpr quint16 kNgReasonPointCloudInvalid = 1u << 2;
constexpr quint16 kNgReasonAlgorithmFailed = 1u << 3;
constexpr quint16 kNgReasonAlgorithmUnsupported = 1u << 4;

constexpr double kPi = 3.14159265358979323846;

void fillPathMeta(InspectionResult* result, const InspectionQuota& quota)
{
    if (result == nullptr) {
        return;
    }
    result->pathId = quota.pathId;
    result->pathName = quota.pathName;
    result->algorithm = quota.algorithm;
}

bool isWithinDeviceQuota(
    common::ScanDeviceKind device,
    int localIndex,
    const InspectionQuota& quota)
{
    if (localIndex <= 0) {
        return false;
    }
    if (device == common::ScanDeviceKind::Telescopic) {
        return quota.expectedTelescopicCount > 0 && localIndex <= quota.expectedTelescopicCount;
    }
    return quota.expectedArmCount > 0 && localIndex <= quota.expectedArmCount;
}

QString incompleteQuotaMessage(
    int armCached,
    int telescopicCached,
    const InspectionQuota& quota)
{
    if (quota.expectedTelescopicCount > 0 && quota.expectedArmCount > 0) {
        return QStringLiteral("扫描未齐：pathId=%1 机械臂 %2/%3，伸缩杆 %4/%5")
            .arg(quota.pathId)
            .arg(armCached)
            .arg(quota.expectedArmCount)
            .arg(telescopicCached)
            .arg(quota.expectedTelescopicCount);
    }
    if (quota.expectedTelescopicCount > 0) {
        return QStringLiteral("扫描未齐：pathId=%1 伸缩杆 %2/%3")
            .arg(quota.pathId)
            .arg(telescopicCached)
            .arg(quota.expectedTelescopicCount);
    }
    return QStringLiteral("扫描未齐：pathId=%1 机械臂 %2/%3")
        .arg(quota.pathId)
        .arg(armCached)
        .arg(quota.expectedArmCount);
}

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

InspectionResult rejectUnsupportedAlgorithm(const InspectionQuota& quota)
{
    InspectionResult result;
    fillPathMeta(&result, quota);
    result.resultCode = 3;
    result.ngReasonWord0 = kNgReasonAlgorithmUnsupported;
    result.sourcePointCount = quota.total();
    result.message =
        QStringLiteral("pathId=%1 (%2) algorithm=%3 尚未接入 Trig_Inspection，"
                       "请切换 activePathId 至焊缝路径（1/5）或等待后续轮次。")
            .arg(quota.pathId)
            .arg(quota.pathName.isEmpty() ? QStringLiteral("-") : quota.pathName)
            .arg(quota.algorithm.isEmpty() ? QStringLiteral("(empty)") : quota.algorithm);
    return result;
}

InspectionResult evaluateWeldSectionInspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota)
{
    InspectionResult result;
    fillPathMeta(&result, quota);
    const int expectedTotal = quota.total();
    result.sourcePointCount = expectedTotal > 0 ? expectedTotal : cache.cachedSegmentCount();

    if (cache.cachedSegmentCount() == 0) {
        result.resultCode = 3;
        result.message = QStringLiteral("pathId=%1：无扫描段缓存，请先完成机械臂/伸缩杆采集。")
                             .arg(quota.pathId);
        return result;
    }

    const quint32 cacheTaskId = cache.runTaskId();
    if (taskId != 0 && cacheTaskId != 0 && cacheTaskId != taskId) {
        result.resultCode = 3;
        result.message = QStringLiteral("pathId=%1：缓存 taskId=%2 与当前任务 taskId=%3 不一致。")
                             .arg(quota.pathId)
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
        result.message = incompleteQuotaMessage(armCached, telescopicCached, quota);
        return result;
    }

    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        if (!isWithinDeviceQuota(key.device, key.localIndex, quota)) {
            continue;
        }
        const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex);
        if (entry == nullptr || !entry->bundle.success()) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonBundleInvalid;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message =
                QStringLiteral("pathId=%1 %2 段 %3 采集数据无效，无法检测。")
                    .arg(quota.pathId)
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
        result.message = QStringLiteral("pathId=%1 焊缝测量算法初始化失败：%2")
                             .arg(quota.pathId)
                             .arg(initError);
        return result;
    }

    auto& weldService = sharedWeldMeasureService();
    std::vector<float> xyz;
    int measuredOk = 0;

    qInfo(LOG_STATION2_INSPECTION).noquote()
        << QStringLiteral("开始焊缝截面测量 pathId=") << quota.pathId
        << QStringLiteral(" name=") << quota.pathName
        << QStringLiteral(" algorithm=") << quota.algorithm
        << QStringLiteral(" 配额臂=") << quota.expectedArmCount
        << QStringLiteral(" 伸缩杆=") << quota.expectedTelescopicCount;

    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        if (!isWithinDeviceQuota(key.device, key.localIndex, quota)) {
            continue;
        }

        const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex);
        const auto& cloud = entry->bundle.mechEyeResult.pointCloud;

        int finiteCount = 0;
        if (!extractFiniteXyz(cloud, &xyz, &finiteCount)) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonPointCloudInvalid;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message =
                QStringLiteral("pathId=%1 %2 段 %3 点云无效或无可测点。")
                    .arg(quota.pathId)
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
                QStringLiteral("pathId=%1 %2 段 %3 焊缝测量失败：%4")
                    .arg(quota.pathId)
                    .arg(common::ConfigManager::scanDeviceKindToString(key.device))
                    .arg(key.localIndex)
                    .arg(error.message);
            qWarning(LOG_STATION2_INSPECTION)
                << "pathId" << quota.pathId
                << common::ConfigManager::scanDeviceKindToString(key.device)
                << "localIndex" << key.localIndex
                << "measure failed:" << error.message
                << "points=" << finiteCount;
            return result;
        }

        accumulateMeasurement(&result.measurement, section);
        ++measuredOk;
        qInfo(LOG_STATION2_INSPECTION)
            << "pathId" << quota.pathId
            << common::ConfigManager::scanDeviceKindToString(key.device)
            << "localIndex" << key.localIndex
            << "mismatch=" << section.mismatchMm
            << "reinforcement=" << section.reinforcementMm
            << "angularity=" << section.angularityMm
            << "maxUndercut=" << section.maxUndercutMm
            << "points=" << finiteCount;
    }

    if (measuredOk <= 0) {
        result.resultCode = 3;
        result.ngReasonWord0 = kNgReasonIncompleteSegments;
        result.message = QStringLiteral("pathId=%1：配额内无可测段（臂 %2/%3，伸缩杆 %4/%5）。")
                             .arg(quota.pathId)
                             .arg(armCached)
                             .arg(quota.expectedArmCount)
                             .arg(telescopicCached)
                             .arg(quota.expectedTelescopicCount);
        return result;
    }

    result.resultCode = 1;
    result.measureItemCount = 1;
    result.measurement.qualityCode = 1;
    result.message = QStringLiteral(
        "焊缝检测通过：pathId=%1 (%2) 已测量 %3 段（机械臂 %4/%5，伸缩杆 %6/%7）；"
        "错边=%8mm 余高=%9mm 棱角度=%10mm 最大咬边=%11mm")
                         .arg(quota.pathId)
                         .arg(quota.pathName.isEmpty() ? QStringLiteral("weld") : quota.pathName)
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

enum class PointPurposeKind {
    Unknown = 0,
    ThicknessInner,
    ThicknessOuter,
    ThicknessGeneric,
    InnerSurface,
};

PointPurposeKind classifyPointPurpose(const QString& purpose)
{
    const QString normalized = purpose.trimmed().toLower();
    if (normalized == QLatin1String("thickness_inner") ||
        normalized == QLatin1String("thickness.inner")) {
        return PointPurposeKind::ThicknessInner;
    }
    if (normalized == QLatin1String("thickness_outer") ||
        normalized == QLatin1String("thickness.outer")) {
        return PointPurposeKind::ThicknessOuter;
    }
    if (normalized == QLatin1String("thickness")) {
        return PointPurposeKind::ThicknessGeneric;
    }
    if (normalized == QLatin1String("inner_surface") ||
        normalized == QLatin1String("inner") ||
        normalized.startsWith(QLatin1String("inner_"))) {
        return PointPurposeKind::InnerSurface;
    }
    return PointPurposeKind::Unknown;
}

struct SegmentCloud {
    int localIndex = 0;
    PointPurposeKind purpose = PointPurposeKind::Unknown;
    std::vector<float> xyz;
    int finiteCount = 0;
};

QString purposeForLocalIndex(int localIndex)
{
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        return cfgMgr->pointPurpose(localIndex);
    }
    return {};
}

bool loadQuotaSegmentClouds(
    const ScanSegmentCache& cache,
    const InspectionQuota& quota,
    QVector<SegmentCloud>* out,
    InspectionResult* failResult)
{
    if (out == nullptr || failResult == nullptr) {
        return false;
    }
    out->clear();

    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        if (!isWithinDeviceQuota(key.device, key.localIndex, quota)) {
            continue;
        }
        // path3 仅机械臂
        if (key.device != common::ScanDeviceKind::Arm) {
            continue;
        }

        const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex);
        if (entry == nullptr || !entry->bundle.success()) {
            failResult->resultCode = 2;
            failResult->ngReasonWord0 = kNgReasonBundleInvalid;
            failResult->measurement.qualityCode = 2;
            failResult->measureItemCount = 1;
            failResult->message =
                QStringLiteral("pathId=%1 段 %2 采集数据无效，无法检测。")
                    .arg(quota.pathId)
                    .arg(key.localIndex);
            return false;
        }

        SegmentCloud cloud;
        cloud.localIndex = key.localIndex;
        cloud.purpose = classifyPointPurpose(purposeForLocalIndex(key.localIndex));
        if (!extractFiniteXyz(entry->bundle.mechEyeResult.pointCloud, &cloud.xyz, &cloud.finiteCount)) {
            failResult->resultCode = 2;
            failResult->ngReasonWord0 = kNgReasonPointCloudInvalid;
            failResult->measurement.qualityCode = 2;
            failResult->measureItemCount = 1;
            failResult->message =
                QStringLiteral("pathId=%1 段 %2 点云无效或无可测点。")
                    .arg(quota.pathId)
                    .arg(key.localIndex);
            return false;
        }
        out->push_back(std::move(cloud));
    }

    std::sort(out->begin(), out->end(), [](const SegmentCloud& a, const SegmentCloud& b) {
        return a.localIndex < b.localIndex;
    });
    return true;
}

struct ThicknessPairRefs {
    int innerIndex = 0;
    int outerIndex = 0;
    const SegmentCloud* inner = nullptr;
    const SegmentCloud* outer = nullptr;
};

QVector<ThicknessPairRefs> buildThicknessPairs(const QVector<SegmentCloud>& clouds)
{
    QVector<ThicknessPairRefs> pairs;
    QVector<const SegmentCloud*> inners;
    QVector<const SegmentCloud*> outers;
    QVector<const SegmentCloud*> generics;

    for (const SegmentCloud& cloud : clouds) {
        switch (cloud.purpose) {
        case PointPurposeKind::ThicknessInner:
            inners.push_back(&cloud);
            break;
        case PointPurposeKind::ThicknessOuter:
            outers.push_back(&cloud);
            break;
        case PointPurposeKind::ThicknessGeneric:
            generics.push_back(&cloud);
            break;
        default:
            break;
        }
    }

    if (!inners.isEmpty() || !outers.isEmpty()) {
        const int n = qMin(inners.size(), outers.size());
        pairs.reserve(n);
        for (int i = 0; i < n; ++i) {
            ThicknessPairRefs pair;
            pair.inner = inners[i];
            pair.outer = outers[i];
            pair.innerIndex = inners[i]->localIndex;
            pair.outerIndex = outers[i]->localIndex;
            pairs.push_back(pair);
        }
        return pairs;
    }

    // 兼容 purpose=thickness：按顺序两两配对（先内后外）
    for (int i = 0; i + 1 < generics.size(); i += 2) {
        ThicknessPairRefs pair;
        pair.inner = generics[i];
        pair.outer = generics[i + 1];
        pair.innerIndex = generics[i]->localIndex;
        pair.outerIndex = generics[i + 1]->localIndex;
        pairs.push_back(pair);
    }
    return pairs;
}

QVector<QPair<const SegmentCloud*, const SegmentCloud*>> buildInnerSurfacePairs(
    const QVector<SegmentCloud>& clouds)
{
    QVector<const SegmentCloud*> frames;
    for (const SegmentCloud& cloud : clouds) {
        if (cloud.purpose == PointPurposeKind::InnerSurface) {
            frames.push_back(&cloud);
        }
    }

    QVector<QPair<const SegmentCloud*, const SegmentCloud*>> pairs;
    for (int i = 0; i + 1 < frames.size(); i += 2) {
        pairs.push_back(qMakePair(frames[i], frames[i + 1]));
    }
    return pairs;
}

scan_tracking::thickness_measure::ThicknessMeasureService& sharedThicknessMeasureService()
{
    static scan_tracking::thickness_measure::ThicknessMeasureService service;
    return service;
}

scan_tracking::inner_surface_measure::InnerSurfaceMeasureService& sharedInnerSurfaceMeasureService()
{
    static scan_tracking::inner_surface_measure::InnerSurfaceMeasureService service;
    return service;
}

bool ensureThicknessMeasureReady(QString* errorMessage)
{
    auto& service = sharedThicknessMeasureService();
    if (service.isReady()) {
        return true;
    }
    scan_tracking::thickness_measure::ThicknessMeasureError error;
    if (!service.initializeFromJson(QString(), &error)) {
        if (errorMessage != nullptr) {
            *errorMessage = error.message + QStringLiteral(
                "（请确认 config/thickness_measure/thickness_config.json 及模板 PCD 路径可用）");
        }
        return false;
    }
    return true;
}

bool ensureInnerSurfaceMeasureReady(QString* errorMessage)
{
    auto& service = sharedInnerSurfaceMeasureService();
    if (service.isReady()) {
        return true;
    }
    scan_tracking::inner_surface_measure::InnerSurfaceMeasureError error;
    if (!service.initializeFromIni(QString(), &error)) {
        if (errorMessage != nullptr) {
            *errorMessage = error.message + QStringLiteral(
                "（请确认 config/inner_surface_measure/config.ini 及模板 PCD 路径可用）");
        }
        return false;
    }
    return true;
}

InspectionResult evaluateThicknessInnerSurfaceInspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota)
{
    InspectionResult result;
    fillPathMeta(&result, quota);
    result.sourcePointCount = quota.total() > 0 ? quota.total() : cache.cachedSegmentCount();

    if (cache.cachedSegmentCount() == 0) {
        result.resultCode = 3;
        result.message = QStringLiteral("pathId=%1：无扫描段缓存，请先完成机械臂采集。")
                             .arg(quota.pathId);
        return result;
    }

    const quint32 cacheTaskId = cache.runTaskId();
    if (taskId != 0 && cacheTaskId != 0 && cacheTaskId != taskId) {
        result.resultCode = 3;
        result.message = QStringLiteral("pathId=%1：缓存 taskId=%2 与当前任务 taskId=%3 不一致。")
                             .arg(quota.pathId)
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
        result.message = incompleteQuotaMessage(armCached, telescopicCached, quota);
        return result;
    }

    QVector<SegmentCloud> clouds;
    if (!loadQuotaSegmentClouds(cache, quota, &clouds, &result)) {
        fillPathMeta(&result, quota);
        return result;
    }

    const QVector<ThicknessPairRefs> thicknessPairs = buildThicknessPairs(clouds);
    const auto innerPairs = buildInnerSurfacePairs(clouds);
    if (thicknessPairs.isEmpty() && innerPairs.isEmpty()) {
        result.resultCode = 3;
        result.ngReasonWord0 = kNgReasonIncompleteSegments;
        result.message = QStringLiteral(
            "pathId=%1：未找到 thickness/inner_surface 点位用途（请检查 scan_paths points[].purpose）。")
                             .arg(quota.pathId);
        return result;
    }

    qInfo(LOG_STATION2_INSPECTION).noquote()
        << QStringLiteral("开始厚度+内表面测量 pathId=") << quota.pathId
        << QStringLiteral(" name=") << quota.pathName
        << QStringLiteral(" 厚度对数=") << thicknessPairs.size()
        << QStringLiteral(" 内表面对数=") << innerPairs.size();

    // --- 厚度 ---
    if (!thicknessPairs.isEmpty()) {
        QString initError;
        if (!ensureThicknessMeasureReady(&initError)) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message = QStringLiteral("pathId=%1 厚度测量初始化失败：%2")
                                 .arg(quota.pathId)
                                 .arg(initError);
            return result;
        }

        QVector<scan_tracking::thickness_measure::ThicknessPairClouds> apiPairs;
        apiPairs.reserve(thicknessPairs.size());
        for (const ThicknessPairRefs& pair : thicknessPairs) {
            scan_tracking::thickness_measure::ThicknessPairClouds apiPair;
            apiPair.inner.xyz = pair.inner->xyz.data();
            apiPair.inner.pointCount = static_cast<size_t>(pair.inner->finiteCount);
            apiPair.outer.xyz = pair.outer->xyz.data();
            apiPair.outer.pointCount = static_cast<size_t>(pair.outer->finiteCount);
            apiPairs.push_back(apiPair);
            qInfo(LOG_STATION2_INSPECTION)
                << "thickness pair inner=" << pair.innerIndex
                << "outer=" << pair.outerIndex
                << "innerPts=" << pair.inner->finiteCount
                << "outerPts=" << pair.outer->finiteCount;
        }

        scan_tracking::thickness_measure::ThicknessAverageMeasurement avg;
        scan_tracking::thickness_measure::ThicknessMeasureError error;
        if (!sharedThicknessMeasureService().measurePairsAverage(apiPairs, &avg, &error) ||
            !avg.valid || avg.successCount == 0) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.measurement.thicknessPairCount = static_cast<int>(avg.pairCount);
            result.measurement.thicknessSuccessCount = static_cast<int>(avg.successCount);
            result.message = QStringLiteral("pathId=%1 厚度测量失败：%2（成功对 %3/%4）")
                                 .arg(quota.pathId)
                                 .arg(error.message.isEmpty() ? QStringLiteral("无有效厚度对")
                                                             : error.message)
                                 .arg(static_cast<int>(avg.successCount))
                                 .arg(static_cast<int>(avg.pairCount > 0 ? avg.pairCount
                                                                        : thicknessPairs.size()));
            return result;
        }

        result.measurement.thicknessMm = avg.thicknessMm;
        result.measurement.thicknessPairCount = static_cast<int>(avg.pairCount);
        result.measurement.thicknessSuccessCount = static_cast<int>(avg.successCount);
        qInfo(LOG_STATION2_INSPECTION)
            << "pathId" << quota.pathId
            << "thicknessMm=" << avg.thicknessMm
            << "success=" << avg.successCount << "/" << avg.pairCount;
    }

    // --- 内表面 ---
    if (!innerPairs.isEmpty()) {
        QString initError;
        if (!ensureInnerSurfaceMeasureReady(&initError)) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message = QStringLiteral("pathId=%1 内表面测量初始化失败：%2")
                                 .arg(quota.pathId)
                                 .arg(initError);
            return result;
        }

        double sumDiameter = 0.0;
        double sumCircumference = 0.0;
        double sumRoundness = 0.0;
        int success = 0;

        for (const auto& pair : innerPairs) {
            scan_tracking::inner_surface_measure::InnerSurfaceAverageMeasurement avg;
            scan_tracking::inner_surface_measure::InnerSurfaceMeasureError error;
            if (!sharedInnerSurfaceMeasureService().measureTwoFramesAverage(
                    pair.first->xyz.data(),
                    static_cast<size_t>(pair.first->finiteCount),
                    pair.second->xyz.data(),
                    static_cast<size_t>(pair.second->finiteCount),
                    &avg,
                    nullptr,
                    nullptr,
                    &error) ||
                !avg.valid) {
                qWarning(LOG_STATION2_INSPECTION)
                    << "pathId" << quota.pathId
                    << "inner_surface pair" << pair.first->localIndex
                    << pair.second->localIndex
                    << "failed:" << error.message;
                continue;
            }
            sumDiameter += avg.diameterMm;
            sumCircumference += avg.circumferenceMm;
            sumRoundness += avg.roundness;
            ++success;
            qInfo(LOG_STATION2_INSPECTION)
                << "pathId" << quota.pathId
                << "inner_surface pair" << pair.first->localIndex
                << pair.second->localIndex
                << "diameter=" << avg.diameterMm
                << "circumference=" << avg.circumferenceMm
                << "roundness=" << avg.roundness;
        }

        result.measurement.innerSurfacePairCount = innerPairs.size();
        result.measurement.innerSurfaceSuccessCount = success;
        if (success <= 0) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message = QStringLiteral("pathId=%1 内表面测量失败：全部 %2 对无效。")
                                 .arg(quota.pathId)
                                 .arg(innerPairs.size());
            return result;
        }

        result.measurement.innerDiameterMm = sumDiameter / success;
        result.measurement.innerCircumferenceMm = sumCircumference / success;
        result.measurement.innerRoundness = sumRoundness / success;
    }

    result.resultCode = 1;
    result.measureItemCount = static_cast<quint16>(
        (result.measurement.thicknessSuccessCount > 0 ? 1 : 0) +
        (result.measurement.innerSurfaceSuccessCount > 0 ? 1 : 0));
    result.measurement.qualityCode = 1;
    result.measurement.measuredSegmentCount =
        result.measurement.thicknessSuccessCount * 2 +
        result.measurement.innerSurfaceSuccessCount * 2;
    result.message = QStringLiteral(
        "厚度+内表面通过：pathId=%1 (%2)；厚度=%3mm（%4/%5 对），"
        "内径=%6mm 周长=%7mm 圆度=%8（%9/%10 对）")
                         .arg(quota.pathId)
                         .arg(quota.pathName.isEmpty() ? QStringLiteral("thickness_inner_surface")
                                                      : quota.pathName)
                         .arg(result.measurement.thicknessMm, 0, 'f', 3)
                         .arg(result.measurement.thicknessSuccessCount)
                         .arg(result.measurement.thicknessPairCount)
                         .arg(result.measurement.innerDiameterMm, 0, 'f', 3)
                         .arg(result.measurement.innerCircumferenceMm, 0, 'f', 3)
                         .arg(result.measurement.innerRoundness, 0, 'f', 3)
                         .arg(result.measurement.innerSurfaceSuccessCount)
                         .arg(result.measurement.innerSurfacePairCount);
    return result;
}

}  // namespace

InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota)
{
    InspectionQuota effective = quota;
    if (effective.pathId <= 0 || effective.algorithm.isEmpty()) {
        if (const auto* cfgMgr = common::ConfigManager::instance()) {
            if (effective.pathId <= 0) {
                effective.pathId = cfgMgr->activePathId();
            }
            if (effective.pathName.isEmpty()) {
                effective.pathName = cfgMgr->activePathName();
            }
            if (effective.algorithm.isEmpty()) {
                effective.algorithm = cfgMgr->activePathAlgorithm();
            }
        }
    }

    // 无显式算法时，兼容旧配置：有扫描缓存则按焊缝处理
    if (effective.algorithm.isEmpty()) {
        effective.algorithm = QStringLiteral("weld_section");
    }

    if (effective.algorithm == QLatin1String("weld_section")) {
        return evaluateWeldSectionInspection(cache, taskId, effective);
    }
    if (effective.algorithm == QLatin1String("thickness_inner_surface")) {
        return evaluateThicknessInnerSurfaceInspection(cache, taskId, effective);
    }
    if (effective.algorithm == QLatin1String("code_read")) {
        // 同步评估入口不会发起拍照；由 InspectionHandler 走 startCodeReadCapture。
        InspectionResult result;
        result.pathId = effective.pathId;
        result.pathName = effective.pathName;
        result.algorithm = effective.algorithm;
        result.resultCode = 3;
        result.message = QStringLiteral(
            "pathId=%1 code_read 应由 InspectionHandler/Trig_CodeRead 异步发起海康 OCR。")
                             .arg(effective.pathId);
        return result;
    }

    return rejectUnsupportedAlgorithm(effective);
}

}  // namespace scan_tracking::flow_control
