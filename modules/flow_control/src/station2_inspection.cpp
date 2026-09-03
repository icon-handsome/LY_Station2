#include "scan_tracking/flow_control/station2_inspection.h"

#include "scan_tracking/container_total_length_measure/container_total_length_service.h"
#include "scan_tracking/inner_surface_measure/inner_surface_measure_service.h"
#include "scan_tracking/thickness_measure_v2/thickness_measure_v2_service.h"
#include "scan_tracking/weld_measure/weld_measure_service.h"

#include "scan_tracking/common/config_manager.h"

#include <QtCore/QElapsedTimer>
#include <QtCore/QLoggingCategory>
#include <QtCore/QVector>

#include <algorithm>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

Q_LOGGING_CATEGORY(LOG_STATION2_INSPECTION, "flow_control.station2_inspection")

namespace scan_tracking::flow_control {

namespace {

using ContainerTotalLengthService =
    ::scan_tracking::container_total_length_measure::ContainerTotalLengthService;
using ContainerTotalLengthError =
    ::scan_tracking::container_total_length_measure::ContainerTotalLengthError;
using ContainerTotalLengthMeasurement =
    ::scan_tracking::container_total_length_measure::ContainerTotalLengthMeasurement;

constexpr quint16 kNgReasonIncompleteSegments = 1u << 0;
constexpr quint16 kNgReasonBundleInvalid = 1u << 1;
constexpr quint16 kNgReasonPointCloudInvalid = 1u << 2;
constexpr quint16 kNgReasonAlgorithmFailed = 1u << 3;
constexpr quint16 kNgReasonAlgorithmUnsupported = 1u << 4;

constexpr double kPi = 3.14159265358979323846;

// 同一 taskId 内，length_volume 路径产出的实测长度供内表面容积计算复用。
std::mutex& measuredLengthCacheMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::map<quint32, double>& measuredLengthCache()
{
    static std::map<quint32, double> cache;
    return cache;
}

void rememberMeasuredLength(quint32 taskId, double lengthMm)
{
    if (taskId == 0 || lengthMm <= 0.0) {
        return;
    }
    std::lock_guard<std::mutex> lock(measuredLengthCacheMutex());
    measuredLengthCache()[taskId] = lengthMm;
    // 防止长期运行时任务结果无限增长；保留最近的有限窗口。
    while (measuredLengthCache().size() > 64) {
        measuredLengthCache().erase(measuredLengthCache().begin());
    }
}

double measuredLengthForTask(quint32 taskId)
{
    if (taskId == 0) {
        return 0.0;
    }
    std::lock_guard<std::mutex> lock(measuredLengthCacheMutex());
    const auto it = measuredLengthCache().find(taskId);
    return it == measuredLengthCache().end() ? 0.0 : it->second;
}

void fillPathMeta(InspectionResult* result, const InspectionQuota& quota)
{
    if (result == nullptr) {
        return;
    }
    result->pathId = quota.pathId;
    result->pathName = quota.pathName;
    result->algorithm = quota.algorithm;
}

bool isAlgorithmSolveEnabled()
{
    const auto* cfgMgr = common::ConfigManager::instance();
    return cfgMgr == nullptr || cfgMgr->flowControlConfig().algorithmEnabled;
}

InspectionResult makeAlgorithmDisabledOkResult(const InspectionQuota& quota)
{
    InspectionResult result;
    fillPathMeta(&result, quota);
    result.resultCode = 1;
    result.ngReasonWord0 = 0;
    result.ngReasonWord1 = 0;
    result.measureItemCount = 0;
    result.message = QStringLiteral(
        "algorithmEnabled=false：已跳过算法解算，返回 OK（仅跑采集主流程）");
    return result;
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

scan_tracking::weld_measure::WeldMeasureService& sharedWeldMeasureService(
    common::ScanDeviceKind device,
    bool ringWeld)
{
    static scan_tracking::weld_measure::WeldMeasureService straightArm;
    static scan_tracking::weld_measure::WeldMeasureService straightTelescopic;
    static scan_tracking::weld_measure::WeldMeasureService ringArm;
    static scan_tracking::weld_measure::WeldMeasureService ringTelescopic;
    const bool isArm = device == common::ScanDeviceKind::Arm;
    if (ringWeld) {
        return isArm ? ringArm : ringTelescopic;
    }
    return isArm ? straightArm : straightTelescopic;
}

bool ensureWeldMeasureReadyForDevice(
    common::ScanDeviceKind device,
    bool ringWeld,
    QString* errorMessage)
{
    const bool isArm = (device == common::ScanDeviceKind::Arm);
    auto& service = sharedWeldMeasureService(device, ringWeld);
    if (service.isReady()) {
        return true;
    }

    const QString configPath = ringWeld
        ? (isArm
               ? scan_tracking::weld_measure::WeldMeasureService::defaultRingArmConfigPath()
               : scan_tracking::weld_measure::WeldMeasureService::defaultRingTelescopicConfigPath())
        : (isArm
               ? scan_tracking::weld_measure::WeldMeasureService::defaultArmConfigPath()
               : scan_tracking::weld_measure::WeldMeasureService::defaultTelescopicConfigPath());

    scan_tracking::weld_measure::WeldMeasureError error;
    if (!service.initializeFromIni(configPath, &error)) {
        if (errorMessage != nullptr) {
            *errorMessage = error.message.isEmpty()
                ? QStringLiteral("WeldMeasure initializeFromIni failed：%1").arg(configPath)
                : error.message;
        }
        return false;
    }
    return true;
}

bool appendFiniteXyz(
    const scan_tracking::mech_eye::PointCloudFrame& frame,
    std::vector<float>* xyzOut,
    int* finitePointCount)
{
    if (xyzOut == nullptr || finitePointCount == nullptr || !frame.isValid()) {
        return false;
    }

    const auto& points = *frame.pointsXYZ;
    // pointCount 与缓冲区可能不一致（异常包/半截云）；按实际 float 三元组长度钳制，避免越界读。
    const int maxByBuffer = static_cast<int>(points.size() / 3u);
    const int declared = frame.pointCount > 0 ? frame.pointCount : 0;
    const int count = std::min(declared, maxByBuffer);
    if (count <= 0) {
        *finitePointCount = 0;
        return false;
    }
    if (declared > maxByBuffer) {   
        qWarning(LOG_STATION2_INSPECTION)
            << "pointCount exceeds pointsXYZ size, clamped" 
            << "declared=" << declared
            << "bufferPoints=" << maxByBuffer;
    }

    const size_t oldSize = xyzOut->size();
    xyzOut->resize(oldSize + static_cast<size_t>(count) * 3u);
    size_t write = oldSize;
    int finite = 0;
    for (int i = 0; i < count; ++i) {
        const float x = points[static_cast<size_t>(i) * 3u + 0u];
        const float y = points[static_cast<size_t>(i) * 3u + 1u];
        const float z = points[static_cast<size_t>(i) * 3u + 2u];
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }
        (*xyzOut)[write++] = x;
        (*xyzOut)[write++] = y;
        (*xyzOut)[write++] = z;
        ++finite;
    }
    xyzOut->resize(write);

    *finitePointCount = finite;
    return finite > 0;
}

bool extractFiniteXyz(
    const scan_tracking::mech_eye::PointCloudFrame& frame,
    std::vector<float>* xyzOut,
    int* finitePointCount)
{
    if (xyzOut == nullptr || finitePointCount == nullptr) {
        return false;
    }
    xyzOut->clear();
    return appendFiniteXyz(frame, xyzOut, finitePointCount);
}

void accumulateMeasurement(
    InspectionMeasurement* agg,
    const scan_tracking::weld_measure::WeldFrameMeasurement& frame)
{
    if (agg == nullptr) {
        return;
    }

    const auto& section = frame.average;
    if (agg->measuredSegmentCount == 0) {
        agg->mismatchMm = section.mismatchMm;
        agg->reinforcementMm = section.reinforcementMm;
        agg->angularityMm = section.angularityMm;
        agg->includedAngleDeg = section.includedAngleRad * 180.0 / kPi;
        agg->leftUndercutMm = section.leftUndercutMm;
        agg->rightUndercutMm = section.rightUndercutMm;
        agg->maxUndercutMm = section.maxUndercutMm;
        // Undercut lengths are frame-level (V2 measureFrame), not section-level.
        agg->leftUndercutLengthMm = frame.leftUndercutLengthMm;
        agg->rightUndercutLengthMm = frame.rightUndercutLengthMm;
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
        if (frame.leftUndercutLengthMm > agg->leftUndercutLengthMm) {
            agg->leftUndercutLengthMm = frame.leftUndercutLengthMm;
        }
        if (frame.rightUndercutLengthMm > agg->rightUndercutLengthMm) {
            agg->rightUndercutLengthMm = frame.rightUndercutLengthMm;
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
                       "请先完成该路径算法接入，或禁用该路径后继续。")
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
    const bool ringWeld = quota.pathId == 5 ||
        quota.pathName.trimmed().compare(QStringLiteral("ring_weld"), Qt::CaseInsensitive) == 0;
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
    if (!ensureWeldMeasureReadyForDevice(common::ScanDeviceKind::Arm, ringWeld, &initError)) {
        result.resultCode = 2;
        result.ngReasonWord0 = kNgReasonAlgorithmFailed;
        result.measurement.qualityCode = 2;
        result.measureItemCount = 1;
        result.message = QStringLiteral("pathId=%1 机械臂焊缝测量算法初始化失败：%2")
                             .arg(quota.pathId)
                             .arg(initError);
        return result;
    }
    if (quota.expectedTelescopicCount > 0 &&
        !ensureWeldMeasureReadyForDevice(
            common::ScanDeviceKind::Telescopic, ringWeld, &initError)) {
        result.resultCode = 2;
        result.ngReasonWord0 = kNgReasonAlgorithmFailed;
        result.measurement.qualityCode = 2;
        result.measureItemCount = 1;
        result.message = QStringLiteral("pathId=%1 伸缩杆焊缝测量算法初始化失败：%2")
                             .arg(quota.pathId)
                             .arg(initError);
        return result;
    }

    qInfo(LOG_STATION2_INSPECTION).noquote()
        << QStringLiteral("开始焊缝正式流程测量 pathId=") << quota.pathId
        << QStringLiteral(" name=") << quota.pathName
        << QStringLiteral(" algorithm=") << quota.algorithm
        << QStringLiteral(" 配额臂=") << quota.expectedArmCount
        << QStringLiteral(" 伸缩杆=") << quota.expectedTelescopicCount;

    using WeldFrameMeasurement = scan_tracking::weld_measure::WeldFrameMeasurement;

    struct DeviceMeasurement {
        common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
        std::vector<WeldFrameMeasurement> frames;
        QString error;
        int errorCode = 0;  // 1 = invalid point cloud, 2 = algorithm failure
        int errorLocalIndex = 0;
        int measuredOk = 0;
    };

    std::vector<ScanSegmentCacheKey> armKeys;
    std::vector<ScanSegmentCacheKey> telescopicKeys;
    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        if (!isWithinDeviceQuota(key.device, key.localIndex, quota)) {
            continue;
        }
        (key.device == common::ScanDeviceKind::Arm ? armKeys : telescopicKeys).push_back(key);
    }

    // Each device owns an independent WeldMeasure context. Keep each device's
    // frame order intact while allowing the arm and telescopic pipelines to run
    // concurrently. The outer evaluation mutex still serializes whole paths.
    const auto measureDevice = [&](common::ScanDeviceKind device,
                                   const std::vector<ScanSegmentCacheKey>& keys) {
        DeviceMeasurement output;
        output.device = device;
        output.frames.reserve(keys.size());
        auto& weldService = sharedWeldMeasureService(device, ringWeld);
        std::vector<float> xyz;
        for (const ScanSegmentCacheKey& key : keys) {
            const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex);
            const auto& cloud = entry->bundle.mechEyeResult.pointCloud;

            int finiteCount = 0;
            if (!extractFiniteXyz(cloud, &xyz, &finiteCount)) {
                output.errorCode = 1;
                output.errorLocalIndex = key.localIndex;
                output.error = QStringLiteral("%1 段 %2 点云无效或无可测点。")
                                   .arg(common::ConfigManager::scanDeviceKindToString(device))
                                   .arg(key.localIndex);
                return output;
            }

            WeldFrameMeasurement frame;
            scan_tracking::weld_measure::WeldMeasureError error;
            qInfo(LOG_STATION2_INSPECTION)
                << "pathId" << quota.pathId
                << common::ConfigManager::scanDeviceKindToString(device)
                << "localIndex" << key.localIndex
                << "measureFrame begin fedPoints=" << finiteCount;
            // Formal V2.2 flow: FrameN <-> localIndex, ROI/ICP/section extraction inside DLL.
            if (!weldService.measureFrame(
                    key.localIndex, xyz.data(), static_cast<size_t>(finiteCount), &frame, &error)) {
                output.errorCode = 2;
                output.errorLocalIndex = key.localIndex;
                output.error = error.message;
                qWarning(LOG_STATION2_INSPECTION)
                    << "pathId" << quota.pathId
                    << common::ConfigManager::scanDeviceKindToString(device)
                    << "localIndex" << key.localIndex
                    << "measureFrame failed:" << error.message
                    << "points=" << finiteCount;
                return output;
            }

            output.frames.push_back(frame);
            ++output.measuredOk;
            qInfo(LOG_STATION2_INSPECTION)
                << "pathId" << quota.pathId
                << common::ConfigManager::scanDeviceKindToString(device)
                << "localIndex" << key.localIndex
                << "validSections=" << frame.validSections << "/" << frame.totalSections
                << "mismatch=" << frame.average.mismatchMm
                << "reinforcement=" << frame.average.reinforcementMm
                << "angularity=" << frame.average.angularityMm
                << "maxUndercutDepth=" << frame.average.maxUndercutMm
                << "undercutLengthL/R=" << frame.leftUndercutLengthMm << "/"
                << frame.rightUndercutLengthMm
                << "points=" << finiteCount;
        }
        return output;
    };

    std::future<DeviceMeasurement> armFuture =
        std::async(std::launch::async, measureDevice, common::ScanDeviceKind::Arm, std::cref(armKeys));
    std::future<DeviceMeasurement> telescopicFuture;
    if (!telescopicKeys.empty()) {
        telescopicFuture = std::async(
            std::launch::async,
            measureDevice,
            common::ScanDeviceKind::Telescopic,
            std::cref(telescopicKeys));
    }

    // Consume in the original cache order (Arm before Telescopic), preserving
    // deterministic error precedence and metric aggregation semantics.
    DeviceMeasurement armResult = armFuture.get();
    DeviceMeasurement telescopicResult;
    telescopicResult.device = common::ScanDeviceKind::Telescopic;
    if (!telescopicKeys.empty()) {
        telescopicResult = telescopicFuture.get();
    }

    const auto applyDeviceResult = [&](const DeviceMeasurement& deviceResult) {
        if (deviceResult.errorCode != 0) {
            result.resultCode = 2;
            result.ngReasonWord0 = deviceResult.errorCode == 1
                ? kNgReasonPointCloudInvalid
                : kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            if (deviceResult.errorCode == 1) {
                result.message = QStringLiteral("pathId=%1 %2")
                                     .arg(quota.pathId)
                                     .arg(deviceResult.error);
            } else {
                result.message = QStringLiteral("pathId=%1 %2 段 %3 焊缝测量失败：%4")
                                     .arg(quota.pathId)
                                     .arg(common::ConfigManager::scanDeviceKindToString(
                                         deviceResult.device))
                                     .arg(deviceResult.errorLocalIndex)
                                     .arg(deviceResult.error);
            }
            return false;
        }
        for (const WeldFrameMeasurement& frame : deviceResult.frames) {
            accumulateMeasurement(&result.measurement, frame);
        }
        return true;
    };

    if (!applyDeviceResult(armResult) || !applyDeviceResult(telescopicResult)) {
        return result;
    }

    const int measuredOk = armResult.measuredOk + telescopicResult.measuredOk;

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

/// 按 pathId 查 purpose，避免后台解算期间 activePath 已切走读到错误点表。
QString purposeForLocalIndex(int pathId, int localIndex)
{
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        if (const common::ScanPathConfig* path = cfgMgr->findScanPathById(pathId)) {
            for (const auto& point : path->points) {
                if (point.pointIndex == localIndex) {
                    return point.purpose;
                }
            }
        }
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
        // 厚度 / 长度容积仅机械臂
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
        cloud.purpose = classifyPointPurpose(purposeForLocalIndex(quota.pathId, key.localIndex));
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

/// 内表面源码要求 exactly 2 帧（两端）。采集点多于 2 时取 localIndex 最小/最大作为两端。
bool selectInnerSurfaceTwoEnds(
    const QVector<SegmentCloud>& clouds,
    const SegmentCloud** outFrame1,
    const SegmentCloud** outFrame2,
    int* availableCount,
    QString* detail)
{
    if (outFrame1 == nullptr || outFrame2 == nullptr) {
        return false;
    }
    *outFrame1 = nullptr;
    *outFrame2 = nullptr;

    QVector<const SegmentCloud*> frames;
    for (const SegmentCloud& cloud : clouds) {
        if (cloud.purpose == PointPurposeKind::InnerSurface) {
            frames.push_back(&cloud);
        }
    }
    if (availableCount != nullptr) {
        *availableCount = frames.size();
    }
    if (frames.size() < 2) {
        if (detail != nullptr) {
            *detail = QStringLiteral("inner_surface 点位不足（需两端共 2 帧，当前 %1）")
                          .arg(frames.size());
        }
        return false;
    }

    *outFrame1 = frames.front();
    *outFrame2 = frames.back();
    if (detail != nullptr) {
        if (frames.size() == 2) {
            *detail = QStringLiteral("两端帧 localIndex=%1,%2")
                          .arg((*outFrame1)->localIndex)
                          .arg((*outFrame2)->localIndex);
        } else {
            *detail = QStringLiteral(
                          "源码要求 exactly 2 帧：从 %1 个 inner_surface 点取两端 localIndex=%2,%3")
                          .arg(frames.size())
                          .arg((*outFrame1)->localIndex)
                          .arg((*outFrame2)->localIndex);
        }
    }
    return *outFrame1 != nullptr && *outFrame2 != nullptr && *outFrame1 != *outFrame2;
}

scan_tracking::thickness_measure_v2::ThicknessMeasureV2Service& sharedThicknessMeasureV2Service()
{
    static scan_tracking::thickness_measure_v2::ThicknessMeasureV2Service service;
    return service;
}

scan_tracking::inner_surface_measure::InnerSurfaceMeasureService& sharedInnerSurfaceMeasureService()
{
    static scan_tracking::inner_surface_measure::InnerSurfaceMeasureService service;
    return service;
}

bool ensureThicknessMeasureReady(QString* errorMessage)
{
    auto& service = sharedThicknessMeasureV2Service();
    if (service.isReady()) {
        return true;
    }
    scan_tracking::thickness_measure_v2::ThicknessV2Error error;
    if (!service.initializeFromIni(QString(), &error)) {
        if (errorMessage != nullptr) {
            *errorMessage = error.message + QStringLiteral(
                "（请确认 config/thickness_measure_v3/thickness_measurement.ini、"
                "外模板 PCD 与 models/thickness_measure_v3/*.onnx 可用）");
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
    const SegmentCloud* innerFrame1 = nullptr;
    const SegmentCloud* innerFrame2 = nullptr;
    int innerAvailableCount = 0;
    QString innerEndsDetail;
    const bool hasInnerEnds = selectInnerSurfaceTwoEnds(
        clouds, &innerFrame1, &innerFrame2, &innerAvailableCount, &innerEndsDetail);
    if (thicknessPairs.isEmpty() && !hasInnerEnds) {
        result.resultCode = 3;
        result.ngReasonWord0 = kNgReasonIncompleteSegments;
        result.message = QStringLiteral(
            "pathId=%1：未找到可用的 thickness 配对或内表面两端帧（请检查 scan_paths points[].purpose）。"
            "%2")
                             .arg(quota.pathId)
                             .arg(innerEndsDetail.isEmpty() ? QString()
                                                           : QStringLiteral(" ") + innerEndsDetail);
        return result;
    }

    qInfo(LOG_STATION2_INSPECTION).noquote()
        << QStringLiteral("开始厚度+内表面测量 pathId=") << quota.pathId
        << QStringLiteral(" name=") << quota.pathName
        << QStringLiteral(" 厚度对数=") << thicknessPairs.size()
        << QStringLiteral(" 内表面=")
        << (hasInnerEnds ? innerEndsDetail : QStringLiteral("无两端帧"));

    // --- 厚度 V3：out-of-process worker 加载 ThicknessMeasureV3.dll ---
    // 主进程只组包/收结果；create/measure/destroy 在 thickness-measure-v3-worker.exe。
    // 预处理仅在 DLL 内按 thickness_measurement.ini 执行，IPC 不再二次降采样。
    if (!thicknessPairs.isEmpty()) {
        QString initError;
        if (!ensureThicknessMeasureReady(&initError)) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message = QStringLiteral("pathId=%1 厚度测量(V2)初始化失败：%2")
                                 .arg(quota.pathId)
                                 .arg(initError);
            return result;
        }

        QVector<scan_tracking::thickness_measure_v2::ThicknessV2PairClouds> pairClouds;
        pairClouds.reserve(thicknessPairs.size());
        for (const ThicknessPairRefs& pair : thicknessPairs) {
            scan_tracking::thickness_measure_v2::ThicknessV2PairClouds cloudsView;
            cloudsView.inner.xyz = pair.inner->xyz.data();
            cloudsView.inner.pointCount = static_cast<size_t>(pair.inner->finiteCount);
            cloudsView.outer.xyz = pair.outer->xyz.data();
            cloudsView.outer.pointCount = static_cast<size_t>(pair.outer->finiteCount);
            pairClouds.push_back(cloudsView);
            qInfo(LOG_STATION2_INSPECTION)
                << "thickness V2 pair inner=" << pair.innerIndex
                << "outer=" << pair.outerIndex
                << "innerPts=" << pair.inner->finiteCount
                << "outerPts=" << pair.outer->finiteCount;
        }

        scan_tracking::thickness_measure_v2::ThicknessV2AverageMeasurement average;
        scan_tracking::thickness_measure_v2::ThicknessV2Error error;
        if (!sharedThicknessMeasureV2Service().measurePairsAverage(pairClouds, &average, &error) ||
            !average.valid ||
            average.successCount == 0) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.measurement.thicknessPairCount = static_cast<int>(average.pairCount > 0
                                                                        ? average.pairCount
                                                                        : pairClouds.size());
            result.measurement.thicknessSuccessCount = static_cast<int>(average.successCount);
            result.message = QStringLiteral("pathId=%1 厚度测量(V2)失败：%2（成功对 %3/%4）")
                                 .arg(quota.pathId)
                                 .arg(error.message.isEmpty() ? QStringLiteral("无有效厚度对")
                                                             : error.message)
                                 .arg(static_cast<int>(average.successCount))
                                 .arg(static_cast<int>(average.pairCount > 0 ? average.pairCount
                                                                            : pairClouds.size()));
            return result;
        }

        result.measurement.thicknessMm = average.thicknessMm;
        result.measurement.thicknessPairCount = static_cast<int>(average.pairCount);
        result.measurement.thicknessSuccessCount = static_cast<int>(average.successCount);
        qInfo(LOG_STATION2_INSPECTION)
            << "pathId" << quota.pathId
            << "thicknessV2Mm=" << average.thicknessMm
            << "success=" << average.successCount << "/" << average.pairCount;
    }

    const double measuredLengthMm = measuredLengthForTask(taskId);

    // --- 内表面：严格对照源码（exactly 2 帧 → ism_measure_two_frames_average）---
    if (hasInnerEnds) {
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

        qInfo(LOG_STATION2_INSPECTION).noquote()
            << QStringLiteral("inner_surface ") << innerEndsDetail
            << QStringLiteral(" available=") << innerAvailableCount;

        scan_tracking::inner_surface_measure::InnerSurfaceAverageMeasurement avg;
        scan_tracking::inner_surface_measure::InnerSurfaceFrameMeasurement frame1;
        scan_tracking::inner_surface_measure::InnerSurfaceFrameMeasurement frame2;
        scan_tracking::inner_surface_measure::InnerSurfaceMeasureError error;
        const bool innerMeasureOk = measuredLengthMm > 0.0
            ? sharedInnerSurfaceMeasureService().measureTwoFramesAverageWithLength(
                innerFrame1->xyz.data(),
                static_cast<size_t>(innerFrame1->finiteCount),
                innerFrame2->xyz.data(),
                static_cast<size_t>(innerFrame2->finiteCount),
                measuredLengthMm,
                &avg,
                &frame1,
                &frame2,
                &error)
            : sharedInnerSurfaceMeasureService().measureTwoFramesAverage(
                innerFrame1->xyz.data(),
                static_cast<size_t>(innerFrame1->finiteCount),
                innerFrame2->xyz.data(),
                static_cast<size_t>(innerFrame2->finiteCount),
                &avg,
                &frame1,
                &frame2,
                &error);
        if (!innerMeasureOk || !avg.valid) {
            result.resultCode = 2;
            result.ngReasonWord0 = kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.measurement.innerSurfacePairCount = 1;
            result.measurement.innerSurfaceSuccessCount = 0;
            result.message = QStringLiteral("pathId=%1 内表面测量失败（两端 localIndex=%2,%3）：%4")
                                 .arg(quota.pathId)
                                 .arg(innerFrame1->localIndex)
                                 .arg(innerFrame2->localIndex)
                                 .arg(error.message.isEmpty() ? QStringLiteral("测量无效")
                                                             : error.message);
            return result;
        }

        result.measurement.innerDiameterMm = avg.diameterMm;
        result.measurement.innerCircumferenceMm = avg.circumferenceMm;
        result.measurement.innerRoundness = avg.roundness;
        result.measurement.innerSurfacePairCount = 1;
        result.measurement.innerSurfaceSuccessCount = 1;
        result.measurement.lengthMm = avg.containerLengthMm;
        if (avg.containerLengthMm > 0.0 && avg.diameterMm > 0.0) {
            result.measurement.volumeLiters = avg.volumeLiters;
            result.measurement.volumeRadiusMm = avg.diameterMm * 0.5;
        }
        qInfo(LOG_STATION2_INSPECTION)
            << "pathId" << quota.pathId
            << "inner_surface ends" << innerFrame1->localIndex << innerFrame2->localIndex
            << "diameter=" << avg.diameterMm
            << "circumference=" << avg.circumferenceMm
            << "roundness=" << avg.roundness
            << "volumeL=" << avg.volumeLiters
            << "containerLengthMm=" << avg.containerLengthMm
            << "lengthSource=" << (measuredLengthMm > 0.0 ? "task_length_volume" : "config_fallback")
            << "frame1.valid=" << frame1.valid
            << "frame2.valid=" << frame2.valid;
    }

    result.resultCode = 1;
    result.measureItemCount = static_cast<quint16>(
        (result.measurement.thicknessSuccessCount > 0 ? 1 : 0) +
        (result.measurement.innerSurfaceSuccessCount > 0 ? 1 : 0));
    result.measurement.qualityCode = 1;
    result.measurement.measuredSegmentCount =
        result.measurement.thicknessSuccessCount * 2 +
        (result.measurement.innerSurfaceSuccessCount > 0 ? 2 : 0);
    result.message = QStringLiteral(
        "厚度+内表面通过：pathId=%1 (%2)；厚度=%3mm（%4/%5 对），"
                         "内径=%6mm 周长=%7mm 圆度=%8 容积=%9L 长度=%11mm（两端 %10，长度来源=%12）")
                         .arg(quota.pathId)
                         .arg(quota.pathName.isEmpty() ? QStringLiteral("thickness_inner_surface")
                                                      : quota.pathName)
                         .arg(result.measurement.thicknessMm, 0, 'f', 3)
                         .arg(result.measurement.thicknessSuccessCount)
                         .arg(result.measurement.thicknessPairCount)
                         .arg(result.measurement.innerDiameterMm, 0, 'f', 3)
                         .arg(result.measurement.innerCircumferenceMm, 0, 'f', 3)
                         .arg(result.measurement.innerRoundness, 0, 'f', 3)
                         .arg(result.measurement.volumeLiters, 0, 'f', 3)
                         .arg(hasInnerEnds ? QStringLiteral("%1/%2")
                                                .arg(innerFrame1->localIndex)
                                                .arg(innerFrame2->localIndex)
                                          : QStringLiteral("-"))
                         .arg(result.measurement.lengthMm, 0, 'f', 3)
                         .arg(measuredLengthMm > 0.0
                                  ? QStringLiteral("task_length_volume")
                                  : QStringLiteral("config_fallback"));
    return result;
}

ContainerTotalLengthService& sharedContainerTotalLengthService()
{
    static ContainerTotalLengthService service;
    return service;
}

bool ensureContainerTotalLengthReady(QString* errorMessage)
{
    auto& service = sharedContainerTotalLengthService();
    if (service.isReady()) {
        return true;
    }

    ContainerTotalLengthError error;
    if (!service.initializeFromIni(QString(), &error)) {
        if (errorMessage != nullptr) {
            *errorMessage = error.message;
        }
        return false;
    }
    return true;
}

double resolveVolumeRadiusMm(const InspectionQuota& quota)
{
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        if (const common::ScanPathConfig* path = cfgMgr->findScanPathById(quota.pathId)) {
            if (path->volumeRadiusMm > 0.0) {
                return path->volumeRadiusMm;
            }
        }
    }
    // 与 smoke 默认一致；现场应在 scan_paths JSON 配置 volumeRadiusMm（通常=内径/2）
    return 600.0;
}

/// path3 专用：校验 LB 后把各臂段有限点直接写入 mergedXyz，避免「每段临时向量 → 再合并」二次拷贝。
/// 段顺序与 loadQuotaSegmentClouds 一致（按 localIndex 升序），保证送入 DLL 的点序不变。
bool mergeArmLengthVolumeClouds(
    const ScanSegmentCache& cache,
    const InspectionQuota& quota,
    std::vector<float>* mergedXyz,
    size_t* mergedCount,
    int* segmentCount,
    InspectionResult* failResult)
{
    if (mergedXyz == nullptr || mergedCount == nullptr || segmentCount == nullptr ||
        failResult == nullptr) {
        return false;
    }

    mergedXyz->clear();
    *mergedCount = 0;
    *segmentCount = 0;

    QVector<int> localIndices;
    localIndices.reserve(quota.expectedArmCount > 0 ? quota.expectedArmCount : 8);
    size_t reserveFloats = 0;
    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        if (!isWithinDeviceQuota(key.device, key.localIndex, quota) ||
            key.device != common::ScanDeviceKind::Arm) {
            continue;
        }
        localIndices.push_back(key.localIndex);
        if (const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex)) {
            const auto& frame = entry->bundle.mechEyeResult.pointCloud;
            if (frame.isValid() && frame.pointCount > 0) {
                reserveFloats += static_cast<size_t>(frame.pointCount) * 3u;
            }
        }
    }
    std::sort(localIndices.begin(), localIndices.end());
    if (localIndices.isEmpty()) {
        failResult->resultCode = 3;
        failResult->ngReasonWord0 = kNgReasonIncompleteSegments;
        failResult->message =
            QStringLiteral("pathId=%1：无可用机械臂点云段。").arg(quota.pathId);
        return false;
    }

    mergedXyz->reserve(reserveFloats);

    for (const int localIndex : localIndices) {
        const ScanSegmentCacheEntry* entry =
            cache.entry(common::ScanDeviceKind::Arm, localIndex);
        if (entry == nullptr || !entry->bundle.success()) {
            failResult->resultCode = 2;
            failResult->ngReasonWord0 = kNgReasonBundleInvalid;
            failResult->measurement.qualityCode = 2;
            failResult->measureItemCount = 1;
            failResult->message =
                QStringLiteral("pathId=%1 段 %2 采集数据无效，无法检测。")
                    .arg(quota.pathId)
                    .arg(localIndex);
            return false;
        }

        // 源码要求单一统一坐标系外表面云：CXP 参与段必须已用 LB Rt_global 变换后再合并。
        const auto& bundle = entry->bundle;
        if (bundle.cxpParticipated()) {
            const auto& lb = bundle.lbPoseResult;
            if (!lb.invoked || !lb.success || !lb.poseMatrix.valid) {
                failResult->resultCode = 2;
                failResult->ngReasonWord0 = kNgReasonPointCloudInvalid;
                failResult->measurement.qualityCode = 2;
                failResult->measureItemCount = 1;
                failResult->message = QStringLiteral(
                                         "pathId=%1 段 %2：CXP 已参与但 LB 位姿无效，无法按源码合并外表面点云（%3）")
                                         .arg(quota.pathId)
                                         .arg(localIndex)
                                         .arg(lb.message.isEmpty() ? QStringLiteral("lb missing")
                                                                  : lb.message);
                return false;
            }
        }

        int finiteCount = 0;
        if (!appendFiniteXyz(bundle.mechEyeResult.pointCloud, mergedXyz, &finiteCount)) {
            failResult->resultCode = 2;
            failResult->ngReasonWord0 = kNgReasonPointCloudInvalid;
            failResult->measurement.qualityCode = 2;
            failResult->measureItemCount = 1;
            failResult->message =
                QStringLiteral("pathId=%1 段 %2 点云无效或无可测点。")
                    .arg(quota.pathId)
                    .arg(localIndex);
            return false;
        }
        *mergedCount += static_cast<size_t>(finiteCount);
        ++(*segmentCount);
    }

    if (*mergedCount == 0 || mergedXyz->empty()) {
        failResult->resultCode = 2;
        failResult->ngReasonWord0 = kNgReasonPointCloudInvalid;
        failResult->measurement.qualityCode = 2;
        failResult->measureItemCount = 1;
        failResult->message = QStringLiteral("pathId=%1：合并点云为空。").arg(quota.pathId);
        return false;
    }
    return true;
}

InspectionResult evaluateLengthVolumeInspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota)
{
    InspectionResult result;
    fillPathMeta(&result, quota);
    result.sourcePointCount = quota.total() > 0 ? quota.total() : cache.cachedSegmentCount();

    if (cache.cachedSegmentCount() == 0) {
        result.resultCode = 3;
        result.message = QStringLiteral("pathId=%1：无扫描段缓存，请先完成机械臂外表面采集。")
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

    std::vector<float> mergedXyz;
    size_t mergedCount = 0;
    int segmentCount = 0;
    if (!mergeArmLengthVolumeClouds(
            cache, quota, &mergedXyz, &mergedCount, &segmentCount, &result)) {
        fillPathMeta(&result, quota);
        return result;
    }

    QString initError;
    if (!ensureContainerTotalLengthReady(&initError)) {
        result.resultCode = 2;
        result.ngReasonWord0 = kNgReasonAlgorithmFailed;
        result.measurement.qualityCode = 2;
        result.measureItemCount = 1;
        result.message = QStringLiteral(
                             "pathId=%1 筒体总长测量初始化失败：%2"
                             "（请确认 config/container_total_length/config.ini 与 Data/sample_cylinder.pcd）")
                             .arg(quota.pathId)
                             .arg(initError);
        return result;
    }

    // 对照源码：ctl_create_from_ini + 一次 ctl_measure(合并外表面云)；容积由 IPC 用 πr²L 补算。
    const double volumeRadiusMm = resolveVolumeRadiusMm(quota);
    qInfo(LOG_STATION2_INSPECTION).noquote()
        << QStringLiteral("开始筒体总长测量 pathId=") << quota.pathId
        << QStringLiteral(" name=") << quota.pathName
        << QStringLiteral(" 段数=") << segmentCount
        << QStringLiteral(" 合并点数=") << static_cast<qulonglong>(mergedCount)
        << QStringLiteral(" volumeRadiusMm=") << volumeRadiusMm;

    ContainerTotalLengthMeasurement measurement;
    ContainerTotalLengthError error;
    if (!sharedContainerTotalLengthService().measure(
            mergedXyz.data(), mergedCount, &measurement, &error) ||
        !measurement.valid) {
        result.resultCode = 2;
        result.ngReasonWord0 = kNgReasonAlgorithmFailed;
        result.measurement.qualityCode = 2;
        result.measureItemCount = 1;
        result.message = QStringLiteral("pathId=%1 筒体总长测量失败：%2")
                             .arg(quota.pathId)
                             .arg(error.message.isEmpty() ? QStringLiteral("测量无效")
                                                         : error.message);
        return result;
    }

    const double volumeLiters =
        kPi * volumeRadiusMm * volumeRadiusMm * measurement.lengthMm / 1.0e6;

    result.resultCode = 1;
    result.measureItemCount = 2;
    result.measurement.qualityCode = 1;
    result.measurement.measuredSegmentCount = segmentCount;
    result.measurement.lengthMm = measurement.lengthMm;
    rememberMeasuredLength(taskId, measurement.lengthMm);
    result.measurement.volumeLiters = volumeLiters;
    result.measurement.volumeRadiusMm = volumeRadiusMm;
    result.measurement.fittedOuterRadiusMm = static_cast<double>(measurement.fittedRadiusMm);
    result.measurement.containerLeftEndPositionMm = measurement.leftEndPosition;
    result.measurement.containerRightEndPositionMm = measurement.rightEndPosition;
    result.measurement.containerIcpFitness = measurement.icpFitness;
    result.measurement.containerFittedRadiusMm = static_cast<double>(measurement.fittedRadiusMm);
    result.measurement.containerIcpConverged = measurement.icpConverged;
    result.message = QStringLiteral(
                         "pathId=%1 筒体总长 OK：length=%2 mm, volume=%3 L, "
                         "radius=%4 mm, fittedOuter=%5 mm, segments=%6")
                         .arg(quota.pathId)
                         .arg(measurement.lengthMm, 0, 'f', 3)
                         .arg(volumeLiters, 0, 'f', 3)
                         .arg(volumeRadiusMm, 0, 'f', 3)
                         .arg(measurement.fittedRadiusMm, 0, 'f', 3)
                         .arg(segmentCount);
    qInfo(LOG_STATION2_INSPECTION).noquote() << result.message;
    return result;
}

InspectionResult evaluateStation2InspectionUnlocked(
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

    if (!isAlgorithmSolveEnabled()) {
        InspectionResult result = makeAlgorithmDisabledOkResult(effective);
        result.sourcePointCount =
            effective.total() > 0 ? effective.total() : cache.cachedSegmentCount();
        qInfo(LOG_STATION2_INSPECTION).noquote() << result.message
            << QStringLiteral(" pathId=") << result.pathId
            << QStringLiteral(" algorithm=") << result.algorithm;
        return result;
    }

    if (effective.algorithm == QLatin1String("weld_section")) {
        return evaluateWeldSectionInspection(cache, taskId, effective);
    }
    if (effective.algorithm == QLatin1String("thickness_inner_surface")) {
        return evaluateThicknessInnerSurfaceInspection(cache, taskId, effective);
    }
    if (effective.algorithm == QLatin1String("length_volume")) {
        return evaluateLengthVolumeInspection(cache, taskId, effective);
    }
    if (effective.algorithm == QLatin1String("self_check")) {
        InspectionResult result;
        result.pathId = effective.pathId;
        result.pathName = effective.pathName;
        result.algorithm = effective.algorithm;
        result.resultCode = 3;
        result.message = QStringLiteral(
            "pathId=%1 self_check 由 Trig_SelfCheck 完成（回零校验 + 臂 3D/CXP 采集），"
            "不走 Inspection。")
                             .arg(effective.pathId);
        return result;
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

InspectionResult makeEvaluateBusyResult(const InspectionQuota& quota)
{
    InspectionResult result;
    fillPathMeta(&result, quota);
    result.resultCode = 3;
    result.message = QStringLiteral(
        "算法解算繁忙（后台任务进行中），请稍后复测");
    return result;
}

std::mutex& station2EvaluateMutex()
{
    static std::mutex mutex;
    return mutex;
}

}  // namespace

bool prewarmActiveStation2InspectionAlgorithm(QString* errorMessage)
{
    const auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr || !cfgMgr->flowControlConfig().algorithmEnabled) {
        return true;
    }

    const QString algorithm = cfgMgr->activePathAlgorithm().trimmed();
    const int pathId = cfgMgr->activePathId();
    const QString pathName = cfgMgr->activePathName();

    if (algorithm == QLatin1String("thickness_inner_surface")) {
        QString detail;
        qInfo(LOG_STATION2_INSPECTION).noquote()
            << QStringLiteral("启动期预热厚度V2+内表面 pathId=") << pathId
            << QStringLiteral(" name=") << pathName;
        if (!ensureThicknessMeasureReady(&detail)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("厚度V2算法预热失败：%1").arg(detail);
            }
            return false;
        }
        if (!ensureInnerSurfaceMeasureReady(&detail)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("内表面算法预热失败：%1").arg(detail);
            }
            return false;
        }
        qInfo(LOG_STATION2_INSPECTION).noquote()
            << QStringLiteral("启动期厚度V2+内表面预热完成 pathId=") << pathId;
        return true;
    }

    if (algorithm != QLatin1String("weld_section")) {
        return true;
    }

    const bool ringWeld = pathId == 5 ||
        pathName.compare(QStringLiteral("ring_weld"), Qt::CaseInsensitive) == 0;

    QString detail;
    qInfo(LOG_STATION2_INSPECTION).noquote()
        << QStringLiteral("启动期预热焊缝算法 pathId=") << pathId
        << QStringLiteral(" name=") << pathName
        << QStringLiteral(" arm=") << cfgMgr->enabledArmPointCount()
        << QStringLiteral(" telescopic=") << cfgMgr->enabledTelescopicPointCount();

    if (cfgMgr->enabledArmPointCount() > 0 &&
        !ensureWeldMeasureReadyForDevice(common::ScanDeviceKind::Arm, ringWeld, &detail)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("机械臂焊缝算法预热失败：%1").arg(detail);
        }
        return false;
    }
    if (cfgMgr->enabledTelescopicPointCount() > 0 &&
        !ensureWeldMeasureReadyForDevice(
            common::ScanDeviceKind::Telescopic, ringWeld, &detail)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("伸缩杆焊缝算法预热失败：%1").arg(detail);
        }
        return false;
    }

    qInfo(LOG_STATION2_INSPECTION).noquote()
        << QStringLiteral("启动期焊缝算法预热完成 pathId=") << pathId;
    return true;
}

void InspectionCloudSnapshot::clear()
{
    runTaskId = 0;
    runCaptureRoot.clear();
    segments.clear();
}

int InspectionCloudSnapshot::countForDevice(common::ScanDeviceKind device) const
{
    int count = 0;
    for (const InspectionSegmentCloud& segment : segments) {
        if (segment.device == device) {
            ++count;
        }
    }
    return count;
}

bool InspectionCloudSnapshot::meetsDeviceQuotas(
    int expectedArmCount,
    int expectedTelescopicCount) const
{
    if (expectedArmCount > 0 && countForDevice(common::ScanDeviceKind::Arm) < expectedArmCount) {
        return false;
    }
    if (expectedTelescopicCount > 0 &&
        countForDevice(common::ScanDeviceKind::Telescopic) < expectedTelescopicCount) {
        return false;
    }
    return segmentCount() > 0;
}

const InspectionSegmentCloud* InspectionCloudSnapshot::find(
    common::ScanDeviceKind device,
    int localIndex) const
{
    for (const InspectionSegmentCloud& segment : segments) {
        if (segment.device == device && segment.localIndex == localIndex) {
            return &segment;
        }
    }
    return nullptr;
}

InspectionCloudSnapshot buildInspectionCloudSnapshot(const ScanSegmentCache& cache)
{
    InspectionCloudSnapshot snapshot;
    snapshot.runTaskId = cache.runTaskId();
    snapshot.runCaptureRoot = cache.runCaptureRoot();
    snapshot.segments.reserve(cache.cachedSegmentCount());

    for (const ScanSegmentCacheKey& key : cache.cachedKeys()) {
        const ScanSegmentCacheEntry* entry = cache.entry(key.device, key.localIndex);
        if (entry == nullptr) {
            continue;
        }

        InspectionSegmentCloud segment;
        segment.device = key.device;
        segment.localIndex = key.localIndex;
        segment.captureOk = entry->bundle.success();
        segment.cxpParticipated = entry->bundle.cxpParticipated();
        const auto& lb = entry->bundle.lbPoseResult;
        segment.lbPoseOk = lb.invoked && lb.success && lb.poseMatrix.valid;

        if (segment.captureOk) {
            const auto& frame = entry->bundle.mechEyeResult.pointCloud;
            const int bufferPointCount = frame.pointsXYZ
                ? static_cast<int>(frame.pointsXYZ->size() / 3u)
                : 0;
            segment.pointCount = std::min(frame.pointCount, bufferPointCount);
            if (segment.pointCount <= 0) {
                segment.captureOk = false;
                segment.pointCount = 0;
            } else {
                // PointCloudFrame 本身就是 shared_ptr 大缓冲；快照只共享所有权。
                // 有限值过滤仍在 evaluate* 的逐帧 extractFiniteXyz 中完成。
                segment.xyz = frame.pointsXYZ;
            }
        }

        snapshot.segments.push_back(std::move(segment));
    }

    std::sort(
        snapshot.segments.begin(),
        snapshot.segments.end(),
        [](const InspectionSegmentCloud& a, const InspectionSegmentCloud& b) {
            if (a.device != b.device) {
                return static_cast<int>(a.device) < static_cast<int>(b.device);
            }
            return a.localIndex < b.localIndex;
        });

    qInfo(LOG_STATION2_INSPECTION).noquote()
        << QStringLiteral("已构建检测轻量快照 segments=") << snapshot.segmentCount()
        << QStringLiteral(" taskId=") << snapshot.runTaskId
        << QStringLiteral(" runRoot=") << snapshot.runCaptureRoot
        << QStringLiteral(" arm=") << snapshot.countForDevice(common::ScanDeviceKind::Arm)
        << QStringLiteral(" telescopic=")
        << snapshot.countForDevice(common::ScanDeviceKind::Telescopic);
    return snapshot;
}

bool buildInspectionSegmentCloud(
    const ScanSegmentCache& cache,
    common::ScanDeviceKind device,
    int localIndex,
    InspectionSegmentCloud* out)
{
    if (out == nullptr) {
        return false;
    }
    const ScanSegmentCacheEntry* entry = cache.entry(device, localIndex);
    if (entry == nullptr) {
        return false;
    }

    InspectionSegmentCloud segment;
    segment.device = device;
    segment.localIndex = localIndex;
    segment.captureOk = entry->bundle.success();
    segment.cxpParticipated = entry->bundle.cxpParticipated();
    const auto& lb = entry->bundle.lbPoseResult;
    segment.lbPoseOk = lb.invoked && lb.success && lb.poseMatrix.valid;
    if (segment.captureOk) {
        const auto& frame = entry->bundle.mechEyeResult.pointCloud;
        const int bufferPointCount = frame.pointsXYZ
            ? static_cast<int>(frame.pointsXYZ->size() / 3u)
            : 0;
        segment.pointCount = std::min(frame.pointCount, bufferPointCount);
        if (segment.pointCount <= 0) {
            segment.captureOk = false;
            segment.pointCount = 0;
        } else {
            segment.xyz = frame.pointsXYZ;
        }
    }
    *out = std::move(segment);
    return true;
}

IncrementalWeldSegmentResult evaluateWeldSectionSegment(
    const InspectionSegmentCloud& segment,
    int pathId,
    bool ringWeld)
{
    IncrementalWeldSegmentResult result;
    result.device = segment.device;
    result.localIndex = segment.localIndex;
    QElapsedTimer timer;
    timer.start();

    if (!segment.captureOk || !segment.xyz || segment.pointCount <= 0) {
        result.errorCode = 1;
        result.errorMessage = QStringLiteral("点云无效或无可测点。");
        result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
        return result;
    }

    QString initError;
    if (!ensureWeldMeasureReadyForDevice(segment.device, ringWeld, &initError)) {
        result.errorCode = 2;
        result.errorMessage = initError;
        result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
        return result;
    }

    scan_tracking::mech_eye::PointCloudFrame cloud;
    cloud.pointsXYZ = segment.xyz;
    cloud.pointCount = segment.pointCount;
    cloud.width = segment.pointCount;
    cloud.height = 1;
    std::vector<float> xyz;
    int finiteCount = 0;
    if (!extractFiniteXyz(cloud, &xyz, &finiteCount)) {
        result.errorCode = 1;
        result.errorMessage = QStringLiteral("点云无效或无可测点。");
        result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
        return result;
    }

    weld_measure::WeldMeasureError error;
    qInfo(LOG_STATION2_INSPECTION)
        << "pathId" << pathId
        << common::ConfigManager::scanDeviceKindToString(segment.device)
        << "localIndex" << segment.localIndex
        << "incremental measureFrame begin fedPoints=" << finiteCount;
    auto& service = sharedWeldMeasureService(segment.device, ringWeld);
    if (!service.measureFrame(
            segment.localIndex,
            xyz.data(),
            static_cast<size_t>(finiteCount),
            &result.frame,
            &error)) {
        result.errorCode = 2;
        result.errorMessage = error.message;
        result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
        return result;
    }

    result.success = true;
    result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
    qInfo(LOG_STATION2_INSPECTION)
        << "pathId" << pathId
        << common::ConfigManager::scanDeviceKindToString(segment.device)
        << "localIndex" << segment.localIndex
        << "incremental measureFrame done elapsedSec=" << result.elapsedSeconds
        << "validSections=" << result.frame.validSections << "/" << result.frame.totalSections;
    return result;
}

InspectionResult aggregateWeldSectionSegments(
    const std::vector<IncrementalWeldSegmentResult>& segments,
    const InspectionQuota& quota,
    double wallElapsedSeconds)
{
    InspectionResult result;
    fillPathMeta(&result, quota);
    result.sourcePointCount = quota.total() > 0
        ? quota.total()
        : static_cast<int>(segments.size());
    result.elapsedSeconds = wallElapsedSeconds;

    std::vector<IncrementalWeldSegmentResult> ordered = segments;
    std::sort(
        ordered.begin(), ordered.end(),
        [](const IncrementalWeldSegmentResult& a, const IncrementalWeldSegmentResult& b) {
            if (a.device != b.device) {
                return static_cast<int>(a.device) < static_cast<int>(b.device);
            }
            return a.localIndex < b.localIndex;
        });

    int armCount = 0;
    int telescopicCount = 0;
    for (const IncrementalWeldSegmentResult& segment : ordered) {
        if (segment.device == common::ScanDeviceKind::Arm) {
            ++armCount;
        } else {
            ++telescopicCount;
        }
    }
    if (armCount < quota.expectedArmCount || telescopicCount < quota.expectedTelescopicCount) {
        result.resultCode = 3;
        result.ngReasonWord0 = kNgReasonIncompleteSegments;
        result.message = incompleteQuotaMessage(armCount, telescopicCount, quota);
        return result;
    }

    int measuredOk = 0;
    for (const IncrementalWeldSegmentResult& segment : ordered) {
        if (!isWithinDeviceQuota(segment.device, segment.localIndex, quota)) {
            continue;
        }
        if (!segment.success) {
            result.resultCode = 2;
            result.ngReasonWord0 = segment.errorCode == 1
                ? kNgReasonPointCloudInvalid
                : kNgReasonAlgorithmFailed;
            result.measurement.qualityCode = 2;
            result.measureItemCount = 1;
            result.message = segment.errorCode == 1
                ? QStringLiteral("pathId=%1 %2 段 %3 点云无效或无可测点。")
                      .arg(quota.pathId)
                      .arg(common::ConfigManager::scanDeviceKindToString(segment.device))
                      .arg(segment.localIndex)
                : QStringLiteral("pathId=%1 %2 段 %3 焊缝测量失败：%4")
                      .arg(quota.pathId)
                      .arg(common::ConfigManager::scanDeviceKindToString(segment.device))
                      .arg(segment.localIndex)
                      .arg(segment.errorMessage);
            return result;
        }
        accumulateMeasurement(&result.measurement, segment.frame);
        ++measuredOk;
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
                         .arg(armCount)
                         .arg(quota.expectedArmCount)
                         .arg(telescopicCount)
                         .arg(quota.expectedTelescopicCount)
                         .arg(result.measurement.mismatchMm, 0, 'f', 3)
                         .arg(result.measurement.reinforcementMm, 0, 'f', 3)
                         .arg(result.measurement.angularityMm, 0, 'f', 3)
                         .arg(result.measurement.maxUndercutMm, 0, 'f', 3);
    return result;
}

/// 将轻量快照物化为仅含有限 XYZ 的临时段缓存，供既有 evaluate* 复用。
ScanSegmentCache materializeInspectionCache(const InspectionCloudSnapshot& snapshot)
{
    ScanSegmentCache cache;
    // 复用主缓存 run 根；prepareRunRoot=false，禁止再 mkdir 出新的 run_0_*。
    cache.bindExistingRunRoot(snapshot.runTaskId, snapshot.runCaptureRoot);

    for (const InspectionSegmentCloud& segment : snapshot.segments) {
        vision::MultiCameraCaptureBundle bundle;
        // 快照已持有共享 XYZ，CXP 图像帧不再需要；保留采集成功状态。
        bundle.heavyPayloadsStripped = true;
        if (segment.captureOk) {
            bundle.mechEyeResult.errorCode = mech_eye::CaptureErrorCode::Success;
        } else {
            bundle.mechEyeResult.errorCode = mech_eye::CaptureErrorCode::CaptureFailed;
            bundle.mechEyeResult.errorMessage = QStringLiteral("snapshot captureOk=false");
        }

        if (segment.xyz && !segment.xyz->empty() && segment.pointCount > 0) {
            bundle.mechEyeResult.pointCloud.pointsXYZ = segment.xyz;
            bundle.mechEyeResult.pointCloud.pointCount = segment.pointCount;
            bundle.mechEyeResult.pointCloud.width = segment.pointCount;
            bundle.mechEyeResult.pointCloud.height = 1;
        }

        if (segment.cxpParticipated) {
            // cxpParticipated() 依据 request 键非空。
            bundle.request.hikCameraAKey = QStringLiteral("snapshot-cxp-a");
            bundle.request.hikCameraBKey = QStringLiteral("snapshot-cxp-b");
            bundle.hikCameraAResult.errorCode = vision::VisionErrorCode::Success;
            bundle.hikCameraBResult.errorCode = vision::VisionErrorCode::Success;
            bundle.lbPoseResult.invoked = true;
            bundle.lbPoseResult.success = segment.lbPoseOk;
            bundle.lbPoseResult.poseMatrix.valid = segment.lbPoseOk;
            if (!segment.lbPoseOk) {
                bundle.lbPoseResult.message = QStringLiteral("snapshot lbPoseOk=false");
            }
        }

        cache.storeSegment(
            segment.device,
            segment.localIndex,
            snapshot.runTaskId,
            std::move(bundle),
            /*prepareRunRoot=*/false);
    }
    return cache;
}

InspectionResult evaluateStation2Inspection(
    const InspectionCloudSnapshot& snapshot,
    quint32 taskId,
    const InspectionQuota& quota)
{
    // 关闭算法时禁止物化大快照，避免联调主流程时仍占数百 MB / 调 DLL。
    if (!isAlgorithmSolveEnabled()) {
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
        InspectionResult result = makeAlgorithmDisabledOkResult(effective);
        result.sourcePointCount = snapshot.segmentCount();
        qInfo(LOG_STATION2_INSPECTION).noquote() << result.message
            << QStringLiteral(" pathId=") << result.pathId
            << QStringLiteral(" algorithm=") << result.algorithm;
        return result;
    }

    // 物化仅含有限 XYZ 的临时缓存；原全量 PointCloudFrame/纹理/CXP 不再进入后台解算持有。
    const ScanSegmentCache cache = materializeInspectionCache(snapshot);
    std::lock_guard<std::mutex> lock(station2EvaluateMutex());
    QElapsedTimer timer;
    timer.start();
    InspectionResult result = evaluateStation2InspectionUnlocked(cache, taskId, quota);
    result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
    return result;
}

InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota)
{
    // 共享算法 Service 非可重入整路径状态；后台多路径并发时串行化整次评估。
    std::lock_guard<std::mutex> lock(station2EvaluateMutex());
    QElapsedTimer timer;
    timer.start();
    InspectionResult result = evaluateStation2InspectionUnlocked(cache, taskId, quota);
    result.elapsedSeconds = timer.nsecsElapsed() / 1e9;
    return result;
}

bool tryEvaluateStation2Inspection(
    const InspectionCloudSnapshot& snapshot,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out)
{
    if (out == nullptr) {
        return false;
    }
    if (!isAlgorithmSolveEnabled()) {
        *out = evaluateStation2Inspection(snapshot, taskId, quota);
        out->elapsedSeconds = 0.0;
        return true;
    }
    std::unique_lock<std::mutex> lock(station2EvaluateMutex(), std::try_to_lock);
    if (!lock.owns_lock()) {
        *out = makeEvaluateBusyResult(quota);
        out->elapsedSeconds = 0.0;
        return false;
    }
    const ScanSegmentCache cache = materializeInspectionCache(snapshot);
    QElapsedTimer timer;
    timer.start();
    *out = evaluateStation2InspectionUnlocked(cache, taskId, quota);
    out->elapsedSeconds = timer.nsecsElapsed() / 1e9;
    return true;
}

bool tryEvaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out)
{
    if (out == nullptr) {
        return false;
    }
    std::unique_lock<std::mutex> lock(station2EvaluateMutex(), std::try_to_lock);
    if (!lock.owns_lock()) {
        *out = makeEvaluateBusyResult(quota);
        out->elapsedSeconds = 0.0;
        return false;
    }
    QElapsedTimer timer;
    timer.start();
    *out = evaluateStation2InspectionUnlocked(cache, taskId, quota);
    out->elapsedSeconds = timer.nsecsElapsed() / 1e9;
    return true;
}

}  // namespace scan_tracking::flow_control
