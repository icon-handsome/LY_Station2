#include "scan_tracking/flow_control/scan_segment_cache.h"

#include "scan_tracking/common/capture_cache_paths.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/vision/hik_mono_io.h"

#include <QtCore/QLoggingCategory>

#include <algorithm>

Q_LOGGING_CATEGORY(LOG_SCAN_CACHE, "flow_control.scan_cache")

namespace scan_tracking::flow_control {

void ScanSegmentCache::reset()
{
    m_entries.clear();
    m_runTaskId = 0;
    m_runCaptureRoot.clear();
    m_runTimestamp.clear();
}

bool ScanSegmentCache::ensureRunRoot(quint32 taskId, QString* runRootOut, QString* timestampOut)
{
    if (taskId == 0) {
        if (runRootOut != nullptr) {
            *runRootOut = QString();
        }
        if (timestampOut != nullptr) {
            *timestampOut = QString();
        }
        return false;
    }

    if (m_runTaskId != taskId) {
        m_entries.clear();
        m_runTaskId = taskId;
        m_runTimestamp = scan_tracking::common::buildCaptureTimestamp();
        m_runCaptureRoot = scan_tracking::common::buildRunCaptureRoot(taskId, m_runTimestamp);
    }

    if (m_runCaptureRoot.isEmpty()) {
        return false;
    }

    if (runRootOut != nullptr) {
        *runRootOut = m_runCaptureRoot;
    }
    if (timestampOut != nullptr) {
        *timestampOut = m_runTimestamp;
    }
    return true;
}

void ScanSegmentCache::storeSegment(
    int segmentIndex,
    quint32 taskId,
    vision::MultiCameraCaptureBundle bundle)
{
    ensureRunRoot(taskId);

    ScanSegmentCacheEntry entry;
    entry.segmentIndex = segmentIndex;
    entry.taskId = taskId;
    entry.runCaptureRoot = m_runCaptureRoot;
    entry.captureTimestamp = m_runTimestamp;
    entry.bundle = std::move(bundle);
    m_entries.insert(segmentIndex, entry);
}

bool ScanSegmentCache::persistSegment(int segmentIndex, QString* errorMessage)
{
    const auto iterator = m_entries.find(segmentIndex);
    if (iterator == m_entries.end()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("段 %1 不在缓存中。").arg(segmentIndex);
        }
        return false;
    }

    return persistScanSegmentBundle(
        iterator->runCaptureRoot,
        iterator->segmentIndex,
        iterator->taskId,
        iterator->captureTimestamp,
        iterator->bundle,
        errorMessage);
}

const ScanSegmentCacheEntry* ScanSegmentCache::entry(int segmentIndex) const
{
    const auto iterator = m_entries.constFind(segmentIndex);
    if (iterator == m_entries.constEnd()) {
        return nullptr;
    }
    return &(*iterator);
}

QVector<int> ScanSegmentCache::cachedSegmentIndices() const
{
    QVector<int> indices;
    indices.reserve(m_entries.size());
    for (auto iterator = m_entries.constBegin(); iterator != m_entries.constEnd(); ++iterator) {
        indices.push_back(iterator.key());
    }
    std::sort(indices.begin(), indices.end());
    return indices;
}

bool ScanSegmentCache::allCachedBundlesSuccessful() const
{
    for (auto iterator = m_entries.constBegin(); iterator != m_entries.constEnd(); ++iterator) {
        if (!iterator->bundle.success()) {
            return false;
        }
    }
    return !m_entries.isEmpty();
}

int ScanSegmentCache::cachedSegmentCount() const
{
    return m_entries.size();
}

bool persistScanSegmentBundle(
    const QString& runRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp,
    const vision::MultiCameraCaptureBundle& bundle,
    QString* errorMessage)
{
    if (runRoot.trimmed().isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("run 落盘根目录为空。");
        }
        return false;
    }

    bool ok = true;
    QString firstError;

    auto recordFailure = [&](const QString& message) {
        ok = false;
        if (firstError.isEmpty()) {
            firstError = message;
        }
    };

    if (bundle.mechEyeResult.pointCloud.isValid()) {
        const QString plyPath = scan_tracking::mech_eye::buildSegmentPlyPath(
            runRoot, segmentIndex, taskId, timestamp);
        if (plyPath.isEmpty() ||
            !scan_tracking::mech_eye::savePointCloudFrameToPly(
                bundle.mechEyeResult.pointCloud, plyPath)) {
            recordFailure(QStringLiteral("Mech-Eye 点云落盘失败：段 %1").arg(segmentIndex));
        }
    }

    if (bundle.mechEyeResult.texture2D.isValid()) {
        const QString pngPath = scan_tracking::mech_eye::buildSegmentMech2DPngPath(
            runRoot, segmentIndex, taskId, timestamp);
        if (pngPath.isEmpty() ||
            !scan_tracking::mech_eye::saveGrayTextureFrameToPng(
                bundle.mechEyeResult.texture2D, pngPath)) {
            recordFailure(QStringLiteral("Mech-Eye 2D 落盘失败：段 %1").arg(segmentIndex));
        }
    }

    if (bundle.hikCameraAResult.success()) {
        const QString bmpPath = scan_tracking::vision::buildSegmentHikMonoPath(
            runRoot, segmentIndex, taskId, QStringLiteral("hikA"), timestamp);
        if (bmpPath.isEmpty() ||
            !scan_tracking::vision::saveHikMonoFrameToBmp(
                bundle.hikCameraAResult.frame, bmpPath)) {
            recordFailure(QStringLiteral("海康 A 落盘失败：段 %1").arg(segmentIndex));
        }
    }

    if (bundle.hikCameraBResult.success()) {
        const QString bmpPath = scan_tracking::vision::buildSegmentHikMonoPath(
            runRoot, segmentIndex, taskId, QStringLiteral("hikB"), timestamp);
        if (bmpPath.isEmpty() ||
            !scan_tracking::vision::saveHikMonoFrameToBmp(
                bundle.hikCameraBResult.frame, bmpPath)) {
            recordFailure(QStringLiteral("海康 B 落盘失败：段 %1").arg(segmentIndex));
        }
    }

    if (!ok) {
        if (errorMessage != nullptr) {
            *errorMessage = firstError;
        }
        qWarning(LOG_SCAN_CACHE).noquote()
            << QStringLiteral("段 %1 落盘部分失败：").arg(segmentIndex) << firstError;
        return false;
    }

    qInfo(LOG_SCAN_CACHE).noquote()
        << QStringLiteral("段 %1 已落盘至 %2").arg(segmentIndex).arg(runRoot);
    return true;
}

}  // namespace scan_tracking::flow_control
