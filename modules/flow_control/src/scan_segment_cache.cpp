#include "scan_tracking/flow_control/scan_segment_cache.h"

#include "scan_tracking/common/capture_cache_paths.h"
#include "scan_tracking/orbbec_gemini/orbbec_capture_io.h"

#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>

#include <algorithm>

Q_LOGGING_CATEGORY(LOG_SCAN_CACHE, "flow_control.scan_cache")

namespace scan_tracking::flow_control {

namespace {

bool copyCaptureFile(const QString& sourcePath, const QString& destPath, QString* errorMessage)
{
    if (sourcePath.trimmed().isEmpty() || destPath.trimmed().isEmpty()) {
        return false;
    }
    if (QFile::exists(destPath)) {
        if (!QFile::remove(destPath)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("无法覆盖已有文件：%1").arg(destPath);
            }
            return false;
        }
    }
    if (!QFile::copy(sourcePath, destPath)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("复制失败 %1 -> %2").arg(sourcePath, destPath);
        }
        return false;
    }
    return true;
}

}  // namespace

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

    const auto& orbbec = bundle.orbbecResult;
    if (!vision::orbbecCapturePayloadReady(orbbec)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Orbbec 采集结果无效：段 %1").arg(segmentIndex);
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

    const QString destPly = scan_tracking::orbbec_gemini::buildSegmentOrbbecPlyPath(
        runRoot, segmentIndex, taskId, timestamp);
    if (destPly.isEmpty() ||
        !copyCaptureFile(orbbec.pointCloudPlyPath, destPly, &firstError)) {
        recordFailure(QStringLiteral("Orbbec 点云落盘失败：段 %1").arg(segmentIndex));
    }

    if (!orbbec.depthRawPngPath.trimmed().isEmpty()) {
        const QString destRaw = scan_tracking::orbbec_gemini::buildSegmentOrbbecDepthRawPath(
            runRoot, segmentIndex, taskId, timestamp);
        if (destRaw.isEmpty() ||
            !copyCaptureFile(orbbec.depthRawPngPath, destRaw, &firstError)) {
            recordFailure(QStringLiteral("Orbbec 深度图落盘失败：段 %1").arg(segmentIndex));
        }
    }

    if (!orbbec.depthPreviewPngPath.trimmed().isEmpty()) {
        const QString destPreview = scan_tracking::orbbec_gemini::buildSegmentOrbbecDepthPreviewPath(
            runRoot, segmentIndex, taskId, timestamp);
        if (destPreview.isEmpty() ||
            !copyCaptureFile(orbbec.depthPreviewPngPath, destPreview, &firstError)) {
            recordFailure(QStringLiteral("Orbbec 深度预览落盘失败：段 %1").arg(segmentIndex));
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
        << QStringLiteral("段 %1 已落盘至 %2/orbbec").arg(segmentIndex).arg(runRoot);
    return true;
}

}  // namespace scan_tracking::flow_control
